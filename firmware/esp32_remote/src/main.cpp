// =============================================================================
// FLARE — esp32_remote
// ESP-NOW transmitter: reads sticks + switches, sends FLARE_RC_Packet_t
// to the quad-side ESP32 at 50Hz.
//
// UART wiring (Nano ESP32 → FK723M1, for future telemetry RX):
//   GPIO17 (TX, D4) → STM32 RX
//   GPIO18 (RX, D5) → STM32 TX  (unused in Phase 5, reserved)
//   GND             → GND
//
// Display states:
//   SPLASH  → shown for SPLASH_DURATION_MS on power-on
//   IDLE    → shown while disarmed (mode == SAFE)
//   FLYING  → shown while armed (mode == ANGLE or ACRO)
// =============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>

#include "flare_protocol.h"

// ---------------------------------------------------------------------------
// Target peer — quad-side ESP32 MAC address
// ---------------------------------------------------------------------------
static const uint8_t kQuadMac[6] = {0xD4, 0xE9, 0xF4, 0xE6, 0xE9, 0x90};

// ---------------------------------------------------------------------------
// Pin assignments — sticks and switches
//
// Both gimbals are mounted 90° rotated from their default orientation.
// Pin assignments are swapped here at the source to compensate, so all
// downstream code (packet fields, STM32 mapping) remains semantically correct.
//
//   Left gimbal:  throttle ← X-axis (A1), yaw ← Y-axis (A0)
//   Right gimbal: roll     ← Y-axis (A2), pitch ← X-axis (A3)
// ---------------------------------------------------------------------------
#define PIN_THROTTLE A1  // left gimbal X-axis  (was A0 pre-rotation)
#define PIN_YAW A0       // left gimbal Y-axis  (was A1 pre-rotation)
#define PIN_PITCH A3     // right gimbal X-axis (was A2 pre-rotation)
#define PIN_ROLL A2      // right gimbal Y-axis (was A3 pre-rotation)

#define PIN_MODE_SWITCH_A D3  // mode switch terminal 1 (UP)   — LOW = ANGLE
#define PIN_MODE_SWITCH_B D4  // mode switch terminal 3 (DOWN) — LOW = ACRO
//
// Mode switch truth table:
//   D3=LOW,  D4=HIGH → FLARE_MODE_ANGLE (UP position)
//   D3=HIGH, D4=HIGH → FLARE_MODE_SAFE  (CENTER — forces disarm on FC)
//   D3=HIGH, D4=LOW  → FLARE_MODE_ACRO  (DOWN position)

// ---------------------------------------------------------------------------
// OLED — HiLetgo 2.42" SSD1309 SPI
// ---------------------------------------------------------------------------
#define PIN_OLED_CS D10
#define PIN_OLED_DC D6
#define PIN_OLED_RST D7

static U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS,
                                                    PIN_OLED_DC, PIN_OLED_RST);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define TX_RATE_HZ 50
#define TX_INTERVAL_MS (1000 / TX_RATE_HZ)  // 20ms
#define DISP_INTERVAL_MS 100                // 10Hz display refresh
#define SPLASH_DURATION_MS 2000             // 2s splash on boot

// ---------------------------------------------------------------------------
// ADC calibration — per axis, measured on hardware
// ---------------------------------------------------------------------------
struct AxisCal {
    uint16_t min;
    uint16_t center;
    uint16_t max;
    uint16_t deadband;
    bool reversed;
};

static const AxisCal kThrottle = {326, 0, 3869, 0, false};
static const AxisCal kYaw = {253, 1949, 3418, 60, false};
static const AxisCal kPitch = {38, 1803, 3129, 40, false};
static const AxisCal kRoll = {251, 1623, 3647, 40, false};

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;

// Last sent packet — read by display update
static FLARE_RC_Packet_t last_pkt = {};

// ---------------------------------------------------------------------------
// Display state machine
// ---------------------------------------------------------------------------
enum DisplayState { DISP_SPLASH, DISP_IDLE, DISP_FLYING };

static DisplayState disp_state = DISP_SPLASH;
static uint32_t splash_start = 0;

// ---------------------------------------------------------------------------
// Telemetry placeholders — replaced by real downlink values in future
// ---------------------------------------------------------------------------
static uint8_t telem_sats = 0;
static uint16_t telem_heading = 0;  // degrees
static int32_t telem_alt_dm = 0;    // decimetres (alt_m * 10)
static uint16_t telem_spd_cms = 0;  // cm/s      (spd_ms * 100)

// ---------------------------------------------------------------------------
// ESP-NOW send callback
// ---------------------------------------------------------------------------
static void on_packet_sent(const uint8_t* mac_addr,
                           esp_now_send_status_t status) {
    (void)mac_addr;
    if (status == ESP_NOW_SEND_SUCCESS)
        packets_sent++;
    else
        packets_failed++;
}

// ---------------------------------------------------------------------------
// map_stick()
// ---------------------------------------------------------------------------
static uint16_t map_stick(uint16_t raw, const AxisCal& cal) {
    raw = constrain(raw, cal.min, cal.max);

    uint16_t mapped;

    if (cal.center == 0) {
        mapped =
            (uint16_t)map(raw, cal.min, cal.max, FLARE_CH_MIN, FLARE_CH_MAX);
    } else {
        if (abs((int)raw - (int)cal.center) < (int)cal.deadband) {
            raw = cal.center;
        }
        if (raw <= cal.center) {
            mapped = (uint16_t)map(raw, cal.min, cal.center, FLARE_CH_MIN,
                                   FLARE_CH_MID);
        } else {
            mapped = (uint16_t)map(raw, cal.center, cal.max, FLARE_CH_MID,
                                   FLARE_CH_MAX);
        }
    }

    mapped = constrain(mapped, FLARE_CH_MIN, FLARE_CH_MAX);
    if (cal.reversed) mapped = FLARE_CH_MIN + FLARE_CH_MAX - mapped;
    return mapped;
}

// ---------------------------------------------------------------------------
// read_mode() / read_mode_debounced()
// ---------------------------------------------------------------------------
static uint8_t read_mode() {
    bool a = (digitalRead(PIN_MODE_SWITCH_A) == LOW);
    bool b = (digitalRead(PIN_MODE_SWITCH_B) == LOW);
    if (a) return FLARE_MODE_ANGLE;
    if (b) return FLARE_MODE_ACRO;
    return FLARE_MODE_SAFE;
}

#define DEBOUNCE_MS 20

static uint8_t read_mode_debounced() {
    static uint8_t last_stable = FLARE_MODE_SAFE;
    static uint8_t candidate = FLARE_MODE_SAFE;
    static uint32_t changed_at = 0;

    uint8_t current = read_mode();
    if (current != candidate) {
        candidate = current;
        changed_at = millis();
    }
    if ((millis() - changed_at) >= DEBOUNCE_MS) {
        last_stable = candidate;
    }
    return last_stable;
}

// ---------------------------------------------------------------------------
// mode_str()
// ---------------------------------------------------------------------------
static const char* mode_str(uint8_t mode) {
    switch (mode) {
        case FLARE_MODE_ANGLE:
            return "ANGLE";
        case FLARE_MODE_ACRO:
            return "ACRO";
        default:
            return "SAFE";
    }
}

// ---------------------------------------------------------------------------
// draw_splash()
// ---------------------------------------------------------------------------
static void draw_splash() {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_logisoso16_tr);
    uint8_t w = u8g2.getStrWidth("FLARE");
    u8g2.setCursor((128 - w) / 2, 28);
    u8g2.print("FLARE");

    u8g2.setFont(u8g2_font_6x12_tr);
    w = u8g2.getStrWidth("Flight Lab");
    u8g2.setCursor((128 - w) / 2, 44);
    u8g2.print("Flight Lab");

    w = u8g2.getStrWidth("v0.1.0");
    u8g2.setCursor((128 - w) / 2, 57);
    u8g2.print("v0.1.0");

    u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// draw_idle()
//
//   DISARMED  |  SAFE
//   SAT:  0   |  0°
//   ALT:  --m
//   BATT: --.-V   RSSI:---%
// ---------------------------------------------------------------------------
static void draw_idle() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    uint8_t mode = last_pkt.mode;

    // Row 1 — arm state | mode
    u8g2.setCursor(0, 12);
    u8g2.print("DISARMED");
    u8g2.setCursor(78, 12);
    u8g2.print(mode_str(mode));

    u8g2.drawHLine(0, 14, 128);
    u8g2.drawVLine(72, 0, 14);

    // Row 2 — satellites | heading
    u8g2.setCursor(0, 26);
    u8g2.print("SAT: ");
    u8g2.print(telem_sats);
    u8g2.setCursor(78, 26);
    u8g2.print(telem_heading);
    u8g2.print((char)176);

    // Row 3 — altitude
    u8g2.setCursor(0, 40);
    u8g2.print("ALT: ");
    if (telem_alt_dm == 0) {
        u8g2.print("--");
    } else {
        u8g2.print(telem_alt_dm / 10);
        u8g2.print(".");
        u8g2.print(telem_alt_dm % 10);
    }
    u8g2.print("m");

    // Row 4 — battery + RSSI placeholders
    u8g2.setCursor(0, 54);
    u8g2.print("BATT:--.-V  RSSI:---%");

    u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// draw_flying()
//
//   ARMED   |  ANGLE
//   SAT:  0 |  0°
//   ALT: --m   SPD:--
//   THR 50% [1500]
//   BATT: --.-V
// ---------------------------------------------------------------------------
static void draw_flying() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    uint8_t mode = last_pkt.mode;
    uint16_t thr_raw = last_pkt.throttle;
    uint8_t thr_pct = (uint8_t)map(thr_raw, FLARE_CH_MIN, FLARE_CH_MAX, 0, 100);

    // Row 1 — arm state | mode
    u8g2.setCursor(0, 12);
    u8g2.print("ARMED");
    u8g2.setCursor(78, 12);
    u8g2.print(mode_str(mode));

    u8g2.drawHLine(0, 14, 128);
    u8g2.drawVLine(72, 0, 14);

    // Row 2 — satellites | heading
    u8g2.setCursor(0, 26);
    u8g2.print("SAT:");
    u8g2.print(telem_sats);
    u8g2.setCursor(78, 26);
    u8g2.print(telem_heading);
    u8g2.print((char)176);

    // Row 3 — altitude | speed
    u8g2.setCursor(0, 38);
    u8g2.print("ALT:");
    if (telem_alt_dm == 0) {
        u8g2.print("--");
    } else {
        u8g2.print(telem_alt_dm / 10);
        u8g2.print(".");
        u8g2.print(telem_alt_dm % 10);
    }
    u8g2.print("m");

    u8g2.setCursor(66, 38);
    u8g2.print("SPD:");
    if (telem_spd_cms == 0) {
        u8g2.print("--");
    } else {
        u8g2.print(telem_spd_cms / 100);
        u8g2.print(".");
        u8g2.print((telem_spd_cms % 100) / 10);
    }

    // Row 4 — throttle % and raw RC value
    u8g2.setCursor(0, 50);
    u8g2.print("THR ");
    u8g2.print(thr_pct);
    u8g2.print("% [");
    u8g2.print(thr_raw);
    u8g2.print("]");

    // Row 5 — battery placeholder
    u8g2.setCursor(0, 62);
    u8g2.print("BATT: --.-V");

    u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// update_display()
//
// armed = mode is ANGLE or ACRO (matches read_and_send() logic exactly)
// ---------------------------------------------------------------------------
static void update_display() {
    bool armed = (last_pkt.armed == FLARE_ARMED);

    switch (disp_state) {
        case DISP_SPLASH:
            if ((millis() - splash_start) >= SPLASH_DURATION_MS) {
                disp_state = armed ? DISP_FLYING : DISP_IDLE;
                armed ? draw_flying() : draw_idle();
            }
            break;

        case DISP_IDLE:
            if (armed) {
                disp_state = DISP_FLYING;
                draw_flying();
            } else {
                draw_idle();
            }
            break;

        case DISP_FLYING:
            if (!armed) {
                disp_state = DISP_IDLE;
                draw_idle();
            } else {
                draw_flying();
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// read_and_send()
// ---------------------------------------------------------------------------
static void read_and_send() {
    FLARE_RC_Packet_t pkt = {};

    pkt.magic = FLARE_PACKET_MAGIC;
    pkt.throttle = map_stick(analogRead(PIN_THROTTLE), kThrottle);
    pkt.yaw = map_stick(analogRead(PIN_YAW), kYaw);
    pkt.pitch = map_stick(analogRead(PIN_PITCH), kPitch);
    pkt.roll = map_stick(analogRead(PIN_ROLL), kRoll);

    pkt.mode = read_mode_debounced();
    pkt.armed = (pkt.mode != FLARE_MODE_SAFE) ? FLARE_ARMED : FLARE_DISARMED;

    pkt.reserved[0] = 0;
    pkt.reserved[1] = 0;
    pkt.checksum = flare_checksum(&pkt);

    esp_now_send(kQuadMac, (const uint8_t*)&pkt, FLARE_PACKET_SIZE);
    last_pkt = pkt;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[FLARE] esp32_remote booting...");

    // Switch pins
    pinMode(PIN_MODE_SWITCH_A, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH_B, INPUT_PULLUP);

    // OLED — draw splash immediately
    u8g2.begin();
    splash_start = millis();
    draw_splash();

    // ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("[FLARE] MAC address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[FLARE] ERROR: esp_now_init() failed - halting");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.setCursor(0, 12);
        u8g2.print("ESP-NOW FAIL");
        u8g2.sendBuffer();
        while (true) {
            delay(1000);
        }
    }

    esp_now_register_send_cb(on_packet_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kQuadMac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[FLARE] ERROR: esp_now_add_peer() failed - halting");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.setCursor(0, 12);
        u8g2.print("PEER ADD FAIL");
        u8g2.sendBuffer();
        while (true) {
            delay(1000);
        }
    }

    Serial.println("[FLARE] ESP-NOW transmitter ready");
    Serial.printf(
        "[FLARE] Transmitting to %02X:%02X:%02X:%02X:%02X:%02X at %dHz\n",
        kQuadMac[0], kQuadMac[1], kQuadMac[2], kQuadMac[3], kQuadMac[4],
        kQuadMac[5], TX_RATE_HZ);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t last_tx_ms = 0;
    static uint32_t last_disp_ms = 0;
    static uint32_t last_report_ms = 0;
    uint32_t now = millis();

    if (now - last_tx_ms >= TX_INTERVAL_MS) {
        last_tx_ms = now;
        read_and_send();
    }

    if (now - last_disp_ms >= DISP_INTERVAL_MS) {
        last_disp_ms = now;
        update_display();
    }

    if (now - last_report_ms >= 5000) {
        last_report_ms = now;
        Serial.printf("[FLARE] tx_ok=%lu  tx_fail=%lu\n", packets_sent,
                      packets_failed);
    }
}