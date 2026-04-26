// =============================================================================
// FLARE — esp32_remote
// ESP-NOW transmitter: reads sticks + switches, sends FLARE_RC_Packet_t
// to the quad-side ESP32 at 50Hz.
//
// UART wiring (Nano ESP32 → FK723M1, for future telemetry RX):
//   GPIO17 (TX, D4) → STM32 RX
//   GPIO18 (RX, D5) → STM32 TX  (unused in Phase 5, reserved)
//   GND             → GND
// =============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>

#include "flare_protocol.h"

// ---------------------------------------------------------------------------
// Target peer — quad-side ESP32 MAC address
// ---------------------------------------------------------------------------
static const uint8_t kQuadMac[6] = {0xE4, 0xB0, 0x63, 0xAF, 0x0F, 0x3C};

// ---------------------------------------------------------------------------
// Pin assignments — sticks and switches
// ---------------------------------------------------------------------------

// Analog stick axes (0-4095 ADC, 12-bit)
#define PIN_THROTTLE A0  // left gimbal Y-axis
#define PIN_YAW A1       // left gimbal X-axis
#define PIN_PITCH A2     // right gimbal Y-axis
#define PIN_ROLL A3      // right gimbal X-axis

// Digital switches (INPUT_PULLUP — LOW = active)
#define PIN_ARM_SWITCH D12    // arming toggle switch — ON = armed
#define PIN_MODE_SWITCH_A D3  // mode switch terminal 1 (UP)   — LOW = ANGLE
#define PIN_MODE_SWITCH_B D4  // mode switch terminal 3 (DOWN) — LOW = ACRO
//
// Mode switch truth table:
//   D3=LOW,  D4=HIGH → FLARE_MODE_ANGLE (UP position)
//   D3=HIGH, D4=HIGH → FLARE_MODE_SAFE  (CENTER position — forces disarm on FC)
//   D3=HIGH, D4=LOW  → FLARE_MODE_ACRO  (DOWN position)

// ---------------------------------------------------------------------------
// OLED pin assignments — HiLetgo 2.42" SSD1309 SPI
// ---------------------------------------------------------------------------
#define PIN_OLED_CS D10
#define PIN_OLED_DC D6
#define PIN_OLED_RST D7

// ---------------------------------------------------------------------------
// U8g2 — SSD1309 128x64, hardware SPI
// ---------------------------------------------------------------------------
static U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS,
                                                    PIN_OLED_DC, PIN_OLED_RST);

// ---------------------------------------------------------------------------
// Transmit / display rates
// ---------------------------------------------------------------------------
#define TX_RATE_HZ 50
#define TX_INTERVAL_MS (1000 / TX_RATE_HZ)  // 20ms
#define DISP_INTERVAL_MS 100                // 10Hz

// ---------------------------------------------------------------------------
// ADC calibration — per axis, measured on hardware
//
// Throttle (A0): no spring center — ADC_CENTER unused for this axis
// Yaw     (A1): spring centered
// Pitch   (A2): spring centered
// Roll    (A3): spring centered
//
// Deadband applied around center in raw ADC counts.
// ---------------------------------------------------------------------------
struct AxisCal {
    uint16_t min;
    uint16_t center;
    uint16_t max;
    uint16_t deadband;
    bool reversed;
};

// min, center, max, deadband, reversed
static const AxisCal kThrottle = {265, 0, 3869, 0,
                                  false};  // A0 - no center/deadband
static const AxisCal kYaw = {253, 1756, 3418, 40, false};   // A1
static const AxisCal kPitch = {38, 1622, 3129, 40, false};  // A2
static const AxisCal kRoll = {251, 1799, 3647, 40, false};  // A3

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;

// Last packet state — used by display update
static FLARE_RC_Packet_t last_pkt = {};

// ---------------------------------------------------------------------------
// ESP-NOW send callback
// ---------------------------------------------------------------------------
static void on_packet_sent(const uint8_t* mac_addr,
                           esp_now_send_status_t status) {
    (void)mac_addr;
    if (status == ESP_NOW_SEND_SUCCESS) {
        packets_sent++;
    } else {
        packets_failed++;
    }
}

// ---------------------------------------------------------------------------
// map_stick()
//
// Maps a raw ADC reading to a PWM channel value (FLARE_CH_MIN–FLARE_CH_MAX).
// Applies per-axis deadband around center, then linearly maps the two halves
// (min→center, center→max) independently to handle asymmetric ADC ranges.
//
// For throttle: pass cal.center == 0 and cal.deadband == 0 — a straight
// single-segment map from min→max is used instead.
// ---------------------------------------------------------------------------
static uint16_t map_stick(uint16_t raw, const AxisCal& cal) {
    raw = constrain(raw, cal.min, cal.max);

    uint16_t mapped;

    if (cal.center == 0) {
        // Throttle path — no center, no deadband, straight map
        mapped =
            (uint16_t)map(raw, cal.min, cal.max, FLARE_CH_MIN, FLARE_CH_MAX);
    } else {
        // Apply deadband
        if (abs((int)raw - (int)cal.center) < (int)cal.deadband) {
            raw = cal.center;
        }

        // Map each half independently to handle asymmetric ADC ranges
        if (raw <= cal.center) {
            mapped = (uint16_t)map(raw, cal.min, cal.center, FLARE_CH_MIN,
                                   FLARE_CH_MID);
        } else {
            mapped = (uint16_t)map(raw, cal.center, cal.max, FLARE_CH_MID,
                                   FLARE_CH_MAX);
        }
    }

    mapped = constrain(mapped, FLARE_CH_MIN, FLARE_CH_MAX);

    if (cal.reversed) {
        mapped = FLARE_CH_MIN + FLARE_CH_MAX - mapped;
    }

    return mapped;
}

// ---------------------------------------------------------------------------
// read_mode()
//
// Reads the two mode switch pins and returns the appropriate FLARE_MODE_*
// constant. CENTER position (both HIGH) returns FLARE_MODE_SAFE which the
// FC treats as disarmed regardless of arm switch state.
// ---------------------------------------------------------------------------
static uint8_t read_mode() {
    bool a = (digitalRead(PIN_MODE_SWITCH_A) == LOW);  // UP position
    bool b = (digitalRead(PIN_MODE_SWITCH_B) == LOW);  // DOWN position

    if (a) return FLARE_MODE_ANGLE;
    if (b) return FLARE_MODE_ACRO;
    return FLARE_MODE_SAFE;
}

// ---------------------------------------------------------------------------
// update_display()
// ---------------------------------------------------------------------------
static void update_display() {
    uint8_t thr_pct =
        (uint8_t)map(last_pkt.throttle, FLARE_CH_MIN, FLARE_CH_MAX, 0, 100);

    bool armed = (last_pkt.armed == FLARE_ARMED);
    bool safe = (last_pkt.mode == FLARE_MODE_SAFE);
    bool acro = (last_pkt.mode == FLARE_MODE_ACRO);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    // Row 1 — identity + arm state
    u8g2.setCursor(0, 12);
    u8g2.print("FLARE");
    u8g2.setCursor(48, 12);
    // Show SAFE if mode switch is centered, regardless of arm switch
    u8g2.print(!armed ? "DISARMED" : (safe ? "SAFE" : "** ARMED **"));

    // Divider
    u8g2.drawHLine(0, 15, 128);

    // Row 2 — flight mode
    u8g2.setCursor(0, 28);
    u8g2.print("Mode: ");
    u8g2.print(safe ? "SAFE" : (acro ? "ACRO" : "ANGLE"));

    // Row 3 — throttle
    u8g2.setCursor(0, 42);
    u8g2.print("Thr: ");
    u8g2.print(thr_pct);
    u8g2.print("%");

    // Row 4 — TX diagnostics
    u8g2.setCursor(0, 56);
    u8g2.print("TX ");
    u8g2.print(packets_sent);
    u8g2.print("/");
    u8g2.print(packets_failed);

    u8g2.sendBuffer();
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

    pkt.armed =
        (digitalRead(PIN_ARM_SWITCH) == HIGH) ? FLARE_ARMED : FLARE_DISARMED;
    pkt.mode = read_mode();

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

    // OLED init
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("FLARE booting...");
    u8g2.sendBuffer();

    // Switch pins
    pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH_A, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH_B, INPUT_PULLUP);

    // ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("[FLARE] MAC address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[FLARE] ERROR: esp_now_init() failed - halting");
        u8g2.clearBuffer();
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

    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("FLARE ready");
    u8g2.sendBuffer();
    delay(500);
}

// ---------------------------------------------------------------------------
// loop() — 50Hz transmit + 10Hz display + 5s serial diagnostics
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