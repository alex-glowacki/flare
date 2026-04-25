// =============================================================================
// FLARE — esp32_remote
// ESP-NOW transmitter: reads sticks + switches, sends FLARE_RC_Packet_t
// to the quad-side ESP32 at 50Hz.
//
// Pin assignments marked TBD — fill in once hardware is wired.
//
// UART wiring (Nano ESP32 → FK723M1, for future telemetry RX):
//   GPIO17 (TX, D4) → STM32 RX
//   GPIO18 (RX, D5) → STM32 TX  (unused in Phase 5, reserved)
//   GND             → GND
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <U8g2lib.h>

#include "flare_protocol.h"

// ---------------------------------------------------------------------------
// Target peer — quad-side ESP32 MAC address
// ---------------------------------------------------------------------------
static const uint8_t kQuadMac[6] = {0xE4, 0xB0, 0x63, 0xAF, 0x0F, 0x3C};

// ---------------------------------------------------------------------------
// Pin assignments — sticks and switches
// ---------------------------------------------------------------------------

// Analog stick axes (0-4095 ADC, 12-bit)
#define PIN_THROTTLE      A0    // gimbal 1 Y-axis
#define PIN_YAW           A1    // gimbal 1 X-axis
#define PIN_PITCH         A2    // gimbal 2 Y-axis
#define PIN_ROLL          A3    // gimbal 2 X-axis

// Digital switches (INPUT_PULLUP - LOW = active)
#define PIN_ARM_SWITCH    D2    // arming toggle switch
#define PIN_MODE_SWITCH   D3    // angle/acro toggle switch

// ---------------------------------------------------------------------------
// OLED pin assignments — HiLetgo 2.42" SSD1309 SPI
// ---------------------------------------------------------------------------
#define PIN_OLED_CS       D10   // chip select
#define PIN_OLED_DC       D6    // data/command
#define PIN_OLED_RST      D7    // reset

// ---------------------------------------------------------------------------
// U8g2 — SSD1309 128x64, hardware SPI
// Constructor: U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI(rotation, cs, dc, reset)
// ---------------------------------------------------------------------------
static U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(
  U8G2_R0,
  PIN_OLED_CS,
  PIN_OLED_DC,
  PIN_OLED_RST
);

// ---------------------------------------------------------------------------
// Transmit rate
// ---------------------------------------------------------------------------
#define TX_RATE_HZ        50
#define TX_INTERVAL_MS    (1000 / TX_RATE_HZ)   // 20ms

// ---------------------------------------------------------------------------
// Display update rate — 10Hz is plenty for a status display
// ---------------------------------------------------------------------------
#define DISP_INTERVAL_MS 100

// ---------------------------------------------------------------------------
// ADC calibration
// ---------------------------------------------------------------------------
#define ADC_MIN       100
#define ADC_MAX       4000
#define ADC_DEADBAND  40

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;

// Last packet state - used by display update
static FLARE_RC_Packet_t last_pkt = {};

// ---------------------------------------------------------------------------
// ESP-NOW send callback
// ---------------------------------------------------------------------------
static void on_packet_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  if (status == ESP_NOW_SEND_SUCCESS) {
    packets_sent++;
  } else {
    packets_failed++;
  }
}

// ---------------------------------------------------------------------------
// map_stick()
// ---------------------------------------------------------------------------
static uint16_t map_stick(uint16_t raw, bool reversed) {
  raw = constrain(raw, ADC_MIN, ADC_MAX);

  uint16_t center = (ADC_MIN + ADC_MAX) / 2;

  if (abs((int)raw - (int)center) < ADC_DEADBAND) {
    raw = center;
  }

  uint16_t mapped = map(raw, ADC_MIN, ADC_MAX, FLARE_CH_MIN, FLARE_CH_MAX);
  mapped = constrain(mapped, FLARE_CH_MIN, FLARE_CH_MAX);

  if (reversed) {
    mapped = FLARE_CH_MIN + FLARE_CH_MAX - mapped;
  }

  return mapped;
}

// ---------------------------------------------------------------------------
// update_display()
//
// Renders the current remote state to the SSD1309 OLED.
//
// Layout (128x64):
//   Line 1: FLARE  [ARMED/DISARMED]
//   Line 2: Mode:  ACRO / ANGLE
//   Line 3: Thr:   XXX%
//   Line 4: TX ok/fail counters
// ---------------------------------------------------------------------------
static void update_display() {
  // Throttle percent: map 1000-2000 → 0-100
  uint8_t thr_pct = (uint8_t)map(last_pkt.throttle, FLARE_CH_MIN, FLARE_CH_MAX, 0, 100);

  bool armed = (last_pkt.armed == FLARE_ARMED);
  bool acro = (last_pkt.mode == FLARE_MODE_ACRO);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  // Row 1 - identity + arm state
  u8g2.setCursor(0, 12);
  u8g2.print("FLARE");
  u8g2.setCursor(48, 12);
  if (armed) {
    u8g2.print("** ARMED **");
  } else {
    u8g2.print("DISARMED");
  }

  // Divider
  u8g2.drawHLine(0, 15, 128);

  // Row 2 - flight mode
  u8g2.setCursor(0, 28);
  u8g2.print("Mode: ");
  u8g2.print(acro ? "ACRO" : "ANGLE");

  // Row 3 - throttle
  u8g2.setCursor(0, 42);
  u8g2.print("Thr: ");
  u8g2.print(thr_pct);
  u8g2.print("%");

  // Row 4 - TX diagnostics
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
  pkt.throttle = map_stick(analogRead(PIN_THROTTLE), false);
  pkt.roll = map_stick(analogRead(PIN_ROLL), false);
  pkt.pitch = map_stick(analogRead(PIN_PITCH), false);
  pkt.yaw = map_stick(analogRead(PIN_YAW), false);

  pkt.armed = (digitalRead(PIN_ARM_SWITCH) == LOW) ? FLARE_ARMED : FLARE_DISARMED;
  pkt.mode = (digitalRead(PIN_MODE_SWITCH) == LOW) ? FLARE_MODE_ACRO : FLARE_MODE_ANGLE;

  pkt.reserved[0] = 0;
  pkt.reserved[1] = 0;
  pkt.checksum = flare_checksum(&pkt);

  esp_now_send(kQuadMac, (const uint8_t *)&pkt, FLARE_PACKET_SIZE);

  // Cache for display
  last_pkt = pkt;
}

// ---------------------------------------------------------------------------
// Setup
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
  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

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
    while (true) { delay(1000); }
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
    while (true) { delay(1000); }
  }

  Serial.println("[FLARE] ESP-NOW transmitter ready");
  Serial.printf("[FLARE] Transmitting to %02X:%02X:%02X:%02X:%02X:%02X at %dHz\n",
      kQuadMac[0], kQuadMac[1], kQuadMac[2], kQuadMac[3], kQuadMac[4],
      kQuadMac[5], TX_RATE_HZ);

  // Boot complete - show ready screen
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("FLARE ready");
  u8g2.sendBuffer();
  delay(500);
}

// ---------------------------------------------------------------------------
// Loop — 50Hz transmit + 10Hz display + 5s serial diagnostics
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
    Serial.printf("[FLARE] tx_ok=%lu  tx_fail=%lu\n", packets_sent, packets_failed);
  }
}