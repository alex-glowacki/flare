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

#include "flare_protocol.h"

// ---------------------------------------------------------------------------
// Target peer — quad-side ESP32 MAC address
// ---------------------------------------------------------------------------
static const uint8_t kQuadMac[6] = {0xE4, 0xB0, 0x63, 0xAF, 0x0F, 0x3C};

// ---------------------------------------------------------------------------
// Pin assignments — TBD, fill in once sticks and switches are wired
// ---------------------------------------------------------------------------

// Analog stick axes (0–4095 ADC, 12-bit)
// Each M7 gimbal has two hall sensor outputs: one per axis
#define PIN_THROTTLE A0 // TBD - gimbal 1 Y-axis
#define PIN_YAW A1      // TBD - gimbal 1 X-axis
#define PIN_PITCH A2    // TBD - gimbal 2 Y-axis
#define PIN_ROLL A3     // TBD - gimbal 2 X-axis

// Digital switches (INPUT_PULLUP - LOW = active)
#define PIN_ARM_SWITCH D2  // TBD - arming toggle switch
#define PIN_MODE_SWITCH D3 // TBD - angle/acro toggle switch

// ---------------------------------------------------------------------------
// Transmit rate
// ---------------------------------------------------------------------------
#define TX_RATE_HZ 50
#define TX_INTERVAL_MS (1000 / TX_RATE_HZ) // 20ms

// ---------------------------------------------------------------------------
// ADC calibration
// Raw ADC range from M7 hall sensors — measure with Serial output and adjust
// Center (mid) values may not be exactly 2048 due to mechanical centering
// ---------------------------------------------------------------------------
#define ADC_MIN 100     // TBD — measure at stick minimum
#define ADC_MAX 4000    // TBD — measure at stick maximum
#define ADC_DEADBAND 40 // raw ADC counts either side of center

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;

// ---------------------------------------------------------------------------
// ESP-NOW send callback
// Called after each transmission attempt. Not used for flow control here —
// just increments counters for the diagnostic printout.
// ---------------------------------------------------------------------------
static void on_packet_sent(const uint8_t *mac_addr,
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
// Maps a raw 12-bit ADC reading to a PWM-style channel value (1000–2000).
// Applies a deadband around center to suppress hall sensor noise at rest.
//
// Parameters:
//   raw       — ADC reading (0–4095)
//   reversed  — true to invert the axis direction
// ---------------------------------------------------------------------------
static uint16_t map_stick(uint16_t raw, bool reversed) {
  // Clamp to calibrated range
  raw = constrain(raw, ADC_MIN, ADC_MAX);

  uint16_t center = (ADC_MIN + ADC_MAX) / 2;

  // Apply deadband around center
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
// read_and_send()
//
// Reads all stick axes and switches, builds a FLARE_RC_Packet_t, computes
// the CRC-8 checksum, and transmits via ESP-NOW.
// ---------------------------------------------------------------------------
static void read_and_send() {
  FLARE_RC_Packet_t pkt = {};

  pkt.magic = FLARE_PACKET_MAGIC;
  pkt.throttle = map_stick(analogRead(PIN_THROTTLE), false);
  pkt.roll = map_stick(analogRead(PIN_ROLL), false);
  pkt.pitch = map_stick(analogRead(PIN_PITCH), false);
  pkt.yaw = map_stick(analogRead(PIN_YAW), false);

  // Switches - LOW = switch closed (INPUT_PULLUP)
  pkt.armed =
      (digitalRead(PIN_ARM_SWITCH) == LOW) ? FLARE_ARMED : FLARE_DISARMED;
  pkt.mode = (digitalRead(PIN_MODE_SWITCH) == LOW) ? FLARE_MODE_ACRO
                                                   : FLARE_MODE_ANGLE;

  pkt.reserved[0] = 0;
  pkt.reserved[1] = 0;
  pkt.checksum = flare_checksum(&pkt);

  esp_now_send(kQuadMac, (const uint8_t *)&pkt, FLARE_PACKET_SIZE);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[FLARE] esp32_remote booting...");

  // Stick pins are analog - no pinMode needed for analogRead
  // Switch pins: pulled high internally, switch connects to GND
  pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

  // ESP-NOW requires Wi-Fi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("[FLARE] MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FLARE] ERROR: esp_now_init() failed - halting");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_send_cb(on_packet_sent);

  // Register quad as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kQuadMac, 6);
  peer.channel = 0; // 0 = use current Wi-Fi channel
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[FLARE] ERROR: esp_now_add_peer() failed - halting");
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
// Loop — 50Hz transmit + 5s diagnostics
// ---------------------------------------------------------------------------
void loop() {
  static uint32_t last_tx_ms = 0;
  static uint32_t last_report_ms = 0;
  uint32_t now = millis();

  if (now - last_tx_ms >= TX_INTERVAL_MS) {
    last_tx_ms = now;
    read_and_send();
  }

  if (now - last_report_ms >= 5000) {
    last_report_ms = now;
    Serial.printf("[FLARE] tx_ok=%lu  tx_fail=%lu\n", packets_sent,
                  packets_failed);
  }
}