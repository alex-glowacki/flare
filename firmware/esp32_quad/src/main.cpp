// =============================================================================
// firmware/esp32_quad/src/main.cpp
// FLARE — ESP32 quad-side firmware
// ESP-NOW receiver → UART bridge to STM32 FC
//
// Board:  ESP32-WROOM-32, 30-pin, CP2102, Type-C
// Chip:   ESP32 (Xtensa LX6 dual-core)
//
// UART wiring (ESP32 → FK723M1-ZGT6):
//   GPIO17 (TX2) → STM32 PA3 (USART2 RX)
//   GND          → GND
// =============================================================================

#include "flare_protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// UART2 to STM32
// GPIO17 = TX2, GPIO16 = RX2 — safe general-purpose pins, not boot-sensitive
// UART0 (GPIO1/GPIO3) is occupied by CP2102 = USB debug (Serial)
// ---------------------------------------------------------------------------
#define STM32_UART_TX_PIN 17 // GPIO17 = TX2
#define STM32_UART_RX_PIN 16 // GPIO16 = RX2
#define STM32_UART_BAUD 115200

static HardwareSerial stm32_uart(2); // UART2 peripheral

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static FLARE_RC_Packet_t latest_packet;
static volatile uint32_t rx_ok = 0;
static volatile uint32_t rx_rejected = 0;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
// Old SDK signature (framework-arduinoespressif32 <= 3.x):
//   (const uint8_t *mac_addr, const uint8_t *data, int len)
// ---------------------------------------------------------------------------
static void on_packet_received(const uint8_t *mac_addr, const uint8_t *data,
                               int len) {
  (void)mac_addr;

  if (len != (int)sizeof(FLARE_RC_Packet_t)) {
    rx_rejected++;
    return;
  }

  FLARE_RC_Packet_t pkt;
  memcpy(&pkt, data, sizeof(FLARE_RC_Packet_t));

  if (pkt.magic != FLARE_PACKET_MAGIC) {
    rx_rejected++;
    return;
  }
  if (pkt.checksum != flare_checksum(&pkt)) {
    rx_rejected++;
    return;
  }

  memcpy(&latest_packet, &pkt, sizeof(FLARE_RC_Packet_t));
  rx_ok++;

  stm32_uart.write((const uint8_t *)&latest_packet, sizeof(FLARE_RC_Packet_t));
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  // USB debug serial - CP2102 on UART0 (GPIO1/GPIO3), maps to Serial
  Serial.begin(115200);
  delay(500);
  Serial.println("[QUAD] step 1 - boot ok (ESP32-WROOM-32)");

  // Safe initial packet: disarmed, sticks centered, throttle low
  memset(&latest_packet, 0, sizeof(latest_packet));
  latest_packet.magic = FLARE_PACKET_MAGIC;
  latest_packet.throttle = FLARE_CH_MID;
  latest_packet.roll = FLARE_CH_MID;
  latest_packet.pitch = FLARE_CH_MID;
  latest_packet.yaw = FLARE_CH_MID;
  latest_packet.armed = FLARE_DISARMED;

  stm32_uart.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX_PIN,
                   STM32_UART_TX_PIN);
  Serial.printf("[QUAD] step 2 - UART2 ok TX=GPIO%d RX=GPIO%d\n",
                STM32_UART_TX_PIN, STM32_UART_RX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("[QUAD] step 3 - WiFi ok MAC=%s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[QUAD] step 4 - ESP-NOW FAILED - halting");
    while (true) {
      delay(1000);
    }
  }
  esp_now_register_recv_cb(on_packet_received);
  Serial.println("[QUAD] step 4 - ESP-NOW ok - waiting for packets");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  static uint32_t last_print = 0;

  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.printf("[QUAD] rx_ok=%-5lu rejected=%-5lu | "
                  "thr=%4u roll=%4u pitch=%4u yaw=%d4u arm=%u\n",
                  rx_ok, rx_rejected, latest_packet.throttle,
                  latest_packet.roll, latest_packet.pitch, latest_packet.yaw,
                  latest_packet.armed);
  }
}