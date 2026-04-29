#include "flare_protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Serial1 routes to A0 (TX=GPIO17) and A1 (RX=GPIO18) on this board variant
#define STM32_UART_TX_PIN 17 // GPIO17 = A0
#define STM32_UART_RX_PIN 18 // GPIO18 = A1

static FLARE_RC_Packet_t latest_packet;
static volatile uint32_t rx_ok = 0;

static void on_packet_received(const uint8_t *mac_addr, const uint8_t *data,
                               int len) {
  if (len != (int)sizeof(FLARE_RC_Packet_t))
    return;

  FLARE_RC_Packet_t pkt;
  memcpy(&pkt, data, sizeof(FLARE_RC_Packet_t));

  if (pkt.magic != FLARE_PACKET_MAGIC)
    return;
  if (pkt.checksum != flare_checksum(&pkt))
    return;

  memcpy(&latest_packet, &pkt, sizeof(FLARE_RC_Packet_t));
  rx_ok++;

  Serial1.write((const uint8_t *)&latest_packet, sizeof(FLARE_RC_Packet_t));
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("[QUAD] step 1 — boot ok");

  memset(&latest_packet, 0, sizeof(latest_packet));
  latest_packet.magic = FLARE_PACKET_MAGIC;
  latest_packet.throttle = FLARE_CH_MIN;
  latest_packet.roll = FLARE_CH_MID;
  latest_packet.pitch = FLARE_CH_MID;
  latest_packet.yaw = FLARE_CH_MID;
  latest_packet.armed = FLARE_DISARMED;

  // Explicit pin assignment required — variant remaps GPIO17/18 to A0/A1
  // physically
  Serial1.begin(115200, SERIAL_8N1, STM32_UART_RX_PIN, STM32_UART_TX_PIN);
  Serial.printf("[QUAD] step 2 — UART ok (TX=GPIO%d=A0, RX=GPIO%d=A1)\n",
                STM32_UART_TX_PIN, STM32_UART_RX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("[QUAD] MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("[QUAD] step 3 — WiFi ok");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[QUAD] step 4 — ESP-NOW FAILED");
    while (1)
      delay(1000);
  }
  esp_now_register_recv_cb(on_packet_received);
  Serial.println("[QUAD] step 4 — ESP-NOW ok");
}

void loop() {
  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.printf(
        "[QUAD] rx_ok=%lu  thr=%u  roll=%u  pitch=%u  yaw=%u  arm=%u\n", rx_ok,
        latest_packet.throttle, latest_packet.roll, latest_packet.pitch,
        latest_packet.yaw, latest_packet.armed);
  }
}