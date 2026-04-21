// =============================================================================
// FLARE — esp32_quad
// ESP-NOW receiver → UART bridge to STM32H723
//
// Receives FLARE_RC_Packet_t from the remote ESP32 over ESP-NOW and forwards
// valid packets to the STM32 flight controller over UART2.
//
// UART wiring (Nano ESP32 → FK723M1):
//   GPIO17 (TX, D4) → STM32 RX  (PA10, USART1)
//   GPIO18 (RX, D5) → STM32 TX  (PA9,  USART1)
//   GND             → GND
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "flare_protocol.h"

// ---------------------------------------------------------------------------
// UART to STM32
// HardwareSerial(1) = UART2 on ESP32-S3
// GPIO17 = TX (Arduino D4), GPIO18 = RX (Arduino D5)
// ---------------------------------------------------------------------------
#define STM32_UART_BAUD     115200
#define STM32_UART_TX_PIN   17      // D4 on Nano ESP32
#define STM32_UART_RX_PIN   18      // D5 on Nano ESP32

static HardwareSerial stm32_uart(1);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
static uint32_t packets_received = 0;
static uint32_t packets_rejected = 0;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
//
// Uses the pre-ESP-IDF-5.x signature compatible with framework-arduinoespressif32
// @ 3.x (ESP32 Arduino core 2.x). The mac_addr parameter is unused — we only
// care about the payload.
//
// Called from the ESP-NOW task (not Arduino loop). Keep it short — validate,
// copy, forward. No Serial.print inside; that can block.
// ---------------------------------------------------------------------------
static void on_packet_received(const uint8_t *mac_addr,
                                const uint8_t *data,
                                int len)
{
    (void)mac_addr;     // unused — suppress -Wunused-parameter

    // Length check
    if (len != FLARE_PACKET_SIZE) {
        packets_rejected++;
        return;
    }

    const FLARE_RC_Packet_t *pkt = (const FLARE_RC_Packet_t *)data;

    // Magic byte check
    if (pkt->magic != FLARE_PACKET_MAGIC) {
        packets_rejected++;
        return;
    }

    // Checksum check
    if (pkt->checksum != flare_checksum(pkt)) {
        packets_rejected++;
        return;
    }

    packets_received++;

    // Forward raw packet bytes to STM32 over UART
    stm32_uart.write(data, FLARE_PACKET_SIZE);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    // Debug serial (USB, monitor)
    Serial.begin(115200);
    delay(500);
    Serial.println("[FLARE] esp32_quad booting...");

    // UART to STM32
    stm32_uart.begin(STM32_UART_BAUD, SERIAL_8N1,
                     STM32_UART_RX_PIN, STM32_UART_TX_PIN);
    Serial.println("[FLARE] STM32 UART ready");

    // ESP-NOW requires Wi-Fi initialised in Station mode (no AP connection)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("[FLARE] MAC address: ");
    Serial.println(WiFi.macAddress());

    // Initialise ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FLARE] ERROR: esp_now_init() failed — halting");
        while (true) { delay(1000); }
    }

    esp_now_register_recv_cb(on_packet_received);

    Serial.println("[FLARE] ESP-NOW receiver ready");
    Serial.println("[FLARE] Waiting for packets...");
}

// ---------------------------------------------------------------------------
// Loop — diagnostics only
// All real work happens in on_packet_received()
// ---------------------------------------------------------------------------
void loop()
{
    static uint32_t last_report_ms = 0;
    uint32_t now = millis();

    // Print packet stats every 5 seconds
    if (now - last_report_ms >= 5000) {
        last_report_ms = now;
        Serial.printf("[FLARE] rx_ok=%lu  rx_bad=%lu\n",
                      packets_received, packets_rejected);
    }
}