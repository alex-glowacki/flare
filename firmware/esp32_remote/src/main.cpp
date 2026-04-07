/**
 * @file    main.cpp
 * @brief   FLARE — ESP32 remote-side firmware stub
 *          Reads gimbal stick ADC, sends channel data over ESP-NOW,
 *          and drives the SSD1306 OLED telemetry display.
 *
 * Phase 5 target. Currently a no-op stub.
 */

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("[esp32_remote] boot ok");
}

void loop() {
  // Phase 5: read sticks, transmit ESP-NOW, update OLED
}