/**
 * @file    main.cpp
 * @brief   FLARE — ESP32 quad-side firmware stub
 *          Receives ESP-NOW packets from the remote and forwards
 *          channel data to the STM32 FC over UART.
 *
 * Phase 4 target. Currently a no-op stub.
 */

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("[esp32_quad] boot ok");
}

void loop() {
  // Phase 4: receive ESP-NOW, parse channels, forward over UART to STM32
}