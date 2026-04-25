// =============================================================================
// FLARE — esp32_remote
// ADC axis identification test — move sticks and watch Serial output
// =============================================================================

#include <Arduino.h>

#define PIN_A0 A0
#define PIN_A1 A1
#define PIN_A2 A2
#define PIN_A3 A3

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[FLARE] ADC test - move stick and watch values");
    Serial.println("A0 (L-Y) | A1 (L-X) | A2 (R-?) | A3 (R-?)");;
}

void loop() {
    uint16_t a0 = analogRead(PIN_A0);
    uint16_t a1 = analogRead(PIN_A1);
    uint16_t a2 = analogRead(PIN_A2);
    uint16_t a3 = analogRead(PIN_A3);

    Serial.printf("A0: %4d | A1: %4d | A2: %4d | A3: %4d\n", a0, a1, a2, a3);
    delay(100);
}