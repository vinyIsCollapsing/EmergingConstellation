#include <Arduino.h>

// Botao A do micro:bit v2 (definido em variants/BBCmicrobitV2/variant.h)
#define PIN_BTN PIN_BUTTON_A

const unsigned long DEBOUNCE_MS = 30;

int lastReading = HIGH;
int stableState = HIGH;
unsigned long lastChangeMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_BTN, INPUT_PULLUP);
  lastReading = digitalRead(PIN_BTN);
  stableState = lastReading;
  Serial.println("Teste: botao A (aperte para ver a mensagem)");
}

void loop() {
  int reading = digitalRead(PIN_BTN);

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = millis();
  }

  if (millis() - lastChangeMs >= DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      Serial.println("botao pressionado");
    }
  }
}