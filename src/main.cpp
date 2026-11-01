#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello, EmergingConstellation!");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED on");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED off");
  delay(1000);
}