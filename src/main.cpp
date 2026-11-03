#include <Arduino.h>
#include "MicrobitV2-HHS.h"
#include "NRF52_Radio_library.h"

#define PIN_BTN PIN_BUTTON_A
const unsigned long DEBOUNCE_MS = 30;

NRF52_Radio MicrobitRadio = NRF52_Radio();

// Radio configs (must be the same in all boards)
const int GROUP = 42;     // 0 - 255
const int BAND = 50;      // 0 - 100
const int POWER = 7;      // 0 - 7

const uint8_t MY_ID = 1;  // Must be diffent for each boarc

unsigned long lastSend = 0;
const unsigned long sendIntervalMs = 2000;


int lastReading = HIGH;
int stableState = HIGH;
unsigned long lastChangeMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BTN, INPUT_PULLUP);
  lastReading = stableState = digitalRead(PIN_BTN);

  MicrobitRadio.enable();
  MicrobitRadio.setGroup(GROUP);
  MicrobitRadio.setFrequencyBand(BAND);
  MicrobitRadio.setTransmitPower(POWER);
  
  Serial.println("Radio pronto");
  Serial.println(MY_ID);
  Serial.println("Ready. Waiting for packets...");
}

void sendPulse() {
  FrameBuffer* myDataSent = new FrameBuffer();
  myDataSent->payload[0] = MY_ID;

  uint32_t timeStamp = millis();
  memcpy(&myDataSent->payload[1], &timeStamp, sizeof(timeStamp));

  myDataSent->length = 5;

  MicrobitRadio.send(myDataSent);
  delete myDataSent;

  Serial.print("TX: pulse sent by: ");
  Serial.print(MY_ID);
  Serial.print("  at t = ");
  Serial.println(timeStamp);
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
      sendPulse();
    }
  }

  // Recepção
  FrameBuffer *myDataReceived = MicrobitRadio.recv();
  if (myDataReceived != NULL) {
    uint8_t idSender = myDataReceived->payload[0];
    uint32_t timeStampSender;
    memcpy(&timeStampSender, &myDataReceived->payload[1], sizeof(timeStampSender));

    Serial.print("RX: pulse from ");
    Serial.print(idSender);
    Serial.print("  rssi: ");
    Serial.print(myDataReceived->rssi);
    Serial.print("  in t = ");
    Serial.println(timeStampSender);

    delete myDataReceived;
  }
}