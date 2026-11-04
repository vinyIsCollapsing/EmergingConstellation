#include <Arduino.h>
#include "MicrobitV2-HHS.h"
#include "NRF52_Radio_library.h"
#include "Adafruit_Microbit.h"

// ==================================================
//  LOG / DEBUG CONFIG
//  1 = enabled, 0 = disabled
// ==================================================
#define ENABLE_SERIAL_LOG     1   // master switch: 0 disables all serial logging
#define LOG_BOOT              0   // boot messages
#define LOG_TX                1   // “TX: pulse sent...”
#define LOG_RX                1   // periodic RX line (every LOG_EVERY_N_PULSES)
#define LOG_NEIGHBORS         1   // neighbours table (periodic report)
#define LOG_NEIGHBOR_EVENTS   0   // “New neighbour” and “Lost neighbour”
#define LOG_GAPS              0   // “GAP from ...” (lost packets)
#define LOG_WARNINGS          1   // warnings (table full, drift out of range)
#define DEBUG_RAW             0   // raw bytes received (verbose)

#define PIN_BTN PIN_BUTTON_A
const unsigned long DEBOUNCE_MS = 30;

#define SECRET                0xEC
#define MAX_NEIGHBORS         6
#define NEIGHBORS_TIMEOUT_MS  (3UL * 2000UL)  // 3 missed pulses = LOST
#define DRIFT_MIN_SPAN_MS     60000UL          // minimum window for calculating drift

// Packet size: 6 bytes for payload + header library overhead
#define PAYLOAD_LEN     6
#define RADIO_LEN       (PAYLOAD_LEN + MICROBIT_RADIO_HEADER_SIZE - 1) // = 9

NRF52_Radio MicrobitRadio = NRF52_Radio();
Adafruit_Microbit_Matrix matrix;

// Radio config (must be the same in all boards)
const int GROUP = 42;     // 0 - 255
const int BAND = 50;      // 0 - 100
const int POWER = 7;      // 0 - 7

const uint8_t MY_ID = 1;  // Must be diffent for each board

struct Neighbor{
  uint8_t         id;
  bool            active;
  unsigned long   firstSeenMs;
  unsigned long   lastSeenMs;
  uint32_t        firstRemoteTs;
  uint32_t        lastRemoteTs;
  int             lastRssi;
  unsigned long   pulseCount;
  unsigned long   lastIntervalMs;
  float           avgIntervalMs;
  float           driftPpm; 
};

Neighbor neighbors[MAX_NEIGHBORS];

const unsigned long SEND_INTERVAL_MS    = 2000;
const unsigned long REPORT_INTERVAL_MS  = 5000;
const unsigned long LOG_EVERY_N_PULSES  = 5;

unsigned long lastSend      = 0;
unsigned long lastReportMs  = 0;

// Button config
int lastReading = HIGH;
int stableState = HIGH;
unsigned long lastChangeMs = 0;

// --- Animation state ---
enum AnimationType {ANIM_NONE, ANIM_TX, ANIM_RX};

AnimationType currenteAnimation = ANIM_NONE;
uint8_t animationFrame = 0;
unsigned long lastFrameMs = 0;

// --- TX Animation ---
const uint8_t TX_FRAMES[][5] = {
  {0b00000, 0b00000, 0b00100, 0b00000, 0b00000},
  {0b00000, 0b00100, 0b01010, 0b00100, 0b00000},
  {0b00100, 0b01010, 0b10001, 0b01010, 0b00100},
  {0b01010, 0b10001, 0b00000, 0b10001, 0b01010}
};

// --- RX Animation ---
const uint8_t RX_FRAMES[][5] = {
  {0b01010, 0b10001, 0b00000, 0b10001, 0b01010},
  {0b00100, 0b01010, 0b10001, 0b01010, 0b00100},
  {0b00000, 0b00100, 0b01010, 0b00100, 0b00000},
  {0b00000, 0b00000, 0b00100, 0b00000, 0b00000}
};

const uint8_t NUM_FRAMES = 4;
const unsigned long FRAME_INTERVAL_MS = 80; // Frame duration

// ==================================================
// -------------- Function declaration --------------
// ==================================================
// Table management
int findNeighbor(uint8_t _id);
int freeSlot();
void updateNeighbor(uint8_t _id, uint32_t _remoteTimestamp, int _rssi, unsigned long _now);
void expireNeighbors(unsigned long _now);
void printNeighborTable(unsigned long _now);
// Radio
void sendPulse();
void handleReceive(unsigned long _now);
// Animation
void startAnimation(AnimationType _type);
void updateAnimation();

// ------ Table management ------

int findNeighbor(uint8_t _id){
  for(int i = 0; i < MAX_NEIGHBORS; i++){
    if(neighbors[i].active && neighbors[i].id == _id){
      return i;
    }
  }
  return -1;
}

int freeSlot(){
  for(int i = 0; i < MAX_NEIGHBORS; i++){
    if(!neighbors[i].active){
      return i;
    }
  }
  return -1;
}

void updateNeighbor(uint8_t _id, uint32_t _remoteTimestamp, int _rssi, unsigned long _now){
  int index = findNeighbor(_id);
  if(index == -1){
    index = freeSlot();
    if(index == -1){
#if ENABLE_SERIAL_LOG && LOG_WARNINGS
      Serial.println("WARNING: neighbor table full");
#endif
      return;
    }

    neighbors[index] = Neighbor{};
    neighbors[index].id = _id;
    neighbors[index].active = true;
    neighbors[index].firstSeenMs = _now;
    neighbors[index].firstRemoteTs = _remoteTimestamp;

#if ENABLE_SERIAL_LOG && LOG_NEIGHBOR_EVENTS
    Serial.print("New neighbor ");
    Serial.println(_id);
#endif
  }

  Neighbor &n = neighbors[index];

  // Interval between packets (measured in local time)
  if(n.pulseCount > 0){
    unsigned long interval = _now - n.lastSeenMs;

    if(interval > (SEND_INTERVAL_MS * 3) / 2){
#if ENABLE_SERIAL_LOG && LOG_GAPS
      unsigned long missed = (interval / SEND_INTERVAL_MS) - 1;
      Serial.print("GAP from ");
      Serial.print(n.id);
      Serial.print(": ");
      Serial.print(interval);
      Serial.print(" ms (~");
      Serial.print(missed);
      Serial.println( "missed)");
#endif
    } else {
      n.lastIntervalMs = interval;
      n.avgIntervalMs = (n.avgIntervalMs == 0)
                        ? (float)interval
                        : 0.8f * n.avgIntervalMs + 0.2f * (float)interval;
    }
  }

  n.lastSeenMs   = _now;
  n.lastRemoteTs = _remoteTimestamp;
  n.lastRssi     = _rssi;
  n.pulseCount++;

  // Drift: compares the total advance of the remote clock with the local clock,
  // using the entire observation period. The radio delay cancels itself out.
  unsigned long localSpan  = n.lastSeenMs - n.firstSeenMs;
  uint32_t      remoteSpan = n.lastRemoteTs - n.firstRemoteTs;

  if (localSpan >= DRIFT_MIN_SPAN_MS) {
    n.driftPpm = ((float)remoteSpan - (float)localSpan) / (float)localSpan * 1e6f;

#if ENABLE_SERIAL_LOG && LOG_WARNINGS
    if (n.driftPpm > 5000.0f || n.driftPpm < -5000.0f) {
      Serial.print("WARNING: drift out of bounds for ");
      Serial.print(n.id);
      Serial.print(" (");
      Serial.print(n.driftPpm, 1);
      Serial.println(" ppm). Verify timestamp.");
    }
#endif
  }

#if ENABLE_SERIAL_LOG && LOG_RX
  if(n.pulseCount % LOG_EVERY_N_PULSES == 0){
    Serial.print("RX | id: ");
    Serial.print(n.id);
    Serial.print(" rssi: ");
    Serial.print(n.lastRssi);
    Serial.print(" interval: ");
    Serial.print(n.lastIntervalMs);
    Serial.print(" last remote timestamp: ");
    Serial.println(n.lastRemoteTs);
  }
#endif
}

void expireNeighbors(unsigned long _now){
  for(int i = 0; i < MAX_NEIGHBORS; i++){
    if(neighbors[i].active && (_now - neighbors[i].lastSeenMs) > NEIGHBORS_TIMEOUT_MS){
#if ENABLE_SERIAL_LOG && LOG_NEIGHBOR_EVENTS
      Serial.print("Lost neighbor ");
      Serial.print(neighbors[i].id);
      Serial.print(" (idle ");
      Serial.print(_now - neighbors[i].lastSeenMs);
      Serial.println(" ms)");
#endif
      neighbors[i].active = false;
    }
  }
}

#if ENABLE_SERIAL_LOG && LOG_NEIGHBORS
void printNeighborTable(unsigned long _now){
  Serial.println("------ neighbors ------");
  int activeCount = 0;
  for(int i = 0; i < MAX_NEIGHBORS; i++){
    if(!neighbors[i].active) continue;
    activeCount++;
    Neighbor &n = neighbors[i];
    Serial.print(" id = ");        Serial.print(n.id);
    Serial.print(" rssi = ");      Serial.print(n.lastRssi);
    Serial.print(" pkts = ");      Serial.print(n.pulseCount);
    Serial.print(" lastInt = ");   Serial.print(n.lastIntervalMs);
    Serial.print(" avgInt = ");    Serial.print(n.avgIntervalMs, 1);
    Serial.print(" span = ");      Serial.print(n.lastSeenMs - n.firstSeenMs);
    Serial.print(" drift_ppm = "); Serial.print(n.driftPpm, 1);
    Serial.print(" age = ");       Serial.println(_now - n.lastSeenMs);
  }
  Serial.print("active neighbors: ");
  Serial.print(activeCount);
  Serial.println(" ------");
}
#endif

// ------ Radio ------
void sendPulse() {
  startAnimation(ANIM_TX);
  FrameBuffer* myDataSent = new FrameBuffer();
  
  uint32_t timeStamp = millis();

  //  Byte-by-byte encoded timestamp (little-endian)
  myDataSent->payload[0] = SECRET;
  myDataSent->payload[1] = MY_ID;
  myDataSent->payload[2] = (timeStamp      ) & 0xFF;
  myDataSent->payload[3] = (timeStamp >>  8) & 0xFF;
  myDataSent->payload[4] = (timeStamp >> 16) & 0xFF;
  myDataSent->payload[5] = (timeStamp >> 24) & 0xFF;

  myDataSent->length = RADIO_LEN;

  MicrobitRadio.send(myDataSent);
  delete myDataSent;

#if ENABLE_SERIAL_LOG && LOG_TX
  Serial.print("TX: pulse sent by: ");
  Serial.print(MY_ID);
  Serial.print("  at t = ");
  Serial.println(timeStamp);
#endif
}

void handleReceive(unsigned long _now) {
  FrameBuffer* myDataReceived = MicrobitRadio.recv();
  if (myDataReceived == NULL) return;

#if ENABLE_SERIAL_LOG && DEBUG_RAW
  Serial.print("RAW len=");
  Serial.print(myDataReceived->length);
  for (int i = 0; i < PAYLOAD_LEN; i++) {
    Serial.print(" ");
    if (myDataReceived->payload[i] < 0x10) Serial.print("0");
    Serial.print(myDataReceived->payload[i], HEX);
  }
  Serial.println();
#endif

  if (myDataReceived->length >= RADIO_LEN && myDataReceived->payload[0] == SECRET) {
    uint8_t  id = myDataReceived->payload[1];
    uint8_t *p  = myDataReceived->payload;

    uint32_t remoteTs = (uint32_t)p[2]
                      | ((uint32_t)p[3] << 8)
                      | ((uint32_t)p[4] << 16)
                      | ((uint32_t)p[5] << 24);

    if (id != MY_ID) {
      updateNeighbor(id, remoteTs, myDataReceived->rssi, _now);
        startAnimation(ANIM_RX);
    }
  }

  delete myDataReceived;
}

// ---------- Animation ----------
void startAnimation(AnimationType _type){
  currenteAnimation = _type;
  animationFrame = 0;
  lastFrameMs = millis();
}

void updateAnimation(){
  if(currenteAnimation == ANIM_NONE) return;

  unsigned long now = millis();
  if(now - lastFrameMs >= FRAME_INTERVAL_MS){
    lastFrameMs = now;
    animationFrame++;

    if(animationFrame >= NUM_FRAMES){
      matrix.clear();
      currenteAnimation = ANIM_NONE;
      return;
    }

    const uint8_t (*frames)[5] = (currenteAnimation == ANIM_TX) ? TX_FRAMES : RX_FRAMES;
    matrix.show(frames[animationFrame]);
  }
}

// Setup/Loop
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BTN, INPUT_PULLUP);
  lastReading = stableState = digitalRead(PIN_BTN);

  // Radio init
  MicrobitRadio.enable();
  MicrobitRadio.setGroup(GROUP);
  MicrobitRadio.setFrequencyBand(BAND);
  MicrobitRadio.setTransmitPower(POWER);

  // Matrix init
  matrix.begin();
  matrix.clear();
  
  for(int i = 0; i < MAX_NEIGHBORS; i++) neighbors[i].active = false;


#if ENABLE_SERIAL_LOG && LOG_BOOT
  Serial.println("Radio ready");
  Serial.println(MY_ID);
  Serial.println("Ready. Waiting for packets...");
#endif
}

void loop() {
  unsigned long now = millis();

  // Periodic trigger
  if(now - lastSend >= SEND_INTERVAL_MS){
    lastSend = now;
    sendPulse();
  }

  // Manual trigger
  int reading = digitalRead(PIN_BTN);
  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = now;
  }

  if (now - lastChangeMs >= DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      sendPulse();
    }
  }

  // Reception with a filter
  handleReceive(now);
  expireNeighbors(now);

  updateAnimation();

  if(now - lastReportMs >= REPORT_INTERVAL_MS){
    lastReportMs = now;
    printNeighborTable(now);
  }
}