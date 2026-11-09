#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "MicrobitV2-HHS.h"
#include "NRF52_Radio_library.h"
#include "Adafruit_Microbit.h"

// ==================================================
//  LOG / DEBUG CONFIG
//  1 = enabled, 0 = disabled
// ==================================================
#define ENABLE_SERIAL_LOG     1   // master switch: 0 disables all serial logging
#define LOG_BOOT              1   // boot messages
#define LOG_TX                1   // TX: pulse sent and FIRE line
#define LOG_RX                1   // periodic RX line (every LOG_EVERY_N_PULSES)
#define LOG_NEIGHBORS         1   // neighbours table (periodic report)
#define LOG_NEIGHBOR_EVENTS   0   // “New neighbour” and “Lost neighbour”
#define LOG_GAPS              0   // “GAP from ...” (lost packets)
#define LOG_WARNINGS          1   // warnings (table full, drift out of range)
#define DEBUG_RAW             0   // raw bytes received (verbose)
#define LOG_PHASE             1   // KICK and PHASE lines per received pulse
#define LOG_WAVE              1   // SYNC and WAVE lines

#define PIN_BTN PIN_BUTTON_A
const unsigned long DEBOUNCE_MS = 30;

// ==================================================
//  PACKET TYPES AND LAYOUT
//  All packets use the same length (RADIO_LEN).
//  Pulse: [0]=SECRET_PULSE [1..4]=sender id [5..8]=timestamp
//  Wave : [0]=SECRET_WAVE  [1..4]=sender id [5..8]=target id
//         [9]=hop [10]=wave id [11..14]=origin id
// ==================================================
#define SECRET_PULSE          0xEC
#define SECRET_WAVE           0xED
#define SECRET_WAVE_ACK       0xEE

// ==================================================
//  SYNCHRONIZATION AND WAVE CONFIG
// ==================================================
#define SYNC_PULSES           5                // consecutive pulses within tolerance
const float SYNC_TOL = 0.05f;                  // |diff| tolerance, in periods

#define WAVE_FRAME_MS         120               // duration of each column
#define WAVE_ARM_MIN_MS       100               // random delay before starting a wave
#define WAVE_ARM_MAX_MS       400
#define MAX_WAVE_HOPS         12                // safety bound on the chain length
#define WAVE_ACK_TIMEOUT_MS   200
#define WAVE_MAX_RETRIES      3

#define MAX_NEIGHBORS         6
#define NEIGHBORS_TIMEOUT_MS  (3UL * 2000UL)  // 3 missed pulses = LOST
#define DRIFT_MIN_SPAN_MS     60000UL         // minimum window for calculating drift

#define PAYLOAD_LEN     15                     // 1 SECRET + 4 sender + 4 target/ts + 2 hop/id + 4 origin
#define RADIO_LEN       (PAYLOAD_LEN + MICROBIT_RADIO_HEADER_SIZE - 1) // = 18

NRF52_Radio MicrobitRadio = NRF52_Radio();
Adafruit_Microbit_Matrix matrix;

// Radio config (must be the same in all boards)
const int GROUP = 42;     // 0 - 255
const int BAND = 50;      // 0 - 100
const int POWER = 7;      // 0 - 7

uint32_t MY_ID;  // Must be diffent for each board

struct Neighbor{
  uint32_t        id;
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
  uint8_t         syncCount;
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

AnimationType currentAnimation = ANIM_NONE;
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

// --- Wave animation state ---
// Column sweep from left (column 0) to right (column 4).
// Bit 4 is the leftmost pixel of each row. If the sweep looks mirrored
// on your board, change bit 4 to bit 0 in each row.
const uint8_t WAVE_FRAMES[5][5] = {
  {0b10000, 0b10000, 0b10000, 0b10000, 0b10000},
  {0b01000, 0b01000, 0b01000, 0b01000, 0b01000},
  {0b00100, 0b00100, 0b00100, 0b00100, 0b00100},
  {0b00010, 0b00010, 0b00010, 0b00010, 0b00010},
  {0b00001, 0b00001, 0b00001, 0b00001, 0b00001}
};

bool          waveActive          = false;
uint8_t       waveFrame           = 0;
unsigned long lastWaveFrameMs     = 0;
uint8_t       waveHop             = 0;     // position of this board in the chain
uint8_t       waveId              = 0;     // id of the wave currently playing
uint32_t      waveOrigin          = 0;     // board that started the wave
uint8_t       waveCounter         = 0;     // waves started by this board
bool          wavePending         = false;
uint32_t      wavePendingTarget   = 0;
uint8_t       wavePendingHop      = 0;
uint8_t       wavePendingId       = 0;
uint32_t      wavePendingOrigin   = 0;
unsigned long wavePendingSentMs   = 0;
uint8_t       wavePendingTries    = 0;

// Dedupe: a board plays a given (origin, id) wave only once
uint32_t lastWaveOrigin = 0;
uint8_t  lastWaveSeen   = 0;
bool     hasLastWave    = false;

// --- Synchronization detection ---
bool          syncArmed   = true;    // ready to trigger on the next sync episode
bool          waveArmed   = false;   // a wave start is scheduled
unsigned long waveArmAtMs = 0;

// Coupled oscillator (Mirollo-Strogatz)
const float   PERIOD_MS     = 2000.0f;  // Natural period (ms)
const float   EPSILON       = 0.02f;    // phase advance per received pulse
const float   RESP_B        = 3.0f;     // curvature of the concave response (b > 0)

// Concave response function: maps phase in [0,1] to state in [0,1]
// and back. The pulse adds EPSILON to the state, which is what makes
// the coupling absorbing and drives the system toward synchrony.
float phaseToState(float x) {
  return logf(1.0f + (expf(RESP_B) - 1.0f) * x) / RESP_B;
}

float stateToPhase(float s) {
  return (expf(RESP_B * s) - 1.0f) / (expf(RESP_B) - 1.0f);
}

// Oscillator state
float         phase                = 0.0f;
unsigned long lastPhaseUpdateMs    = 0;
unsigned long lastFireMs           = 0;
bool          everFired            = false;

// ==================================================
// -------------- Function declaration --------------
// ==================================================
// Byte helpers
uint32_t readU32(const uint8_t *p);
void     writeU32(uint8_t *p, uint32_t v);
// Table management
int findNeighbor(uint32_t _id);
int freeSlot();
int  pickNextNeighbor();
bool isSynced();
void updateNeighbor(uint32_t _id, uint32_t _remoteTimestamp, int _rssi, unsigned long _now);
void expireNeighbors(unsigned long _now);
void printNeighborTable(unsigned long _now);
// Radio
void sendPulse();
void sendWave(uint32_t target, uint8_t hop, uint8_t id, uint32_t origin);
void handlePulse(uint32_t id, uint32_t remoteTs, int rssi, unsigned long now);
void handleWave(uint32_t target, uint8_t hop, uint8_t id, uint32_t origin);
void handleReceive(unsigned long _now);
void sendWaveAck(uint32_t target, uint8_t id, uint32_t origin);
void handleWaveAck(uint32_t sender, uint32_t target, uint8_t id, uint32_t origin);
// Animation
void startAnimation(AnimationType _type);
void updateAnimation();
void startWave(uint8_t hop, uint8_t id, uint32_t origin);
void updateWave(unsigned long now);
void forwardWave();
void startWaveLocal();
void updateSyncWave(unsigned long now);
void retryPendingWave(unsigned long now);

// ==================================================
// ------------- Byte helpers -------------
// ==================================================
uint32_t readU32(const uint8_t *p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

void writeU32(uint8_t *p, uint32_t v) {
  p[0] = (v      ) & 0xFF;
  p[1] = (v >>  8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

// ------ Table management ------

int findNeighbor(uint32_t _id){
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

// Next board in the ring: smallest active id greater than ours,
// or the smallest active id if we are the highest.
int pickNextNeighbor() {
  int best = -1;
  int smallest = -1;
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (!neighbors[i].active) continue;
    if (smallest < 0 || neighbors[i].id < neighbors[smallest].id) smallest = i;
    if (neighbors[i].id > MY_ID &&
        (best < 0 || neighbors[i].id < neighbors[best].id)) {
      best = i;
    }
  }
  return (best >= 0) ? best : smallest;
}

// Synchronized: at least one neighbor, and every active neighbor
// has been within tolerance for SYNC_PULSES consecutive pulses.
bool isSynced() {
  int active = 0;
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (!neighbors[i].active) continue;
    active++;
    if (neighbors[i].syncCount < SYNC_PULSES) return false;
  }
  return active > 0;
}

void updateNeighbor(uint32_t _id, uint32_t _remoteTimestamp, int _rssi, unsigned long _now){
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

  memset(myDataSent->payload, 0, PAYLOAD_LEN);
  myDataSent->payload[0] = SECRET_PULSE;
  writeU32(&myDataSent->payload[1], MY_ID);
  writeU32(&myDataSent->payload[5], timeStamp);

  myDataSent->length = RADIO_LEN;

  MicrobitRadio.send(myDataSent);
  delete myDataSent;

#if ENABLE_SERIAL_LOG && LOG_TX
  Serial.print("TX: pulse sent by: ");
  Serial.print(MY_ID);
  Serial.print("  at t = ");
  Serial.println(timeStamp);

  Serial.print("FIRE | t=");
  Serial.print(timeStamp);
  Serial.print(" phase_after=");
  Serial.println(phase, 3);
#endif
}

void sendWave(uint32_t target, uint8_t hop, uint8_t id, uint32_t origin) {
  FrameBuffer* myDataSent = new FrameBuffer();

  memset(myDataSent->payload, 0, PAYLOAD_LEN);
  myDataSent->payload[0] = SECRET_WAVE;
  writeU32(&myDataSent->payload[1],  MY_ID);
  writeU32(&myDataSent->payload[5],  target);
  myDataSent->payload[9]  = hop;
  myDataSent->payload[10] = id;
  writeU32(&myDataSent->payload[11], origin);

  myDataSent->length = RADIO_LEN;

  MicrobitRadio.send(myDataSent);
  delete myDataSent;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | send to=");
  Serial.print(target);
  Serial.print(" hop=");
  Serial.print(hop);
  Serial.print(" id=");
  Serial.print(id);
  Serial.print(" origin=");
  Serial.println(origin);
#endif
}

void handlePulse(uint32_t id, uint32_t remoteTs, int rssi, unsigned long now) {
  updateNeighbor(id, remoteTs, rssi, now);
  startAnimation(ANIM_RX);

  // Phase difference relative to our last fire, in [-0.5, +0.5]
  float phaseDiff = 0.0f;
  if (everFired) {
    unsigned long sinceFire = (now - lastFireMs) % (unsigned long)PERIOD_MS;
    phaseDiff = (float)sinceFire / PERIOD_MS;
    if (phaseDiff > 0.5f) phaseDiff -= 1.0f;
  }

  // Sync counter per neighbor
  int idx = findNeighbor(id);
  if (idx >= 0 && everFired) {
    if (fabsf(phaseDiff) < SYNC_TOL) {
      if (neighbors[idx].syncCount < 255) neighbors[idx].syncCount++;
    } else {
      neighbors[idx].syncCount = 0;
    }
  }

  // Mirollo-Strogatz coupling with concave response and absorption
  float phaseBefore = phase;
  float s = phaseToState(phase) + EPSILON;
  if (s >= 1.0f) {
    phase = 1.0f;                  // absorption: fires on the next loop()
  } else {
    phase = stateToPhase(s);
  }

#if ENABLE_SERIAL_LOG && LOG_PHASE
  Serial.print("KICK | id=");
  Serial.print((unsigned long)id);
  Serial.print(" before=");
  Serial.print(phaseBefore, 3);
  Serial.print(" after=");
  Serial.println(phase, 3);

  if (everFired) {
    Serial.print("PHASE | id=");
    Serial.print((unsigned long)id);
    Serial.print(" diff=");
    Serial.print(phaseDiff, 3);
    Serial.print(" phase=");
    Serial.println(phase, 3);
  }
#endif
}

void handleWave(uint32_t target, uint8_t hop, uint8_t id, uint32_t origin) {
  // A wave is already travelling: do not start a competing one
  waveArmed = false;

  if (waveActive) return;
  if (target != MY_ID) return;

  // Ignore duplicates of a wave already played
  if (hasLastWave && origin == lastWaveOrigin && id == lastWaveSeen) return;

  lastWaveOrigin = origin;
  lastWaveSeen   = id;
  hasLastWave    = true;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | recv hop=");
  Serial.print(hop);
  Serial.print(" id=");
  Serial.print(id);
  Serial.print(" origin=");
  Serial.println(origin);
#endif

  startWave(hop, id, origin);
}

void handleReceive(unsigned long _now) {
  FrameBuffer* myDataReceived = MicrobitRadio.recv();
  if (myDataReceived == NULL) return;

#if ENABLE_SERIAL_LOG && DEBUG_RAW
  Serial.print("RAW len=");
  Serial.print(myDataReceived->length);
  for (int i = 0; i < 9; i++) {
    Serial.print(" ");
    if (myDataReceived->payload[i] < 0x10) Serial.print("0");
    Serial.print(myDataReceived->payload[i], HEX);
  }
  Serial.println();
#endif

  if (myDataReceived->length >= RADIO_LEN) {
    uint8_t *p = myDataReceived->payload;
    uint32_t senderId = readU32(&p[1]);

    if (senderId != MY_ID) {
      if (p[0] == SECRET_PULSE) {
        handlePulse(senderId, readU32(&p[5]), myDataReceived->rssi, _now);
      } else if (p[0] == SECRET_WAVE) {
        uint32_t target = readU32(&p[5]);
        if (target == MY_ID) {
          sendWaveAck(senderId, p[10], readU32(&p[11]));
        }
        handleWave(target, p[9], p[10], readU32(&p[11]));
      } else if (p[0] == SECRET_WAVE_ACK) {
        handleWaveAck(senderId, readU32(&p[5]), p[10], readU32(&p[11]));
      }
    }
  }

  delete myDataReceived;
}

void sendWaveAck(uint32_t target, uint8_t id, uint32_t origin) {
  FrameBuffer* ack = new FrameBuffer();

  memset(ack->payload, 0, PAYLOAD_LEN);
  ack->payload[0] = SECRET_WAVE_ACK;
  writeU32(&ack->payload[1],  MY_ID);
  writeU32(&ack->payload[5],  target);
  ack->payload[10] = id;
  writeU32(&ack->payload[11], origin);

  ack->length = RADIO_LEN;

  MicrobitRadio.send(ack);
  delete ack;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | ack to=");
  Serial.println(target);
#endif
}

void handleWaveAck(uint32_t sender, uint32_t target, uint8_t id, uint32_t origin) {
  if (!wavePending) return;
  if (target != MY_ID) return;
  if (sender != wavePendingTarget) return;
  if (id != wavePendingId || origin != wavePendingOrigin) return;

  wavePending = false;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | ack received from ");
  Serial.println(sender);
#endif
}

// ---------- Animation ----------
void startAnimation(AnimationType _type) {
  if (waveActive) return;          // the wave has priority on the matrix
  currentAnimation = _type;
  animationFrame = 0;
  lastFrameMs = millis();
}

void updateAnimation() {
  if (currentAnimation == ANIM_NONE) return;

  unsigned long now = millis();
  if (now - lastFrameMs >= FRAME_INTERVAL_MS) {
    lastFrameMs = now;
    animationFrame++;

    if (animationFrame >= NUM_FRAMES) {
      matrix.clear();
      currentAnimation = ANIM_NONE;
      return;
    }

    const uint8_t (*frames)[5] = (currentAnimation == ANIM_TX) ? TX_FRAMES : RX_FRAMES;
    matrix.show(frames[animationFrame]);
  }
}

// ==================================================
// ------------- Wave animation -------------
// ==================================================

void startWave(uint8_t hop, uint8_t id, uint32_t origin) {
  wavePending     = false;
  waveActive      = true;
  waveFrame       = 0;
  waveHop         = hop;
  waveId          = id;
  waveOrigin      = origin;
  lastWaveFrameMs = millis();
  currentAnimation = ANIM_NONE;    // stop any TX/RX animation in progress
  matrix.show(WAVE_FRAMES[0]);
}

void updateWave(unsigned long now) {
  if (!waveActive) return;
  if (now - lastWaveFrameMs < WAVE_FRAME_MS) return;

  lastWaveFrameMs = now;
  waveFrame++;

  if (waveFrame >= 5) {
    matrix.clear();
    waveActive = false;
    forwardWave();                 // this board's part is done: pass the wave on
    return;
  }

  matrix.show(WAVE_FRAMES[waveFrame]);
}

// Passes the wave to the next board, unless the lap is complete
void forwardWave() {
  uint8_t nextHop = waveHop + 1;
  if (nextHop > MAX_WAVE_HOPS) {
#if ENABLE_SERIAL_LOG && LOG_WAVE
    Serial.println("WAVE | hop limit reached, stopping");
#endif
    return;
  }

  int idx = pickNextNeighbor();
  if (idx < 0) {
#if ENABLE_SERIAL_LOG && LOG_WAVE
    Serial.println("WAVE | no neighbor to forward to");
#endif
    return;
  }

  uint32_t target = neighbors[idx].id;
  if (target == waveOrigin) {
#if ENABLE_SERIAL_LOG && LOG_WAVE
    Serial.println("WAVE | lap complete");
#endif
    return;
  }

  sendWave(target, nextHop, waveId, waveOrigin);

  // Keep the packet until the next board confirms it
  wavePending       = true;
  wavePendingTarget = target;
  wavePendingHop    = nextHop;
  wavePendingId     = waveId;
  wavePendingOrigin = waveOrigin;
  wavePendingSentMs = millis();
  wavePendingTries  = 0;
}

// This board starts a wave (hop 0). Called when the sync arming expires.
void startWaveLocal() {
  waveCounter++;
  lastWaveOrigin = MY_ID;
  lastWaveSeen   = waveCounter;
  hasLastWave    = true;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | start local, id=");
  Serial.println(waveCounter);
#endif

  startWave(0, waveCounter, MY_ID);
}

// Arms a wave start once per sync episode, with a random delay.
// If a wave from another board arrives first, handleWave() cancels it.
void updateSyncWave(unsigned long now) {
  if (!isSynced()) {
    syncArmed = true;              // re-arm for the next sync episode
    waveArmed = false;             // cancel any pending start
    return;
  }

  if (syncArmed && !waveActive && !waveArmed) {
    syncArmed = false;
    waveArmed = true;
    waveArmAtMs = now + (unsigned long)random(WAVE_ARM_MIN_MS, WAVE_ARM_MAX_MS + 1);
#if ENABLE_SERIAL_LOG && LOG_WAVE
    Serial.println("SYNC | synchronized, arming wave");
#endif
  }

  if (waveArmed && now >= waveArmAtMs) {
    waveArmed = false;
    if (!waveActive) startWaveLocal();
  }
}

void retryPendingWave(unsigned long now) {
  if (!wavePending) return;
  if (now - wavePendingSentMs < WAVE_ACK_TIMEOUT_MS) return;

  if (wavePendingTries >= WAVE_MAX_RETRIES) {
    wavePending = false;
#if ENABLE_SERIAL_LOG && LOG_WAVE
    Serial.print("WAVE | no ACK from ");
    Serial.print(wavePendingTarget);
    Serial.println(", giving up");
#endif
    return;
  }

  wavePendingTries++;
  wavePendingSentMs = now;

#if ENABLE_SERIAL_LOG && LOG_WAVE
  Serial.print("WAVE | retry ");
  Serial.print(wavePendingTries);
  Serial.print(" to ");
  Serial.println(wavePendingTarget);
#endif

  sendWave(wavePendingTarget, wavePendingHop, wavePendingId, wavePendingOrigin);
}

// Setup/Loop
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Automatic ID signing
  uint32_t deviceId = NRF_FICR->DEVICEID[1];
  MY_ID = deviceId;

#if ENABLE_SERIAL_LOG && LOG_BOOT
  Serial.print("MY_ID (32-bit): ");
  Serial.println(MY_ID);
#endif

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

  // Randomize initial phase using the factory ID as seed
  // (avoids the four boards starting synchronized by accident)
  randomSeed(NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1]);
  phase = (float)random(0, 10000) / 10000.0f;

  lastPhaseUpdateMs = millis();
  lastFireMs        = 0;
  everFired         = false;

#if ENABLE_SERIAL_LOG && LOG_BOOT
  Serial.print("Initial phase: ");
  Serial.println(phase, 3);
#endif
}

void loop() {
  unsigned long now = millis();

  // Advance oscillator phase
  unsigned long dt = now - lastPhaseUpdateMs;
  lastPhaseUpdateMs = now;
  phase += (float)dt / PERIOD_MS;

  // Fire when phase reaches 1
  if(phase >= 1.0f){
    phase -= 1.0f;      // preserve excess
    lastFireMs = now;
    everFired = true;
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
      // Reset phase and send a pulse
      phase   = 0.0f;
      lastFireMs = now;
      everFired = true;
      sendPulse();
    }
  }

  // Reception with a filter
  handleReceive(now);
  expireNeighbors(now);

  // Synchronization and wave
  updateSyncWave(now);
  updateWave(now);
  retryPendingWave(now);

  updateAnimation();

#if ENABLE_SERIAL_LOG && LOG_NEIGHBORS
  if(now - lastReportMs >= REPORT_INTERVAL_MS){
    lastReportMs = now;
    printNeighborTable(now);
  }
#endif
}