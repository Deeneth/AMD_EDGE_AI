// ████████████████████████████████████████████████████████████████████████████
// PROJECT  : Dynamic Green Light Traffic System
// FILE     : esp32master_main.ino
// DEVICE   : ESP32 Master
// ROLE     : Servo + Traffic Lights + Timing Logic + Emergency Handling
// VERSION  : 3.0 — RASPBERRY PI + YOLOV8 + SINKING MODE LEDS
// ████████████████████████████████████████████████████████████████████████████
//
// ARCHITECTURE (v3.0):
//   Raspberry Pi (YOLOv8 + USB Webcam) replaces ESP32-CAM
//   Pi runs traffic_pi.py → sends CNT packets via UDP
//   Master sends CAPTURE/PATROL commands to Pi (same as before)
//   Master IP is learned automatically from first Pi packet
//   NO CHANGES needed to packet format — fully compatible!
//
// SINKING MODE WIRING:
//   LED Anode (+) ──── 330Ω resistor ──── 3.3V or 5V
//   LED Cathode (-) ── GPIO pin
//   GPIO LOW  = LED ON
//   GPIO HIGH = LED OFF
//
// ████████████████████████████████████████████████████████████████████████████


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 1: INCLUDES & DEFINES
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ── Debug ────────────────────────────────────────────────────────────────────
#define DEBUG true
#define DBG(x)        if(DEBUG) Serial.println(x)
#define DBGF(x, ...)  if(DEBUG) Serial.printf(x, __VA_ARGS__)

// ── Sinking mode helpers ──────────────────────────────────────────────────────
// In sinking mode: LOW = ON, HIGH = OFF
#define LED_ON  LOW
#define LED_OFF HIGH


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 2: CONFIGURATION — CHANGE THESE
// ═══════════════════════════════════════════════════════════════════════════

// ── WiFi ─────────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "srsd";
const char* WIFI_PASSWORD = "Sivajivayilajelabi@123";

// ── Ports ────────────────────────────────────────────────────────────────────
const int MASTER_PORT = 4210;
const int CAM_PORT    = 4211;

// ── Servo Pin & Angles ────────────────────────────────────────────────────────
#define SERVO_PIN         18
#define SERVO_WEST       180
#define SERVO_NORTH       90
#define SERVO_EAST         0
#define SERVO_SETTLE_MS  600

// ── Traffic Light GPIO Pins ───────────────────────────────────────────────────
// West Lane
#define WEST_RED     15
#define WEST_YELLOW  13
#define WEST_GREEN   12
// North Lane
#define NORTH_RED    14
#define NORTH_YELLOW 27
#define NORTH_GREEN  26
// East Lane
#define EAST_RED      5
#define EAST_YELLOW  33
#define EAST_GREEN   32

// ── Buzzer ───────────────────────────────────────────────────────────────────
// Buzzer stays in SOURCING mode (HIGH = ON) — sinking only for LEDs
#define BUZZER_PIN    19

// ── Timing ───────────────────────────────────────────────────────────────────
#define MIN_GREEN_TIME        5
#define MAX_GREEN_TIME       60
#define BASE_GREEN_TIME      10
#define TIME_PER_UNIT         2
#define FALLBACK_GREEN_TIME  20
#define YELLOW_DURATION    2000
#define CAM_RESPONSE_TIMEOUT 8000
#define FOG_QUALITY_THRESHOLD 40

// ── Pedestrian ───────────────────────────────────────────────────────────────
#define PED_DELAY_MS    2000
#define PED_MAX_DELAYS     2

// ── Ambulance ────────────────────────────────────────────────────────────────
#define AMBULANCE_MIN_GREEN  15000
#define AMBULANCE_CLEARANCE   3000


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 3: GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

WiFiUDP udp;
char    udpBuffer[256];
Servo   camServo;
int     servoAngle = SERVO_WEST;

enum Lane { WEST = 0, NORTH = 1, EAST = 2, NONE_LANE = -1 };
const char* LANE_NAMES[] = {"WEST", "NORTH", "EAST"};
Lane  cycleOrder[] = {WEST, NORTH, EAST};
int   cycleIndex   = 0;

// Per-lane stored counts
// Classes: car, threewheel, bus, truck, motorbike, van, ambulance, firetruck, pedestrian
struct LaneCount {
  int  cars = 0, threewheels = 0, buses = 0, trucks = 0, motorbikes = 0, vans = 0;
  bool ambulance = false, fireTruck = false, pedestrian = false;
  int  quality   = 100;
  bool fresh     = false;
};
LaneCount counts[3];

bool fallbackMode = false;
bool fogMode      = false;

IPAddress camIP;
bool      camIPKnown = false;
unsigned long lastCamPacket = 0;


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 4: WIFI SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setupWiFi() {
  DBG("[WiFi] Connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    DBGF("\n[WiFi] Master IP: %s\n", WiFi.localIP().toString().c_str());
    udp.begin(MASTER_PORT);
  } else {
    DBG("[WiFi] FAILED — fallback mode");
    fallbackMode = true;
  }
}

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(3000);
  }
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 5: SERVO CONTROL
// ═══════════════════════════════════════════════════════════════════════════

void setupServo() {
  camServo.attach(SERVO_PIN);
  camServo.write(SERVO_WEST);
  servoAngle = SERVO_WEST;
  delay(500);
  DBG("[SERVO] Initialised at WEST");
}

void faceLane(Lane lane) {
  int target;
  switch (lane) {
    case WEST:  target = SERVO_WEST;  break;
    case NORTH: target = SERVO_NORTH; break;
    case EAST:  target = SERVO_EAST;  break;
    default:    return;
  }
  if (servoAngle != target) {
    DBGF("[SERVO] Rotating to %s (%d°)\n", LANE_NAMES[lane], target);
    camServo.write(target);
    servoAngle = target;
    delay(SERVO_SETTLE_MS);
  }
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 6: TRAFFIC LIGHT CONTROL — SINKING MODE
// ═══════════════════════════════════════════════════════════════════════════
//
// WIRING FOR EACH LED:
//   (+) Anode  → 330Ω resistor → 3.3V
//   (-) Cathode → GPIO pin
//
// GPIO LOW  = LED ON  ✅
// GPIO HIGH = LED OFF ✅

const int PIN_RED[]    = {WEST_RED,    NORTH_RED,    EAST_RED};
const int PIN_YELLOW[] = {WEST_YELLOW, NORTH_YELLOW, EAST_YELLOW};
const int PIN_GREEN[]  = {WEST_GREEN,  NORTH_GREEN,  EAST_GREEN};

void setupLights() {
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_RED[i],    OUTPUT);
    pinMode(PIN_YELLOW[i], OUTPUT);
    pinMode(PIN_GREEN[i],  OUTPUT);
    // ── SINKING: initialise all LEDs OFF = HIGH ──────────────────────────
    digitalWrite(PIN_RED[i],    LED_OFF);
    digitalWrite(PIN_YELLOW[i], LED_OFF);
    digitalWrite(PIN_GREEN[i],  LED_OFF);
  }
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Buzzer OFF
}

// ── Set individual light state ────────────────────────────────────────────────
// r=true means RED ON, y=true means YELLOW ON, g=true means GREEN ON
void setLight(Lane lane, bool r, bool y, bool g) {
  if (lane == NONE_LANE) return;
  // SINKING: true=ON=LOW, false=OFF=HIGH
  digitalWrite(PIN_RED[lane],    r ? LED_ON : LED_OFF);
  digitalWrite(PIN_YELLOW[lane], y ? LED_ON : LED_OFF);
  digitalWrite(PIN_GREEN[lane],  g ? LED_ON : LED_OFF);
}

// ── All lanes RED ─────────────────────────────────────────────────────────────
void allRed() {
  for (int i = 0; i < 3; i++) setLight((Lane)i, true, false, false);
  DBG("[LIGHTS] All RED");
}

// ── One lane GREEN, others RED ────────────────────────────────────────────────
void setGreen(Lane lane) {
  for (int i = 0; i < 3; i++) {
    if (i == (int)lane) {
      setLight((Lane)i, false, false, true);  // GREEN ON
    } else {
      setLight((Lane)i, true, false, false);  // RED ON
    }
  }
  DBGF("[LIGHTS] %s = GREEN\n", LANE_NAMES[lane]);
}

// ── Transition GREEN → YELLOW → RED ──────────────────────────────────────────
void transitionToRed(Lane lane) {
  setLight(lane, false, true, false);   // YELLOW ON
  DBGF("[LIGHTS] %s = YELLOW\n", LANE_NAMES[lane]);
  delay(YELLOW_DURATION);
  setLight(lane, true, false, false);   // RED ON
  DBGF("[LIGHTS] %s = RED\n", LANE_NAMES[lane]);
}

// ── Blink RED on all lanes (fallback alert) ───────────────────────────────────
void blinkRedAll() {
  // All RED ON
  for (int i = 0; i < 3; i++) digitalWrite(PIN_RED[i], LED_ON);
  delay(200);
  // All RED OFF
  for (int i = 0; i < 3; i++) digitalWrite(PIN_RED[i], LED_OFF);
  delay(200);
}

// ── Blink YELLOW on all lanes (fog warning) ───────────────────────────────────
void blinkYellowAll() {
  for (int i = 0; i < 3; i++) setLight((Lane)i, false, true, false);
  delay(150);
  for (int i = 0; i < 3; i++) setLight((Lane)i, false, false, false);
  delay(150);
}

// ── Buzzer (stays sourcing mode) ──────────────────────────────────────────────
void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); }
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  }


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 7: SEND COMMANDS TO CAM
// ═══════════════════════════════════════════════════════════════════════════

void sendToCAM(const char* msg) {
  if (!camIPKnown) return;
  udp.beginPacket(camIP, CAM_PORT);
  udp.print(msg);
  udp.endPacket();
  DBGF("[CMD→CAM] %s\n", msg);
}

void sendCaptureCommand(Lane lane) {
  char buf[32];
  snprintf(buf, sizeof(buf), "CAPTURE:%s", LANE_NAMES[lane]);
  sendToCAM(buf);
}

void sendPatrolCommand(Lane greenLane, int durationSec) {
  char buf[48];
  snprintf(buf, sizeof(buf), "PATROL:%s,DUR:%d", LANE_NAMES[greenLane], durationSec);
  sendToCAM(buf);
}

void sendACK(Lane lane, int greenTime) {
  char buf[32];
  snprintf(buf, sizeof(buf), "ACK:%s,GT:%d", LANE_NAMES[lane], greenTime);
  sendToCAM(buf);
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 8: PACKET PARSER
// ═══════════════════════════════════════════════════════════════════════════

int extractInt(String& msg, const char* key) {
  String k = String(key) + ":";
  int idx = msg.indexOf(k);
  if (idx < 0) return 0;
  return msg.substring(idx + k.length()).toInt();
}

Lane extractLane(String& msg) {
  if (msg.indexOf("WEST")  >= 0) return WEST;
  if (msg.indexOf("NORTH") >= 0) return NORTH;
  if (msg.indexOf("EAST")  >= 0) return EAST;
  return NONE_LANE;
}

void parseCNT(String& msg) {
  Lane lane = extractLane(msg);
  if (lane == NONE_LANE) return;
  counts[lane].cars        = extractInt(msg, "CR");
  counts[lane].threewheels = extractInt(msg, "TW");
  counts[lane].buses       = extractInt(msg, "BS");
  counts[lane].trucks      = extractInt(msg, "TK");
  counts[lane].motorbikes  = extractInt(msg, "MB");
  counts[lane].vans        = extractInt(msg, "VN");
  counts[lane].ambulance   = extractInt(msg, "AM") == 1;
  counts[lane].fireTruck   = extractInt(msg, "FT") == 1;
  counts[lane].pedestrian  = extractInt(msg, "PD") == 1;
  counts[lane].quality     = extractInt(msg, "QC");
  counts[lane].fresh       = true;
  DBGF("[PARSE] %s CR:%d TW:%d BS:%d TK:%d MB:%d VN:%d AM:%d FT:%d PD:%d QC:%d\n",
    LANE_NAMES[lane],
    counts[lane].cars, counts[lane].threewheels, counts[lane].buses,
    counts[lane].trucks, counts[lane].motorbikes, counts[lane].vans,
    counts[lane].ambulance, counts[lane].fireTruck,
    counts[lane].pedestrian, counts[lane].quality);
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 9: RECEIVE PACKET FROM CAM
// ═══════════════════════════════════════════════════════════════════════════

String waitForCAMPacket(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    int sz = udp.parsePacket();
    if (sz > 0) {
      if (!camIPKnown) {
        camIP = udp.remoteIP();
        camIPKnown = true;
        DBGF("[UDP] CAM IP: %s\n", camIP.toString().c_str());
      }
      udp.read(udpBuffer, sizeof(udpBuffer));
      udpBuffer[sz] = '\0';
      lastCamPacket = millis();
      String msg = String(udpBuffer);
      msg.trim();
      DBGF("[UDP←CAM] %s\n", msg.c_str());
      return msg;
    }
    delay(20);
  }
  return "";
}

void pollPackets() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;
  if (!camIPKnown) { camIP = udp.remoteIP(); camIPKnown = true; }
  udp.read(udpBuffer, sizeof(udpBuffer));
  udpBuffer[sz] = '\0';
  lastCamPacket = millis();
  String msg = String(udpBuffer);
  msg.trim();
  DBGF("[UDP←CAM] %s\n", msg.c_str());

  if (msg.startsWith("CNT:"))       parseCNT(msg);
  if (msg.startsWith("FOG:"))       fogMode = (extractInt(msg, "QC") < FOG_QUALITY_THRESHOLD);
  if (msg.startsWith("VIO:")) {
    Lane v = extractLane(msg);
    DBGF("[VIO] ⚠️  Violation on %s!\n", v == NONE_LANE ? "?" : LANE_NAMES[v]);
  }
  if (msg.startsWith("CAM:READY"))   { DBG("[SYS] CAM online"); fallbackMode = false; }
  if (msg.startsWith("CAM:TIMEOUT")) { DBG("[SYS] CAM timeout"); fallbackMode = true; }
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 10: DYNAMIC GREEN TIME CALCULATOR
// ═══════════════════════════════════════════════════════════════════════════

int calculateGreenTime(Lane lane) {
  LaneCount& c = counts[lane];

  // Total countable vehicles (excludes flags)
  int total = c.cars + c.threewheels + c.buses + c.trucks + c.motorbikes + c.vans;
  if (total == 0) { DBG("[TIMING] Empty lane → minimum"); return MIN_GREEN_TIME; }

  // Weighted score per vehicle type:
  //   motorbike   = 1.0  (small, fast)
  //   threewheel  = 1.5  (medium)
  //   car         = 2.0  (standard)
  //   van         = 2.5  (larger than car)
  //   truck       = 3.5  (large, slow)
  //   bus         = 4.0  (largest, most passengers)
  float score = (c.motorbikes  * 1.0f) +
                (c.threewheels * 1.5f) +
                (c.cars        * 2.0f) +
                (c.vans        * 2.5f) +
                (c.trucks      * 3.5f) +
                (c.buses       * 4.0f);

  int t = (int)(BASE_GREEN_TIME + score * TIME_PER_UNIT);

  // Lane type classification
  float heavyRatio = (float)(c.buses + c.trucks)          / total;
  float fastRatio  = (float)(c.motorbikes + c.threewheels) / total;

  if      (heavyRatio > 0.30f) { t += 10; DBGF("[TIMING] %s=HEAVY  +10s\n", LANE_NAMES[lane]); }
  else if (fastRatio  > 0.70f) { t -=  5; DBGF("[TIMING] %s=FAST   -5s\n",  LANE_NAMES[lane]); }

  t = constrain(t, MIN_GREEN_TIME, MAX_GREEN_TIME);
  DBGF("[TIMING] %s green=%ds (score=%.1f)\n", LANE_NAMES[lane], t, score);
  return t;
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 11: REQUEST CAPTURE FROM CAM
// ═══════════════════════════════════════════════════════════════════════════

int requestCapture(Lane lane) {
  DBGF("[CAPTURE] Requesting %s capture\n", LANE_NAMES[lane]);
  faceLane(lane);
  sendCaptureCommand(lane);

  unsigned long start = millis();
  while (millis() - start < CAM_RESPONSE_TIMEOUT) {
    String msg = waitForCAMPacket(500);
    if (msg.startsWith("CNT:")) {
      parseCNT(msg);
      Lane rcvLane = extractLane(msg);
      if (counts[rcvLane].quality < FOG_QUALITY_THRESHOLD) {
        DBG("[CAPTURE] Fog detected"); fogMode = true;
      }
      int gt = calculateGreenTime(rcvLane);
      sendACK(rcvLane, gt);
      return gt;
    }
    if (msg.startsWith("CAM:TIMEOUT")) { fallbackMode = true; return FALLBACK_GREEN_TIME; }
    if (msg.startsWith("FOG:"))        { fogMode = true; }
  }
  DBG("[CAPTURE] No response — fallback");
  fallbackMode = true;
  return FALLBACK_GREEN_TIME;
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 12: PEDESTRIAN SAFETY DELAY
// ═══════════════════════════════════════════════════════════════════════════

void handlePedestrianDelay(Lane lane) {
  if (!counts[lane].pedestrian) return;
  DBGF("[PED] Pedestrian on %s — delaying GREEN\n", LANE_NAMES[lane]);
  for (int i = 0; i < PED_MAX_DELAYS; i++) {
    delay(PED_DELAY_MS);
    pollPackets();
    if (!counts[lane].pedestrian) { DBG("[PED] Cleared"); return; }
    DBGF("[PED] Extension %d/%d\n", i + 1, PED_MAX_DELAYS);
  }
  DBG("[PED] Max extensions — proceeding");
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 13: AMBULANCE / FIRE TRUCK OVERRIDE
// ═══════════════════════════════════════════════════════════════════════════

void handleEmergency(Lane emergLane, Lane currentGreen) {
  DBGF("[EMERGENCY] 🚨 Override → %s\n", LANE_NAMES[emergLane]);
  if (currentGreen != NONE_LANE && currentGreen != emergLane) {
    sendToCAM("STOP_PATROL");
    transitionToRed(currentGreen);
  }
  setGreen(emergLane);
  buzzerOn();

  unsigned long start = millis();
  while (millis() - start < AMBULANCE_MIN_GREEN) { pollPackets(); delay(100); }

  unsigned long clearStart = millis();
  while (millis() - clearStart < 10000) {
    pollPackets();
    if (!counts[emergLane].ambulance && !counts[emergLane].fireTruck) {
      DBG("[EMERGENCY] Cleared"); break;
    }
    delay(200);
  }
  delay(AMBULANCE_CLEARANCE);
  buzzerOff();
  allRed();
  delay(500);
  DBG("[EMERGENCY] Resuming");
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 14: FALLBACK MODE
// ═══════════════════════════════════════════════════════════════════════════

void runFallbackCycle() {
  DBG("[FALLBACK] Fixed 20s rotation");
  for (int i = 0; i < 3; i++) {
    Lane lane = cycleOrder[i];
    faceLane(lane);
    setGreen(lane);
    unsigned long start = millis();
    while (millis() - start < (FALLBACK_GREEN_TIME * 1000UL)) {
      pollPackets();
      if (!fallbackMode) { transitionToRed(lane); allRed(); return; }
      delay(100);
    }
    transitionToRed(lane);
  }
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 15: FOG MODE
// ═══════════════════════════════════════════════════════════════════════════

void runFogMode() {
  DBG("[FOG] ⚠️  Poor visibility — yellow blink warning");
  // Blink yellow on all lanes for 5 seconds
  unsigned long start = millis();
  while (millis() - start < 5000) {
    blinkYellowAll();
  }
  allRed();
  delay(500);
  // Then run fixed 20s fallback rotation
  runFallbackCycle();
  fogMode = false;
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 16: MAIN GREEN PHASE
// ═══════════════════════════════════════════════════════════════════════════

void runGreenPhase(Lane lane, int greenTimeSec) {
  handlePedestrianDelay(lane);
  setGreen(lane);
  sendPatrolCommand(lane, greenTimeSec);

  unsigned long start = millis();
  unsigned long durMs = (unsigned long)greenTimeSec * 1000UL;
  DBGF("[GREEN] %s GREEN for %d s\n", LANE_NAMES[lane], greenTimeSec);

  while (millis() - start < durMs) {
    pollPackets();
    for (int i = 0; i < 3; i++) {
      if (i == (int)lane) continue;
      if (counts[i].ambulance || counts[i].fireTruck) {
        DBGF("[GREEN] Emergency on %s!\n", LANE_NAMES[i]);
        handleEmergency((Lane)i, lane);
        return;
      }
    }
    if (fogMode) { sendToCAM("STOP_PATROL"); transitionToRed(lane); runFogMode(); return; }
    delay(100);
  }
  sendToCAM("STOP_PATROL");
  transitionToRed(lane);
  counts[lane].fresh = false;
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 17: SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  DBG("\n[SYSTEM] Dynamic Green Light — ESP32 Master v3.0 (Pi + YOLOv8)");

  setupLights();   // All LEDs OFF on boot (HIGH in sinking mode)
  setupServo();
  setupWiFi();

  allRed();
  DBG("[SYSTEM] All RED — waiting for Raspberry Pi...\n");

  // Wait up to 15s for Pi
  unsigned long waitStart = millis();
  while (!camIPKnown && millis() - waitStart < 15000) {
    String msg = waitForCAMPacket(500);
    if (msg.startsWith("CAM:READY")) {
      camIP = udp.remoteIP();
      camIPKnown = true;
      DBG("[SYSTEM] Raspberry Pi online!");
    }
  }
  if (!camIPKnown) {
    DBG("[SYSTEM] Pi not found — fallback mode");
    fallbackMode = true;
  }
}


// ═══════════════════════════════════════════════════════════════════════════
// SECTION 18: MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  checkWiFi();
  pollPackets();

  if (fogMode)      { runFogMode();      return; }
  if (fallbackMode) { runFallbackCycle(); return; }

  Lane currentLane = cycleOrder[cycleIndex];
  Lane nextLane    = cycleOrder[(cycleIndex + 1) % 3];

  DBGF("\n[CYCLE] ===== %s TURN =====\n", LANE_NAMES[currentLane]);

  // STEP 1: Capture current lane
  int greenTime = requestCapture(currentLane);
  if (fallbackMode || fogMode) return;

  // STEP 2: Set GREEN immediately
  faceLane(currentLane);
  handlePedestrianDelay(currentLane);
  setGreen(currentLane);
  DBGF("[CYCLE] %s GREEN for %d s\n", LANE_NAMES[currentLane], greenTime);

  // STEP 3: Preload next lane
  DBGF("[CYCLE] Preloading %s count\n", LANE_NAMES[nextLane]);
  requestCapture(nextLane);
  if (fallbackMode || fogMode) return;

  // STEP 4: Servo back + start patrol
  faceLane(currentLane);
  sendPatrolCommand(currentLane, greenTime);

  // STEP 5: Wait green time
  unsigned long elapsed = millis();
  unsigned long greenMs = (unsigned long)greenTime * 1000UL;

  while (millis() - elapsed < greenMs) {
    pollPackets();
    for (int i = 0; i < 3; i++) {
      if (i == (int)currentLane) continue;
      if (counts[i].ambulance || counts[i].fireTruck) {
        sendToCAM("STOP_PATROL");
        handleEmergency((Lane)i, currentLane);
        return;
      }
    }
    if (fogMode) {
      sendToCAM("STOP_PATROL");
      transitionToRed(currentLane);
      runFogMode();
      return;
    }
    delay(100);
  }

  // STEP 6: End green
  sendToCAM("STOP_PATROL");
  transitionToRed(currentLane);

  // STEP 7: Advance
  cycleIndex = (cycleIndex + 1) % 3;
  allRed();
  delay(500);
}

// ████████████████████████████████████████████████████████████████████████████
// END OF ESP32 MASTER CODE v2.0 — SINKING MODE
// ████████████████████████████████████████████████████████████████████████████
