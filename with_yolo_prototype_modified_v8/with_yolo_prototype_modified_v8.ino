/*
  STAP ESP32 Controller — Firmware v18.4 (Watchdog Decoupling & Buffer Fix)
  =====================================================================
  Increases the hardware watchdog tolerance window to 6000ms and implements
  state-change verification to minimize I2C transactions.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// =============================================================
// 1. PIN CONFIGURATION
// =============================================================
const int N_YELLOW = 1;  const int N_RED = 2;  const int N_GREEN = 3;
const int E_YELLOW = 4;  const int E_RED = 5;  const int E_GREEN = 6;
const int W_YELLOW = 9;  const int W_RED = 10; const int W_GREEN = 11;
const int S_YELLOW = 12; const int S_RED = 13; const int S_GREEN = 14;

const int ledWhite  = 17; const int ledBlue   = 18;
const int ledRed    = 19; const int ledYellow = 20;
const int ledSouth  = 21; const int ledWest   = 25;
const int ledEast   = 26; const int ledNorth  = 27;

const int latchPin = 5;  const int clockPin = 18;
const int dataPin  = 19; const int oePin    = 4;

#define ADDR_NORTH   0x70
#define ADDR_SOUTH   0x72
#define ADDR_EAST    0x74
#define ADDR_WEST    0x76

const int btnAuto      = 12; const int btnManual    = 13;
const int btnEmergency = 14; const int btnManHazard = 26;
const int btnGoNorth   = 27; const int btnGoEast    = 25;
const int btnGoSouth   = 33; const int btnGoWest    = 32;

const int rainSensorPin  = 34;
const int RAIN_THRESHOLD = 3000;

// =============================================================
// 2. OBJECTS
// =============================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_7segment timerNorth = Adafruit_7segment();
Adafruit_7segment timerSouth = Adafruit_7segment();
Adafruit_7segment timerEast  = Adafruit_7segment();
Adafruit_7segment timerWest  = Adafruit_7segment();

uint32_t lightState = 0;

// =============================================================
// 3. TIMING CONSTANTS
// =============================================================
const int YELLOW_TIME = 3;

const int    FALLBACK_GREEN[] = {50, 50, 39, 35};
const String FALLBACK_LANE[]  = {"NORTH", "SOUTH", "EAST", "WEST"};
const int    FALLBACK_COUNT   = 4;

// EXTENDED WATCHDOG BUFFER WINDOW (6.0 Seconds)
const unsigned long WATCHDOG_THRESHOLD = 6000; 

// =============================================================
// 4. STATE VARIABLES
// =============================================================
enum Mode { AUTO, MANUAL };
Mode currentMode = AUTO;

enum OnlineSignal { SIG_GREEN, SIG_YELLOW, SIG_WAITING };
OnlineSignal  onlineSignal      = SIG_WAITING;
String        activeLane        = "NORTH";
int           greenCountdown    = 0;
unsigned long yellowStartMillis = 0;

// Watchdog tracking variables
unsigned long lastCommMillis       = 0; 
bool          isOffline            = false;
int           fallbackIdx          = 0;
int           fallbackCountdown    = 50;
bool          fallbackInYellow     = false;
unsigned long fallbackYellowStart  = 0;

unsigned long lastTickMillis       = 0;
unsigned long lastTelemetryTime    = 0;
bool          rainDetected         = false;

// Memory verification caches to eliminate duplicate I2C traffic loads
int lastN = -2, lastS = -2, lastE = -2, lastW = -2;
String lastLine1 = "", lastLine2 = "", lastLine3 = "", lastLine4 = "";

// =============================================================
// 5. MANUAL MODE STATE
// =============================================================
enum ManualState {
  MAN_STOPPED, MAN_TRANSITION,
  MAN_N_GO, MAN_S_GO, MAN_E_GO, MAN_W_GO,
  MAN_EMERGENCY
};

ManualState manualState             = MAN_STOPPED;
ManualState manualTarget            = MAN_STOPPED;
ManualState prevManualState         = MAN_STOPPED;
bool          manualHazardActive    = false;
unsigned long manualTransitionStart = 0;

// =============================================================
// 6. FORWARD DECLARATIONS
// =============================================================
void updateShiftRegister(); void syncIndicatorLEDs();
void setAllRed();
void setNorthGo(); void setSouthGo(); void setEastGo(); void setWestGo();
void setYellow(String lane);
void setTransitionLights(ManualState prev);
void blinkYellows();
void showCentered(Adafruit_7segment &disp, int number);
void updateTimers(int n, int s, int e, int w, bool forceClear);
void updateLCD(String l1, String l2, String l3, String l4);
bool checkButtonPress(int pin);
void parsePythonCommand(String msg);
void runAutoOnline(unsigned long ms, bool forceUpdate);
void runAutoFallback(unsigned long ms, bool forceUpdate);
void handleManual(unsigned long ms, bool forceUpdate);
void broadcastManualStates();

// =============================================================
// 7. SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  pinMode(latchPin, OUTPUT); pinMode(clockPin, OUTPUT);
  pinMode(dataPin,  OUTPUT); pinMode(oePin,    OUTPUT);
  digitalWrite(oePin, HIGH);
  lightState = 0;
  updateShiftRegister();

  Wire.begin();
  timerNorth.begin(ADDR_NORTH); timerNorth.setBrightness(10);
  timerSouth.begin(ADDR_SOUTH); timerSouth.setBrightness(10);
  timerEast.begin(ADDR_EAST);   timerEast.setBrightness(10);
  timerWest.begin(ADDR_WEST);   timerWest.setBrightness(10);

  lcd.init(); lcd.backlight();
  updateLCD("====================", " STAP SYSTEM READY  ", "   WATCHDOG ACTIVE  ", "====================");

  pinMode(rainSensorPin,  INPUT);
  pinMode(btnAuto,        INPUT_PULLUP); pinMode(btnManual,    INPUT_PULLUP);
  pinMode(btnEmergency,   INPUT_PULLUP); pinMode(btnManHazard, INPUT_PULLUP);
  pinMode(btnGoNorth,     INPUT_PULLUP); pinMode(btnGoEast,    INPUT_PULLUP);
  pinMode(btnGoSouth,     INPUT_PULLUP); pinMode(btnGoWest,    INPUT_PULLUP);

  setAllRed();
  lastCommMillis = millis(); 
  delay(1000);
  digitalWrite(oePin, LOW);
}

// =============================================================
// 8. MAIN LOOP
// =============================================================
void loop() {
  unsigned long ms = millis();
  rainDetected = (analogRead(rainSensorPin) < RAIN_THRESHOLD);

  if (ms - lastTelemetryTime >= 400) {
    lastTelemetryTime = ms;
    Serial.println("RAIN:" + String(rainDetected ? "1" : "0") + ",MODE:" + String(currentMode == MANUAL ? "MANUAL" : "AUTO"));
    if (currentMode == MANUAL) {
      broadcastManualStates();
    }
    Serial.flush();
  }

  static String buf = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length() > 0) {
        parsePythonCommand(buf);
        buf = "";
        break;
      }
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }

  if (checkButtonPress(btnAuto)) {
    currentMode    = AUTO;
    lastCommMillis = ms; 
    isOffline      = false;
    updateShiftRegister();
    updateTimers(-1, -1, -1, -1, true); // Force clean displays immediately
  } else if (checkButtonPress(btnManual)) {
    currentMode  = MANUAL;
    manualState  = MAN_STOPPED;
    manualTarget = MAN_STOPPED;
    setAllRed();
    broadcastManualStates();
    updateTimers(-1, -1, -1, -1, true);
  }

  if (currentMode == AUTO) {
    if (ms - lastCommMillis >= WATCHDOG_THRESHOLD) {
      if (!isOffline) {
        isOffline           = true;
        fallbackIdx         = 0;
        fallbackCountdown   = FALLBACK_GREEN[0];
        fallbackInYellow    = false;
        fallbackYellowStart = 0;
        onlineSignal        = SIG_WAITING;
        Serial.println("ALERT:WATCHDOG_TRIPPED_FALLBACK_ENGAGED");
        updateTimers(-1, -1, -1, -1, true); // Clear display remnants
      }
    }
  }

  bool executeDisplayTick = false;
  if (ms - lastTickMillis >= 1000) {
    lastTickMillis = ms;
    executeDisplayTick = true; 
    if (currentMode == AUTO) {
      if (!isOffline && onlineSignal == SIG_GREEN && greenCountdown > 0)
        greenCountdown--;
      if (isOffline && !fallbackInYellow && fallbackCountdown > 0)
        fallbackCountdown--;
    }
  }

  switch (currentMode) {
    case AUTO:
      if (!isOffline) runAutoOnline(ms, executeDisplayTick);
      else            runAutoFallback(ms, executeDisplayTick);
      break;
    case MANUAL:
      handleManual(ms, executeDisplayTick);
      break;
  }
}

// =============================================================
// 9. PYTHON COMMAND PARSER
// =============================================================
void parsePythonCommand(String msg) {
  if (msg.startsWith("PHASE:")   ||
      msg.startsWith("YELLOW:")  ||
      msg.startsWith("PING:")    ||
      msg.startsWith("MODE:")    ||
      msg.startsWith("MANUAL_LIGHT:") ||
      msg.startsWith("HAZARD:")  ||
      msg.startsWith("EMERGENCY_OVERRIDE:")) {
      
    lastCommMillis = millis();
    isOffline      = false;
  } else {
    return;
  }

  if (msg.startsWith("PING:")) {
    return;
  }

  if (msg.startsWith("MODE:")) {
    String mode = msg.substring(5);
    mode.trim();

    if (mode == "AUTO") {
      currentMode        = AUTO;
      manualHazardActive = false;
      manualState        = MAN_STOPPED;
      manualTarget       = MAN_STOPPED;
      updateShiftRegister();
    } else if (mode == "MANUAL") {
      currentMode  = MANUAL;
      manualState  = MAN_STOPPED;
      manualTarget = MAN_STOPPED;
      manualHazardActive = false;
      setAllRed();
      broadcastManualStates();
    } else if (mode == "HAZARD") {
      currentMode        = MANUAL;
      manualHazardActive = true;
      manualState        = MAN_STOPPED;
      manualTarget       = MAN_STOPPED;
      updateShiftRegister();
    }
    return;
  }

  if (msg.startsWith("HAZARD:")) {
    currentMode        = MANUAL;
    manualHazardActive = true;
    manualState        = MAN_STOPPED;
    blinkYellows();
    return;
  }

  if (msg.startsWith("MANUAL_LIGHT:")) {
    String payload = msg.substring(13);
    payload.trim();
    int commaIdx = payload.indexOf(',');
    if (commaIdx == -1) return;

    String lane  = payload.substring(0, commaIdx);
    String state = payload.substring(commaIdx + 1);
    lane.trim(); state.trim();

    if      (lane == "NORTH" && state == "GREEN")  { manualState = MAN_N_GO; setNorthGo(); }
    else if (lane == "SOUTH" && state == "GREEN")  { manualState = MAN_S_GO; setSouthGo(); }
    else if (lane == "EAST"  && state == "GREEN")  { manualState = MAN_E_GO; setEastGo();  }
    else if (lane == "WEST"  && state == "GREEN")  { manualState = MAN_W_GO; setWestGo();  }
    else if (state == "RED") { manualState = MAN_STOPPED; setAllRed(); }
    else if (state == "YELLOW") { setYellow(lane); }

    updateTimers(-1, -1, -1, -1, true);
    broadcastManualStates(); 
    return;
  }

  if (msg.startsWith("EMERGENCY_OVERRIDE:")) {
    String lane = msg.substring(19);
    lane.trim();
    currentMode        = AUTO;
    manualHazardActive = false;
    onlineSignal       = SIG_YELLOW;
    yellowStartMillis  = millis();
    setYellow(activeLane);
    activeLane = lane;
    return;
  }

  if (msg.startsWith("YELLOW:")) {
    String lane = msg.substring(7);
    lane.trim();
    activeLane        = lane;
    onlineSignal      = SIG_YELLOW;
    yellowStartMillis = millis();
    return;
  }

  if (msg.startsWith("PHASE:")) {
    if (onlineSignal == SIG_YELLOW) {
      unsigned long elapsed = millis() - yellowStartMillis;
      if (elapsed < (unsigned long)(YELLOW_TIME * 1000)) return;
    }

    int    commaIdx = msg.indexOf(',');
    String lane;
    int    duration = greenCountdown;

    if (commaIdx != -1) {
      lane = msg.substring(6, commaIdx);
      int durIdx = msg.indexOf("DURATION:");
      if (durIdx != -1) duration = msg.substring(durIdx + 9).toInt();
    } else {
      lane = msg.substring(6);
    }
    lane.trim();

    activeLane = lane;
    greenCountdown = duration;
    onlineSignal   = SIG_GREEN;
    return;
  }
}

// =============================================================
# 10. AUTO ONLINE MODE
// =============================================================
void runAutoOnline(unsigned long ms, bool forceUpdate) {
  if (onlineSignal == SIG_GREEN) {
    if      (activeLane == "NORTH") setNorthGo();
    else if (activeLane == "SOUTH") setSouthGo();
    else if (activeLane == "EAST")  setEastGo();
    else if (activeLane == "WEST")  setWestGo();
  } else if (onlineSignal == SIG_YELLOW) {
    setYellow(activeLane);
    if (ms - yellowStartMillis >= (unsigned long)(YELLOW_TIME * 1000)) {
      onlineSignal = SIG_WAITING;
    }
  } else {
    setAllRed();
  }

  if (forceUpdate) {
    int disp = 0;
    if (onlineSignal == SIG_GREEN) disp = greenCountdown;
    else if (onlineSignal == SIG_YELLOW) disp = max(0, YELLOW_TIME - (int)((ms - yellowStartMillis) / 1000));

    if (onlineSignal == SIG_GREEN || onlineSignal == SIG_YELLOW) {
      if      (activeLane == "NORTH") updateTimers(disp, -1, -1, -1, false);
      else if (activeLane == "SOUTH") updateTimers(-1, disp, -1, -1, false);
      else if (activeLane == "EAST")  updateTimers(-1, -1, disp, -1, false);
      else if (activeLane == "WEST")  updateTimers(-1, -1, -1, disp, false);
    } else {
      updateTimers(-1, -1, -1, -1, false);
    }

    String title  = rainDetected ? "- AUTO (RAIN MODE) -" : "- AUTO (SMART AI)  -";
    String sigStr = (onlineSignal == SIG_GREEN) ? "GREEN" : ((onlineSignal == SIG_YELLOW) ? "YELLOW" : "ALL RED");
    String l2_text = "ACTIVE LANE: " + activeLane;
    String l3_text = "SIGNAL: " + sigStr;
    String l4_text = "COUNTDOWN: " + String(disp) + "s";
    updateLCD(title, l2_text, l3_text, l4_text);
  }
}

// =============================================================
// 11. AUTO FALLBACK MODE
// =============================================================
void runAutoFallback(unsigned long ms, bool forceUpdate) {
  if (fallbackInYellow) {
    unsigned long elapsed = ms - fallbackYellowStart;
    if (elapsed >= (unsigned long)(YELLOW_TIME * 1000)) {
      fallbackInYellow  = false;
      fallbackIdx       = (fallbackIdx + 1) % FALLBACK_COUNT;
      fallbackCountdown = FALLBACK_GREEN[fallbackIdx];
    }
  }

  if (!fallbackInYellow && fallbackCountdown <= 0) {
    fallbackInYellow    = true;
    fallbackYellowStart = ms;
  }

  String fbLane = FALLBACK_LANE[fallbackIdx];
  if (fallbackInYellow) {
    setYellow(fbLane);
  } else {
    if      (fbLane == "NORTH") setNorthGo();
    else if (fbLane == "SOUTH") setSouthGo();
    else if (fbLane == "EAST")  setEastGo();
    else if (fbLane == "WEST")  setWestGo();
  }

  if (forceUpdate) {
    int currentRemaining = fallbackInYellow ? max(0, YELLOW_TIME - (int)((ms - fallbackYellowStart) / 1000)) : fallbackCountdown;

    int timers[4];
    for (int i = 0; i < 4; i++) {
      if (fallbackIdx == i) {
        timers[i] = currentRemaining; 
      } else {
        int totalWait = currentRemaining;
        if (!fallbackInYellow) totalWait += YELLOW_TIME; 
        
        int checkIdx = (fallbackIdx + 1) % 4;
        while (checkIdx != i) {
          totalWait += FALLBACK_GREEN[checkIdx] + YELLOW_TIME;
          checkIdx = (checkIdx + 1) % 4;
        }
        timers[i] = totalWait; 
      }
    }

    updateTimers(timers[0], timers[1], timers[2], timers[3], false);
    String sig = fallbackInYellow ? "YELLOW" : "GREEN";
    
    updateLCD("[ALARM] OFFLINE MODE", 
              "COMMUNICATION LOSS", 
              "LANE: " + fbLane + " [" + sig + "]", 
              "COUNTDOWN: " + String(currentRemaining) + "s");
  }
}

// =============================================================
// 12. MANUAL OVERRIDE
// =============================================================
void handleManual(unsigned long ms, bool forceUpdate) {
  if (checkButtonPress(btnEmergency)) {
    if (manualState == MAN_EMERGENCY) {
      manualState = MAN_STOPPED; manualTarget = MAN_STOPPED;
      setAllRed(); updateShiftRegister();
    } else {
      manualState = MAN_EMERGENCY; manualTarget = MAN_STOPPED;
      manualHazardActive = false; updateShiftRegister();
    }
    broadcastManualStates();
    updateTimers(-1, -1, -1, -1, true);
  }
  if (manualState == MAN_EMERGENCY) {
    if (forceUpdate) {
      updateLCD("[EMERGENCY OVERRIDE]", "INTERSECTION LOCKED", "ALL APPROACHES: RED", "PRESS EMG TO CLEAR ");
    }
    setAllRed(); 
    return;
  }

  if (checkButtonPress(btnManHazard)) {
    manualHazardActive = !manualHazardActive;
    if (manualHazardActive) { manualState = MAN_STOPPED; manualTarget = MAN_STOPPED; }
    updateShiftRegister();
    broadcastManualStates();
    updateTimers(-1, -1, -1, -1, true);
  }
  if (manualHazardActive) {
    if ((ms / 500) % 2 == 0) blinkYellows();
    else { lightState &= 0xFFFF0000; updateShiftRegister(); }
    
    if (forceUpdate) {
      updateLCD("- MANUAL OVERRIDE - ", "STATUS: HAZARD ZONE ", "FLASHING YELLOW LENS", "YIELD ALL APPROACHES");
    }
    return;
  }

  if (manualState != MAN_TRANSITION) {
    if      (checkButtonPress(btnGoNorth) && manualState != MAN_N_GO) { prevManualState = manualState; manualTarget = MAN_N_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoSouth) && manualState != MAN_S_GO) { prevManualState = manualState; manualTarget = MAN_S_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoEast)  && manualState != MAN_E_GO) { prevManualState = manualState; manualTarget = MAN_E_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoWest)  && manualState != MAN_W_GO) { prevManualState = manualState; manualTarget = MAN_W_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    if (manualState == MAN_TRANSITION) {
      broadcastManualStates();
      updateTimers(-1, -1, -1, -1, true);
    }
  }

  if (manualState == MAN_TRANSITION) {
    setTransitionLights(prevManualState); updateShiftRegister();
    long elapsed   = ms - manualTransitionStart;
    int  remaining = max(0, YELLOW_TIME - (int)(elapsed / 1000));
    
    if (forceUpdate) {
      if      (prevManualState == MAN_N_GO) updateTimers(remaining, -1, -1, -1, false);
      else if (prevManualState == MAN_S_GO) updateTimers(-1, remaining, -1, -1, false);
      else if (prevManualState == MAN_E_GO) updateTimers(-1, -1, remaining, -1, false);
      else if (prevManualState == MAN_W_GO) updateTimers(-1, -1, -1, remaining, false);
      updateLCD("- MANUAL OVERRIDE - ", "SWITCHING CHANNELS  ", "CLEARANCE: " + String(remaining) + "s   ", "CHANGING SIGNAL HEAD");
    }
    if (elapsed >= (long)(YELLOW_TIME * 1000)) { 
      manualState = manualTarget; 
      broadcastManualStates();
      updateTimers(-1, -1, -1, -1, true);
    }
    return;
  }

  if (forceUpdate) {
    if      (manualState == MAN_N_GO) { updateLCD("- MANUAL OVERRIDE - ", "ACTIVE FLOW: NORTH  ", "MANUAL ROUTING REQ  ", "SELECT NEXT APPROACH"); }
    else if (manualState == MAN_S_GO) { updateLCD("- MANUAL OVERRIDE - ", "ACTIVE FLOW: SOUTH  ", "MANUAL ROUTING REQ  ", "SELECT NEXT APPROACH"); }
    else if (manualState == MAN_E_GO) { updateLCD("- MANUAL OVERRIDE - ", "ACTIVE FLOW: EAST   ", "MANUAL ROUTING REQ  ", "SELECT NEXT APPROACH"); }
    else if (manualState == MAN_W_GO) { updateLCD("- MANUAL OVERRIDE - ", "ACTIVE FLOW: WEST   ", "MANUAL ROUTING REQ  ", "SELECT NEXT APPROACH"); }
    else                              { updateLCD("- MANUAL OVERRIDE - ", "REMOTE WORKSTATION  ", "ALL APPROACHES: RED ", "AWAITING SERIAL REQ "); }
  }

  if      (manualState == MAN_N_GO) setNorthGo();
  else if (manualState == MAN_S_GO) setSouthGo();
  else if (manualState == MAN_E_GO) setEastGo();
  else if (manualState == MAN_W_GO) setWestGo();
  else                              setAllRed();
}

// =============================================================
// 12b. EXPLICIT HARDWARE LIGHT STATE BROADCASTER
// =============================================================
void broadcastManualStates() {
  String lanes[] = {"NORTH", "SOUTH", "EAST", "WEST"};
  int greens[]   = {N_GREEN, S_GREEN, E_GREEN, W_GREEN};
  int yellows[]  = {N_YELLOW, S_YELLOW, E_YELLOW, W_YELLOW};
  
  for (int i = 0; i < 4; i++) {
    String currentLamp = "RED";
    if (bitRead(lightState, greens[i]))       currentLamp = "GREEN";
    else if (bitRead(lightState, yellows[i])) currentLamp = "YELLOW";
    
    Serial.println("STATE:" + lanes[i] + "," + currentLamp);
  }
  Serial.flush();
}

// =============================================================
// 13. SHIFT REGISTER & LIGHT PRESETS
// =============================================================
void syncIndicatorLEDs() {
  lightState &= 0x0000FFFF;
  if (currentMode == AUTO) {
    bitSet(lightState, ledBlue);
  } else {
    bitSet(lightState, ledWhite);
    if      (manualState == MAN_EMERGENCY)                    bitSet(lightState, ledRed);
    else if (manualHazardActive)                               bitSet(lightState, ledYellow);
    else {
      if (manualState == MAN_N_GO || manualTarget == MAN_N_GO) bitSet(lightState, ledNorth);
      if (manualState == MAN_S_GO || manualTarget == MAN_S_GO) bitSet(lightState, ledSouth);
      if (manualState == MAN_E_GO || manualTarget == MAN_E_GO) bitSet(lightState, ledEast);
      if (manualState == MAN_W_GO || manualTarget == MAN_W_GO) bitSet(lightState, ledWest);
    }
  }
}

void updateShiftRegister() {
  syncIndicatorLEDs();
  digitalWrite(latchPin, LOW);
  for (int i = 3; i >= 0; i--) shiftOut(dataPin, clockPin, MSBFIRST, (lightState >> (i * 8)) & 0xFF);
  digitalWrite(latchPin, HIGH);
}

void setAllRed()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setNorthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_GREEN); bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setSouthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_GREEN); bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setEastGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_GREEN); bitSet(lightState,W_RED);   updateShiftRegister(); }
void setWestGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_GREEN); updateShiftRegister(); }

void setYellow(String lane) {
  lightState &= 0xFFFF0000;
  if      (lane == "NORTH") { bitSet(lightState, N_YELLOW); bitSet(lightState, S_RED); bitSet(lightState, E_RED); bitSet(lightState, W_RED); }
  else if (lane == "SOUTH") { bitSet(lightState, S_YELLOW); bitSet(lightState, N_RED); bitSet(lightState, E_RED); bitSet(lightState, W_RED); }
  else if (lane == "EAST")  { bitSet(lightState, E_YELLOW); bitSet(lightState, N_RED); bitSet(lightState, S_RED); bitSet(lightState, W_RED); }
  else if (lane == "WEST")  { bitSet(lightState, W_YELLOW); bitSet(lightState, N_RED); bitSet(lightState, S_RED); bitSet(lightState, E_RED); }
  updateShiftRegister();
}

void blinkYellows() {
  lightState &= 0xFFFF0000;
  bitSet(lightState, N_YELLOW); bitSet(lightState, S_YELLOW);
  bitSet(lightState, E_YELLOW); bitSet(lightState, W_YELLOW);
  updateShiftRegister();
}

void setTransitionLights(ManualState prev) {
  lightState &= 0xFFFF0000;
  if      (prev == MAN_N_GO) { bitSet(lightState,N_YELLOW); bitSet(lightState,S_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_S_GO) { bitSet(lightState,S_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_E_GO) { bitSet(lightState,E_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_W_GO) { bitSet(lightState,W_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,E_RED); }
  else { setAllRed(); }
}

// =============================================================
// 14. LCD ENGINE (State Cache Protected)
// =============================================================
void updateLCD(String l1, String l2, String l3, String l4) {
  if (l1 != lastLine1) { lcd.setCursor(0,0); lcd.print("                    "); lcd.setCursor(0,0); lcd.print(l1.substring(0, 20)); lastLine1 = l1; }
  if (l2 != lastLine2) { lcd.setCursor(0,1); lcd.print("                    "); lcd.setCursor(0,1); lcd.print(l2.substring(0, 20)); lastLine2 = l2; }
  if (l3 != lastLine3) { lcd.setCursor(0,2); lcd.print("                    "); lcd.setCursor(0,2); lcd.print(l3.substring(0, 20)); lastLine3 = l3; } 
  if (l4 != lastLine4) { lcd.setCursor(0,3); lcd.print("                    "); lcd.setCursor(0,3); lcd.print(l4.substring(0, 20)); lastLine4 = l4; }
}

// =============================================================
// 15. 7-SEGMENT REGISTRY MANAGEMENT (State Cache Protected)
// =============================================================
void updateTimers(int n, int s, int e, int w, bool forceClear) {
  // If the parameters match previous snapshots, bypass execution to clear the I2C bus completely
  if (forceClear || n != lastN) { showCentered(timerNorth, n); lastN = n; }
  if (forceClear || s != lastS) { showCentered(timerSouth, s); lastS = s; }
  if (forceClear || e != lastE) { showCentered(timerEast,  e); lastE = e; }
  if (forceClear || w != lastW) { showCentered(timerWest,  w); lastW = w; }
}

void showCentered(Adafruit_7segment &disp, int number) {
  if (number < 0) { disp.clear(); disp.writeDisplay(); return; }
  disp.drawColon(false);
  disp.clear();
  
  int hundreds = (number / 100) % 10;
  int tens     = (number / 10) % 10;
  int units    = number % 10;
  
  disp.writeDigitRaw(0, 0x00); 
  if (number >= 100) {
    disp.writeDigitNum(1, hundreds);
    disp.writeDigitNum(3, tens);
  } else if (number >= 10) {
    disp.writeDigitRaw(1, 0x00); 
    disp.writeDigitNum(3, tens);
  } else {
    disp.writeDigitRaw(1, 0x00); 
    disp.writeDigitRaw(3, 0x00); 
  }
  disp.writeDigitNum(4, units);
  disp.writeDisplay();
}

// =============================================================
// 16. DEBOUNCED BUTTON
// =============================================================
bool checkButtonPress(int pin) {
  static bool       init      = false;
  static int           last[40]  = {};
  static unsigned long time[40]  = {};
  if (!init) {
    for (int i = 0; i < 40; i++) { last[i] = HIGH; time[i] = 0; }
    init = true;
  }
  int  r = digitalRead(pin);
  bool p = false;
  if (r == LOW && last[pin] == HIGH && millis() - time[pin] > 50) {
    p = true; time[pin] = millis();
  }
  last[pin] = r;
  return p;
}