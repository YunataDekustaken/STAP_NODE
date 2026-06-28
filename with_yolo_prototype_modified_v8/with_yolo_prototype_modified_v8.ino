/*
  STAP ESP32 Controller — Firmware v17.7 (LCD String Optimization)
  =====================================================================
  All text screens optimized to fit within strict 20-character line lengths
  to fix clipped data arrays or trailing character overlap artifacts.
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

// Per-lane timer display I2C addresses
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

// Per-lane countdown displays
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

const unsigned long DATA_TIMEOUT = 30000;

// =============================================================
// 4. STATE VARIABLES
// =============================================================
enum Mode { AUTO, MANUAL };
Mode currentMode = AUTO;

// ── Auto online signal state ──────────────────────────────────
enum OnlineSignal { SIG_GREEN, SIG_YELLOW, SIG_WAITING };
OnlineSignal  onlineSignal      = SIG_WAITING;
String        activeLane        = "NORTH";
int           greenCountdown    = 0;
unsigned long yellowStartMillis = 0;

// ── Offline fallback state ────────────────────────────────────
unsigned long lastCommMillis      = 0;
unsigned long silenceStartMillis  = 0;
bool          isOffline           = false;
int           fallbackIdx         = 0;
int           fallbackCountdown   = 50;
bool          fallbackInYellow    = false;
unsigned long fallbackYellowStart = 0;

// ── Shared ────────────────────────────────────────────────────
unsigned long lastTickMillis    = 0;
unsigned long lastTelemetryTime = 0;
bool          rainDetected      = false;

// ── LCD cache boundaries (Deduplication system) ───────────────
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
void updateTimers(int n, int s, int e, int w);
void updateLCD(String l1, String l2, String l3, String l4);
bool checkButtonPress(int pin);
void parsePythonCommand(String msg);
void runAutoOnline(unsigned long ms);
void runAutoFallback(unsigned long ms);
void handleManual(unsigned long ms);
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
  updateLCD("====================", " ESP32 TRAFFIC CTRL ", "   SYSTEM BOOTING   ", "====================");

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
    Serial.println(
      "RAIN:" + String(rainDetected ? "1" : "0") +
      ",MODE:" + String(currentMode == MANUAL ? "MANUAL" : "AUTO") +
      ",HAZARD:" + String(manualHazardActive ? "1" : "0") +
      ",EMERGENCY:" + String(manualState == MAN_EMERGENCY ? "1" : "0")
    );
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
    currentMode        = AUTO;
    lastCommMillis     = ms;
    isOffline          = false;
    silenceStartMillis = 0;
    updateShiftRegister();
  } else if (checkButtonPress(btnManual)) {
    ManualState startingState = MAN_STOPPED;
    if (currentMode == AUTO) {
      if      (activeLane == "NORTH" && onlineSignal == SIG_GREEN) startingState = MAN_N_GO;
      else if (activeLane == "SOUTH" && onlineSignal == SIG_GREEN) startingState = MAN_S_GO;
      else if (activeLane == "EAST"  && onlineSignal == SIG_GREEN) startingState = MAN_E_GO;
      else if (activeLane == "WEST"  && onlineSignal == SIG_GREEN) startingState = MAN_W_GO;
    }
    currentMode  = MANUAL;
    manualState  = startingState;
    manualTarget = startingState;
    if (startingState == MAN_STOPPED) {
      setAllRed();
    } else {
      if      (startingState == MAN_N_GO) setNorthGo();
      else if (startingState == MAN_S_GO) setSouthGo();
      else if (startingState == MAN_E_GO) setEastGo();
      else if (startingState == MAN_W_GO) setWestGo();
    }
    broadcastManualStates();
  }

  if (currentMode == AUTO) {
    bool nowSilent = (ms - lastCommMillis > 1000);
    if (!nowSilent) {
      silenceStartMillis = 0;
      isOffline           = false;
    } else {
      if (silenceStartMillis == 0) silenceStartMillis = ms;
      if (!isOffline && (ms - silenceStartMillis >= DATA_TIMEOUT)) {
        isOffline            = true;
        fallbackIdx          = 0;
        fallbackCountdown    = FALLBACK_GREEN[0];
        fallbackInYellow     = false;
        fallbackYellowStart  = 0;
        onlineSignal         = SIG_WAITING;
      }
    }
  }

  if (ms - lastTickMillis >= 1000) {
    lastTickMillis = ms;
    if (currentMode == AUTO) {
      if (!isOffline && onlineSignal == SIG_GREEN && greenCountdown > 0)
        greenCountdown--;
      if (isOffline && !fallbackInYellow && fallbackCountdown > 0)
        fallbackCountdown--;
    }
  }

  switch (currentMode) {
    case AUTO:
      if (!isOffline) runAutoOnline(ms);
      else            runAutoFallback(ms);
      break;
    case MANUAL:
      handleManual(ms);
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
      msg.startsWith("DISPLAY:") ||
      msg.startsWith("MODE:")    ||
      msg.startsWith("MANUAL_LIGHT:") ||
      msg.startsWith("HAZARD:")  ||
      msg.startsWith("EMERGENCY_OVERRIDE:")) {
    lastCommMillis     = millis();
    silenceStartMillis = 0;
    isOffline          = false;
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
      if (currentMode != MANUAL || manualHazardActive || manualState == MAN_EMERGENCY) {
        ManualState startingState = MAN_STOPPED;
        if (currentMode == AUTO) {
          if      (activeLane == "NORTH" && onlineSignal == SIG_GREEN) startingState = MAN_N_GO;
          else if (activeLane == "SOUTH" && onlineSignal == SIG_GREEN) startingState = MAN_S_GO;
          else if (activeLane == "EAST"  && onlineSignal == SIG_GREEN) startingState = MAN_E_GO;
          else if (activeLane == "WEST"  && onlineSignal == SIG_GREEN) startingState = MAN_W_GO;
        }
        
        currentMode  = MANUAL;
        manualState  = startingState;
        manualTarget = startingState;
        manualHazardActive = false;
        
        if (startingState == MAN_STOPPED) {
          setAllRed();
        } else {
          if      (startingState == MAN_N_GO) setNorthGo();
          else if (startingState == MAN_S_GO) setSouthGo();
          else if (startingState == MAN_E_GO) setEastGo();
          else if (startingState == MAN_W_GO) setWestGo();
        }
        broadcastManualStates();
      }
    } else if (mode == "HAZARD") {
      currentMode        = MANUAL;
      manualHazardActive = true;
      manualState        = MAN_STOPPED;
      manualTarget       = MAN_STOPPED;
      updateShiftRegister();
    } else if (mode == "EMERGENCY") {
      currentMode  = MANUAL;
      manualState  = MAN_EMERGENCY;
      manualTarget = MAN_EMERGENCY;
      manualHazardActive = false;
      setAllRed();
      broadcastManualStates();
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
    lane.trim();
    state.trim();

    if      (lane == "NORTH" && state == "GREEN") {
      if (manualState != MAN_N_GO && manualState != MAN_TRANSITION) {
        prevManualState = manualState; manualTarget = MAN_N_GO; manualState = MAN_TRANSITION; manualTransitionStart = millis();
        setTransitionLights(prevManualState);
      }
    }
    else if (lane == "SOUTH" && state == "GREEN") {
      if (manualState != MAN_S_GO && manualState != MAN_TRANSITION) {
        prevManualState = manualState; manualTarget = MAN_S_GO; manualState = MAN_TRANSITION; manualTransitionStart = millis();
        setTransitionLights(prevManualState);
      }
    }
    else if (lane == "EAST"  && state == "GREEN") {
      if (manualState != MAN_E_GO && manualState != MAN_TRANSITION) {
        prevManualState = manualState; manualTarget = MAN_E_GO; manualState = MAN_TRANSITION; manualTransitionStart = millis();
        setTransitionLights(prevManualState);
      }
    }
    else if (lane == "WEST"  && state == "GREEN") {
      if (manualState != MAN_W_GO && manualState != MAN_TRANSITION) {
        prevManualState = manualState; manualTarget = MAN_W_GO; manualState = MAN_TRANSITION; manualTransitionStart = millis();
        setTransitionLights(prevManualState);
      }
    }
    else if (state == "RED") {
      manualState = MAN_STOPPED;
      setAllRed();
    }
    else if (state == "YELLOW") {
      setYellow(lane);
    }

    updateTimers(-1, -1, -1, -1);
    broadcastManualStates();
    return;
  }

  if (msg.startsWith("EMERGENCY_OVERRIDE:")) {
    String lane = msg.substring(19);
    lane.trim();

    currentMode        = AUTO;
    manualHazardActive = false;
    isOffline          = false;

    onlineSignal      = SIG_YELLOW;
    yellowStartMillis = millis();
    setYellow(activeLane);

    activeLane = lane;
    return;
  }

  if (msg.startsWith("DISPLAY:")) {
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
// 10. AUTO ONLINE MODE
// =============================================================
void runAutoOnline(unsigned long ms) {
  switch (onlineSignal) {
    case SIG_GREEN:
      if      (activeLane == "NORTH") { setNorthGo(); updateTimers(greenCountdown, -1, -1, -1); }
      else if (activeLane == "SOUTH") { setSouthGo(); updateTimers(-1, greenCountdown, -1, -1); }
      else if (activeLane == "EAST")  { setEastGo();  updateTimers(-1, -1, greenCountdown, -1); }
      else if (activeLane == "WEST")  { setWestGo();  updateTimers(-1, -1, -1, greenCountdown); }
      break;

    case SIG_YELLOW: {
      setYellow(activeLane);
      unsigned long elapsed = ms - yellowStartMillis;
      int yRemain = max(0, YELLOW_TIME - (int)(elapsed / 1000));
      if      (activeLane == "NORTH") updateTimers(yRemain, -1, -1, -1);
      else if (activeLane == "SOUTH") updateTimers(-1, yRemain, -1, -1);
      else if (activeLane == "EAST")  updateTimers(-1, -1, yRemain, -1);
      else if (activeLane == "WEST")  updateTimers(-1, -1, -1, yRemain);

      if (elapsed >= (unsigned long)(YELLOW_TIME * 1000)) {
        onlineSignal = SIG_WAITING;
      }
      break;
    }

    case SIG_WAITING:
      setAllRed();
      updateTimers(-1, -1, -1, -1);
      break;
  }

  String title  = rainDetected ? "- AUTO (+RAIN) -" : "- AUTO (SMART AI) -";
  String sigStr;
  int    disp   = 0;

  if (onlineSignal == SIG_GREEN) {
    sigStr = "GREEN";
    disp   = greenCountdown;
  } else if (onlineSignal == SIG_YELLOW) {
    sigStr = "YELLOW";
    disp   = max(0, YELLOW_TIME - (int)((ms - yellowStartMillis) / 1000));
  } else {
    sigStr = "ALL RED";
  }

  String cntLine = (onlineSignal == SIG_WAITING)
    ? "Switching lane..."
    : "Countdown: " + String(disp) + "s";
  updateLCD(title, "ACTIVE: " + activeLane, "SIGNAL: " + sigStr, cntLine);
}

// =============================================================
// 11. AUTO FALLBACK MODE
// =============================================================
void runAutoFallback(unsigned long ms) {
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

  int currentRemaining = fallbackInYellow
    ? max(0, YELLOW_TIME - (int)((ms - fallbackYellowStart) / 1000))
    : fallbackCountdown;

  int timers[4];
  for (int i = 0; i < 4; i++) {
    if (fallbackIdx == i) {
      timers[i] = currentRemaining; 
    } else {
      int totalWait = currentRemaining;
      if (!fallbackInYellow) {
        totalWait += YELLOW_TIME; 
      }
      
      int checkIdx = (fallbackIdx + 1) % 4;
      while (checkIdx != i) {
        totalWait += FALLBACK_GREEN[checkIdx] + YELLOW_TIME;
        checkIdx = (checkIdx + 1) % 4;
      }
      timers[i] = totalWait; 
    }
  }

  updateTimers(timers[0], timers[1], timers[2], timers[3]);

  String sig  = fallbackInYellow ? "YELLOW" : "GREEN";
  updateLCD("- AUTO FALLBACK -", "NETWORK LOSS",
            "Lane: " + fbLane + " (" + sig + ")",
            "Countdown: " + String(currentRemaining) + "s");
}

// =============================================================
// 12. MANUAL OVERRIDE
// =============================================================
void handleManual(unsigned long ms) {
  if (checkButtonPress(btnEmergency)) {
    if (manualState == MAN_EMERGENCY || (manualState == MAN_TRANSITION && manualTarget == MAN_EMERGENCY)) {
      manualState = MAN_STOPPED; manualTarget = MAN_STOPPED;
      setAllRed(); updateShiftRegister();
    } else {
      prevManualState = manualState; 
      manualTarget = MAN_EMERGENCY; 
      manualTransitionStart = ms;
      manualState = MAN_TRANSITION; 
      manualHazardActive = false; 
      updateShiftRegister();
    }
    broadcastManualStates();
  }

  if (manualState == MAN_EMERGENCY) {
    setAllRed(); updateTimers(-1, -1, -1, -1);
    updateLCD("!!! OVERRIDE !!!", "EMERGENCY LOCKDOWN", "All Lanes: RED", "Press EMG to clear");
    return;
  }

  if (checkButtonPress(btnManHazard)) {
    manualHazardActive = !manualHazardActive;
    if (manualHazardActive) { manualState = MAN_STOPPED; manualTarget = MAN_STOPPED; }
    updateShiftRegister();
    broadcastManualStates();
  }
  if (manualHazardActive) {
    if ((ms / 500) % 2 == 0) blinkYellows();
    else { lightState &= 0xFFFF0000; updateShiftRegister(); }
    updateTimers(-1, -1, -1, -1);
    updateLCD("--- MANUAL MODE ---", "STATUS: HAZARD", "Flashing Yellows", "Yield All Traffic");
    return;
  }

  if (manualState != MAN_TRANSITION) {
    if       (checkButtonPress(btnGoNorth) && manualState != MAN_N_GO) { prevManualState = manualState; manualTarget = MAN_N_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoSouth) && manualState != MAN_S_GO) { prevManualState = manualState; manualTarget = MAN_S_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoEast)  && manualState != MAN_E_GO) { prevManualState = manualState; manualTarget = MAN_E_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    else if (checkButtonPress(btnGoWest)  && manualState != MAN_W_GO) { prevManualState = manualState; manualTarget = MAN_W_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; }
    if (manualState == MAN_TRANSITION) broadcastManualStates();
  }

  if (manualState == MAN_TRANSITION) {
    setTransitionLights(prevManualState); updateShiftRegister();
    long elapsed   = ms - manualTransitionStart;
    int   remaining = max(0, YELLOW_TIME - (int)(elapsed / 1000));
    if       (prevManualState == MAN_N_GO) updateTimers(remaining, -1, -1, -1);
    else if (prevManualState == MAN_S_GO) updateTimers(-1, remaining, -1, -1);
    else if (prevManualState == MAN_E_GO) updateTimers(-1, -1, remaining, -1);
    else if (prevManualState == MAN_W_GO) updateTimers(-1, -1, -1, remaining);
    else                                 updateTimers(-1, -1, -1, -1);
    
    if (manualTarget == MAN_EMERGENCY) {
      updateLCD("!!! OVERRIDE !!!", "EMERGENCY CLEAR", "Yellow: " + String(remaining) + "s", "Securing crossbox");
    } else {
      updateLCD("--- MANUAL MODE ---", "SWITCHING LANES", "Wait: " + String(remaining) + "s", "Changing lane...");
    }

    if (elapsed >= (long)(YELLOW_TIME * 1000)) { 
      manualState = manualTarget; 
      if      (manualState == MAN_N_GO) { setNorthGo(); }
      else if (manualState == MAN_S_GO) { setSouthGo(); }
      else if (manualState == MAN_E_GO) { setEastGo();  }
      else if (manualState == MAN_W_GO) { setWestGo();  }
      else                              { setAllRed();  }
      updateShiftRegister();
      broadcastManualStates();
    }
    return;
  }

  if      (manualState == MAN_N_GO) { setNorthGo(); updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", "GO: NORTH", "Override Active", "Select next lane..."); }
  else if (manualState == MAN_S_GO) { setSouthGo(); updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", "GO: SOUTH", "Override Active", "Select next lane..."); }
  else if (manualState == MAN_E_GO) { setEastGo();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", "GO: EAST",  "Override Active", "Select next lane..."); }
  else if (manualState == MAN_W_GO) { setWestGo();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", "GO: WEST",  "Override Active", "Select next lane..."); }
  else                               { setAllRed();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", "REMOTE CONTROL", "All lanes RED", "Awaiting command..."); }
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
    if       (manualState == MAN_EMERGENCY)                    bitSet(lightState, ledRed);
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

void setYellow(String lane) {
  lightState &= 0xFFFF0000;
  if       (lane == "NORTH") { bitSet(lightState, N_YELLOW); bitSet(lightState, S_RED); bitSet(lightState, E_RED); bitSet(lightState, W_RED); }
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

void setAllRed()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setNorthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_GREEN); bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setSouthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_GREEN); bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setEastGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_GREEN); bitSet(lightState,W_RED);   updateShiftRegister(); }
void setWestGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_GREEN); updateShiftRegister(); }

void setTransitionLights(ManualState prev) {
  lightState &= 0xFFFF0000;
  if       (prev == MAN_N_GO) { bitSet(lightState,N_YELLOW); bitSet(lightState,S_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_S_GO) { bitSet(lightState,S_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_E_GO) { bitSet(lightState,E_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_W_GO) { bitSet(lightState,W_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,E_RED); }
  else { setAllRed(); }
}

// =============================================================
// 14. LCD
// =============================================================
void updateLCD(String l1, String l2, String l3, String l4) {
  if (l1 != lastLine1) { lcd.setCursor(0,0); lcd.print("                    "); lcd.setCursor(0,0); lcd.print(l1); lastLine1 = l1; }
  if (l2 != lastLine2) { lcd.setCursor(0,1); lcd.print("                    "); lcd.setCursor(0,1); lcd.print(l2); lastLine2 = l2; }
  if (l3 != lastLine3) { lcd.setCursor(0,2); lcd.print("                    "); lcd.setCursor(0,2); lcd.print(l3); lastLine3 = l3; } 
  if (l4 != lastLine4) { lcd.setCursor(0,3); lcd.print("                    "); lcd.setCursor(0,3); lcd.print(l4); lastLine4 = l4; }
}

// =============================================================
// 15. 7-SEGMENT PER-LANE TIMERS
// =============================================================
void updateTimers(int n, int s, int e, int w) {
  showCentered(timerNorth, n); showCentered(timerSouth, s);
  showCentered(timerEast,  e); showCentered(timerWest,  w);
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
