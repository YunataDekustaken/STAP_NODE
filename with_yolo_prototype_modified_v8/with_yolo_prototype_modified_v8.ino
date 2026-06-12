/*
  STAP ESP32 Controller — Firmware v7 (Yellow Countdown Display)
  ===============================================================
  CHANGES FROM v6:
   1. Added DISPLAY: command handler — two commands recognized:
        DISPLAY:YELLOW,N  → starts N-second countdown on the 4-digit
                            7-segment display: shows "Y  3 → Y  2 → Y  1"
        DISPLAY:OFF       → immediately blanks all 4 digits
   2. Added dedicated Adafruit_7segment yellowDisplay at ADDR_DISPLAY (0x78)
      — separate from the 4 per-lane timer displays (0x70/0x72/0x74/0x76)
   3. Added displayYellowCountdown() and displayOff() helper functions
   4. Added yellow display state variables:
        displayActive, displayCountdown, displayLastTickMillis
   5. loop() calls tickYellowDisplay(ms) every iteration to drive the
      countdown independently from other logic — no blocking delays
   6. parsePythonCommand now recognizes "DISPLAY:" prefix

  DISPLAY BEHAVIOR:
   - During GREEN:  display is blank (DISPLAY:OFF sent by Python)
   - During YELLOW: display shows "Y  3" → "Y  2" → "Y  1" then blanks
   - The 4th digit (rightmost) shows the countdown number
   - The 1st digit (leftmost) shows "Y" using 7-seg segments
   - Middle two digits are blank for clarity
   - Display self-blanks when countdown reaches 0

  WHY YELLOW-ONLY:
   - Yellow is always a fixed 3s constant — the displayed number is
     always 100% accurate, never conflicts with adaptive green timing
   - Drivers get actionable lane-switching info at exactly the right moment

  EVERYTHING ELSE IS IDENTICAL TO v6:
   - All fixes from v6 (yellow flicker, PHASE: guard, one-cmd-per-loop)
   - Manual mode, fallback, shift register, LCD, per-lane timers, buttons
     are completely unchanged
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

// Per-lane timer display I2C addresses (unchanged from v6)
#define ADDR_NORTH   0x70
#define ADDR_SOUTH   0x72
#define ADDR_EAST    0x74
#define ADDR_WEST    0x76

// Yellow countdown display — the single 4-digit 7-segment
// Set this to the actual I2C address of your display.
// Default Adafruit HT16K33 backpack address is 0x70; since
// your lane displays use 0x70–0x76, use 0x78 (A0+A1 bridged).
#define ADDR_DISPLAY 0x78

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

// Per-lane countdown displays (used for operator-side timing info)
Adafruit_7segment timerNorth = Adafruit_7segment();
Adafruit_7segment timerSouth = Adafruit_7segment();
Adafruit_7segment timerEast  = Adafruit_7segment();
Adafruit_7segment timerWest  = Adafruit_7segment();

// Road-facing yellow countdown display (4-digit, shows Y  3 → Y  2 → Y  1)
Adafruit_7segment yellowDisplay = Adafruit_7segment();

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

// ── LCD dedup ─────────────────────────────────────────────────
String lastLine1 = "", lastLine2 = "", lastLine3 = "", lastLine4 = "";

// ── Yellow display state ──────────────────────────────────────
// Tracks the road-facing 7-segment display independently.
// displayActive:           true while a DISPLAY:YELLOW,N countdown is running
// displayCountdown:        seconds remaining to show (counts down to 0 then blanks)
// displayLastTickMillis:   timestamp of last 1-second decrement
bool          displayActive          = false;
int           displayCountdown       = 0;
unsigned long displayLastTickMillis  = 0;

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
void displayYellowCountdown(int seconds);
void displayOff();
void tickYellowDisplay(unsigned long ms);

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

  // Initialize yellow countdown display
  yellowDisplay.begin(ADDR_DISPLAY);
  yellowDisplay.setBrightness(10);
  displayOff(); // Start blank

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
// One command per loop iteration (v6 fix #2 preserved).
// tickYellowDisplay() is called every iteration for smooth countdown.
// =============================================================
void loop() {
  unsigned long ms = millis();
  rainDetected = (analogRead(rainSensorPin) < RAIN_THRESHOLD);

  if (ms - lastTelemetryTime >= 400) {
    lastTelemetryTime = ms;
    Serial.println(
      "RAIN:" + String(rainDetected ? "1" : "0") +
      ",MODE:" + String(currentMode == MANUAL ? "MANUAL" : "AUTO")
    );
    Serial.flush();
  }

  // One command per loop iteration (v6 fix #2 preserved)
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
    currentMode  = MANUAL;
    manualState  = MAN_STOPPED;
    manualTarget = MAN_STOPPED;
    setAllRed();
    displayOff(); // Blank display when switching to manual
  }

  if (currentMode == AUTO) {
    bool nowSilent = (ms - lastCommMillis > 1000);
    if (!nowSilent) {
      silenceStartMillis = 0;
      isOffline          = false;
    } else {
      if (silenceStartMillis == 0) silenceStartMillis = ms;
      if (!isOffline && (ms - silenceStartMillis >= DATA_TIMEOUT)) {
        isOffline            = true;
        fallbackIdx          = 0;
        fallbackCountdown    = FALLBACK_GREEN[0];
        fallbackInYellow     = false;
        fallbackYellowStart  = 0;
        onlineSignal         = SIG_WAITING;
        displayOff(); // Blank display on fallback entry
      }
    }
  }

  // 1-second tick for green/fallback countdowns
  if (ms - lastTickMillis >= 1000) {
    lastTickMillis = ms;
    if (currentMode == AUTO) {
      if (!isOffline && onlineSignal == SIG_GREEN && greenCountdown > 0)
        greenCountdown--;
      if (isOffline && !fallbackInYellow && fallbackCountdown > 0)
        fallbackCountdown--;
    }
  }

  // Tick the yellow display countdown every loop (independent timer)
  tickYellowDisplay(ms);

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
// Added DISPLAY: handling alongside existing PHASE:/YELLOW:/PING:
// =============================================================
void parsePythonCommand(String msg) {
  // Refresh comms timer on any valid message
  if (msg.startsWith("PHASE:")   ||
      msg.startsWith("YELLOW:")  ||
      msg.startsWith("PING:")    ||
      msg.startsWith("DISPLAY:")) {
    lastCommMillis     = millis();
    silenceStartMillis = 0;
    isOffline          = false;
  } else {
    return;
  }

  // ── PING — keepalive only, no state change ─────────────────
  if (msg.startsWith("PING:")) {
    return;
  }

  // ── DISPLAY — road-facing 7-segment control ────────────────
  if (msg.startsWith("DISPLAY:")) {
    String payload = msg.substring(8); // Everything after "DISPLAY:"
    payload.trim();

    if (payload == "OFF") {
      // Blank the display immediately
      displayOff();
    }
    else if (payload.startsWith("YELLOW,")) {
      // DISPLAY:YELLOW,N — start countdown from N seconds
      int seconds = payload.substring(7).toInt();
      if (seconds > 0) {
        displayYellowCountdown(seconds);
      }
    }
    return;
  }

  // ── PHASE — start a new green phase ───────────────────────
  if (msg.startsWith("PHASE:")) {
    // v6 fix #1: ignore if local yellow is still running
    if (onlineSignal == SIG_YELLOW) {
      unsigned long elapsed = millis() - yellowStartMillis;
      if (elapsed < (unsigned long)(YELLOW_TIME * 1000)) {
        return;
      }
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

    activeLane     = lane;
    greenCountdown = duration;
    onlineSignal   = SIG_GREEN;
    return;
  }

  // ── YELLOW — start yellow phase ───────────────────────────
  if (msg.startsWith("YELLOW:")) {
    String lane = msg.substring(7);
    lane.trim();
    activeLane        = lane;
    onlineSignal      = SIG_YELLOW;
    yellowStartMillis = millis();
    // Note: the display countdown is started separately by DISPLAY:YELLOW,N
    // which Python sends immediately after YELLOW:lane
    return;
  }
}

// =============================================================
// 10. YELLOW DISPLAY HELPERS
// =============================================================

/*
  displayYellowCountdown(seconds)
  --------------------------------
  Starts the countdown on the road-facing 4-digit 7-segment display.
  Shows: [Y][blank][blank][N] where N is the countdown digit.

  7-segment encoding for 'Y':
    Segments b, c, f, g = bits 1, 2, 5, 6 = 0b01100110 = 0x66
  The Adafruit library's writeDigitRaw() accepts a raw segment bitmask.
  Adafruit HT16K33 segment mapping: bit0=a, bit1=b, bit2=c, bit3=d,
                                     bit4=e, bit5=f, bit6=g, bit7=dp
  Y = segments b+c+f+g = 0b01100110 = 0x66
*/
void displayYellowCountdown(int seconds) {
  displayActive         = true;
  displayCountdown      = seconds;
  displayLastTickMillis = millis();

  // Render immediately so driver sees it without waiting 1s
  yellowDisplay.clear();
  yellowDisplay.writeDigitRaw(0, 0x66);  // 'Y' on digit 0 (leftmost)
  yellowDisplay.writeDigitRaw(1, 0x00);  // blank
  yellowDisplay.writeDigitRaw(3, 0x00);  // blank (digit 2 is colon pos, skip)
  yellowDisplay.writeDigitNum(4, seconds % 10); // countdown digit (rightmost)
  yellowDisplay.drawColon(false);
  yellowDisplay.writeDisplay();
}

/*
  displayOff()
  ------------
  Blanks all 4 digits immediately.
  Called on: DISPLAY:OFF, GREEN start, manual mode switch, fallback entry.
*/
void displayOff() {
  displayActive    = false;
  displayCountdown = 0;
  yellowDisplay.clear();
  yellowDisplay.writeDisplay();
}

/*
  tickYellowDisplay(ms)
  ----------------------
  Called every loop iteration. Decrements displayCountdown every second
  and updates the display. Self-blanks when countdown reaches 0.
  Uses its own independent timer (displayLastTickMillis) so it never
  interferes with greenCountdown or fallbackCountdown ticks.
*/
void tickYellowDisplay(unsigned long ms) {
  if (!displayActive) return;

  // Decrement every 1 second
  if (ms - displayLastTickMillis >= 1000) {
    displayLastTickMillis = ms;
    displayCountdown--;

    if (displayCountdown <= 0) {
      // Countdown finished — blank the display
      displayOff();
    } else {
      // Update digit 4 (rightmost) with new countdown value
      yellowDisplay.clear();
      yellowDisplay.writeDigitRaw(0, 0x66);          // 'Y'
      yellowDisplay.writeDigitRaw(1, 0x00);          // blank
      yellowDisplay.writeDigitRaw(3, 0x00);          // blank
      yellowDisplay.writeDigitNum(4, displayCountdown % 10); // countdown
      yellowDisplay.drawColon(false);
      yellowDisplay.writeDisplay();
    }
  }
}

// =============================================================
// 11. AUTO ONLINE MODE (v6 fix #3 preserved)
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
      // v6 fix #3: render yellow FIRST, check timer expiry AFTER
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

  // LCD
  String title  = rainDetected ? "-- AUTO (+RAIN) --" : "-- AUTO (SMART AI) --";
  String sigStr;
  int    disp   = 0;

  if (onlineSignal == SIG_GREEN) {
    sigStr = "GREEN";
    disp   = greenCountdown;
  } else if (onlineSignal == SIG_YELLOW) {
    sigStr = "YELLOW";
    disp   = max(0, YELLOW_TIME - (int)((ms - yellowStartMillis) / 1000));
  } else {
    sigStr = "--- ALL RED ---";
  }

  String cntLine = (onlineSignal == SIG_WAITING)
    ? "Switching lane..."
    : "Countdown: " + String(disp) + "s";
  updateLCD(title, "ACTIVE: " + activeLane, "SIGNAL: " + sigStr, cntLine);
}

// =============================================================
// 12. AUTO FALLBACK MODE
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
    // Show yellow countdown on display during fallback yellow phase too
    displayYellowCountdown(YELLOW_TIME);
  }

  String fbLane = FALLBACK_LANE[fallbackIdx];
  if (fallbackInYellow) {
    int yRemain = max(0, YELLOW_TIME - (int)((ms - fallbackYellowStart) / 1000));
    setYellow(fbLane);
    if      (fbLane == "NORTH") updateTimers(yRemain, -1, -1, -1);
    else if (fbLane == "SOUTH") updateTimers(-1, yRemain, -1, -1);
    else if (fbLane == "EAST")  updateTimers(-1, -1, yRemain, -1);
    else if (fbLane == "WEST")  updateTimers(-1, -1, -1, yRemain);
  } else {
    if      (fbLane == "NORTH") { setNorthGo(); updateTimers(fallbackCountdown, -1, -1, -1); }
    else if (fbLane == "SOUTH") { setSouthGo(); updateTimers(-1, fallbackCountdown, -1, -1); }
    else if (fbLane == "EAST")  { setEastGo();  updateTimers(-1, -1, fallbackCountdown, -1); }
    else if (fbLane == "WEST")  { setWestGo();  updateTimers(-1, -1, -1, fallbackCountdown); }
  }

  String sig  = fallbackInYellow ? "YELLOW" : "GREEN";
  int    disp = fallbackInYellow
    ? max(0, YELLOW_TIME - (int)((ms - fallbackYellowStart) / 1000))
    : fallbackCountdown;
  updateLCD("-- AUTO FALLBACK --", "NETWORK LOSS",
            "Active: " + fbLane + " [" + sig + "]",
            "Countdown: " + String(disp) + "s");
}

// =============================================================
// 13. MANUAL OVERRIDE
// =============================================================
void handleManual(unsigned long ms) {
  if (checkButtonPress(btnEmergency)) {
    if (manualState == MAN_EMERGENCY) {
      manualState = MAN_STOPPED; manualTarget = MAN_STOPPED;
      setAllRed(); updateShiftRegister();
    } else {
      manualState = MAN_EMERGENCY; manualTarget = MAN_STOPPED;
      manualHazardActive = false; updateShiftRegister();
    }
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
  }
  if (manualHazardActive) {
    if ((ms / 500) % 2 == 0) blinkYellows();
    else { lightState &= 0xFFFF0000; updateShiftRegister(); }
    updateTimers(-1, -1, -1, -1);
    updateLCD("--- MANUAL MODE ---", ">> STATUS: HAZARD", "Flashing Yellows", "Yield all traffic");
    return;
  }

  if (manualState != MAN_TRANSITION) {
    if      (checkButtonPress(btnGoNorth) && manualState != MAN_N_GO) { prevManualState = manualState; manualTarget = MAN_N_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; displayYellowCountdown(YELLOW_TIME); }
    else if (checkButtonPress(btnGoSouth) && manualState != MAN_S_GO) { prevManualState = manualState; manualTarget = MAN_S_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; displayYellowCountdown(YELLOW_TIME); }
    else if (checkButtonPress(btnGoEast)  && manualState != MAN_E_GO) { prevManualState = manualState; manualTarget = MAN_E_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; displayYellowCountdown(YELLOW_TIME); }
    else if (checkButtonPress(btnGoWest)  && manualState != MAN_W_GO) { prevManualState = manualState; manualTarget = MAN_W_GO; manualState = MAN_TRANSITION; manualTransitionStart = ms; displayYellowCountdown(YELLOW_TIME); }
  }

  if (manualState == MAN_TRANSITION) {
    setTransitionLights(prevManualState); updateShiftRegister();
    long elapsed   = ms - manualTransitionStart;
    int  remaining = max(0, YELLOW_TIME - (int)(elapsed / 1000));
    if      (prevManualState == MAN_N_GO) updateTimers(remaining, -1, -1, -1);
    else if (prevManualState == MAN_S_GO) updateTimers(-1, remaining, -1, -1);
    else if (prevManualState == MAN_E_GO) updateTimers(-1, -1, remaining, -1);
    else if (prevManualState == MAN_W_GO) updateTimers(-1, -1, -1, remaining);
    else                                  updateTimers(-1, -1, -1, -1);
    updateLCD("--- MANUAL MODE ---", ">> SWITCHING LANES", "Wait: " + String(remaining) + "s", "Changing active lane");
    if (elapsed >= (long)(YELLOW_TIME * 1000)) { manualState = manualTarget; displayOff(); }
    return;
  }

  if      (manualState == MAN_N_GO) { setNorthGo(); updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", ">> GO: NORTH", "Manual override act.", "Select next lane ->"); }
  else if (manualState == MAN_S_GO) { setSouthGo(); updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", ">> GO: SOUTH", "Manual override act.", "Select next lane ->"); }
  else if (manualState == MAN_E_GO) { setEastGo();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", ">> GO: EAST",  "Manual override act.", "Select next lane ->"); }
  else if (manualState == MAN_W_GO) { setWestGo();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", ">> GO: WEST",  "Manual override act.", "Select next lane ->"); }
  else                              { setAllRed();  updateTimers(-1,-1,-1,-1); updateLCD("--- MANUAL MODE ---", ">> IDLE", "All lanes RED", "Select a lane to GO"); }
}

// =============================================================
// 14. SHIFT REGISTER & LIGHT PRESETS (unchanged from v6)
// =============================================================
void syncIndicatorLEDs() {
  lightState &= 0x0000FFFF;
  if (currentMode == AUTO) {
    bitSet(lightState, ledBlue);
  } else {
    bitSet(lightState, ledWhite);
    if      (manualState == MAN_EMERGENCY)                     bitSet(lightState, ledRed);
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

void setAllRed()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setNorthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_GREEN); bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setSouthGo() { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_GREEN); bitSet(lightState,E_RED);   bitSet(lightState,W_RED);   updateShiftRegister(); }
void setEastGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_GREEN); bitSet(lightState,W_RED);   updateShiftRegister(); }
void setWestGo()  { lightState &= 0xFFFF0000; bitSet(lightState,N_RED);   bitSet(lightState,S_RED);   bitSet(lightState,E_RED);   bitSet(lightState,W_GREEN); updateShiftRegister(); }

void setTransitionLights(ManualState prev) {
  lightState &= 0xFFFF0000;
  if      (prev == MAN_N_GO) { bitSet(lightState,N_YELLOW); bitSet(lightState,S_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_S_GO) { bitSet(lightState,S_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,E_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_E_GO) { bitSet(lightState,E_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,W_RED); }
  else if (prev == MAN_W_GO) { bitSet(lightState,W_YELLOW); bitSet(lightState,N_RED); bitSet(lightState,S_RED); bitSet(lightState,E_RED); }
  else { setAllRed(); }
}

// =============================================================
// 15. LCD (unchanged from v6)
// =============================================================
void updateLCD(String l1, String l2, String l3, String l4) {
  if (l1 != lastLine1) { lcd.setCursor(0,0); lcd.print("                    "); lcd.setCursor(0,0); lcd.print(l1); lastLine1 = l1; }
  if (l2 != lastLine2) { lcd.setCursor(0,1); lcd.print("                    "); lcd.setCursor(0,1); lcd.print(l2); lastLine2 = l2; }
  if (l3 != lastLine3) { lcd.setCursor(0,2); lcd.print("                    "); lcd.setCursor(0,2); lcd.print(l3); lastLine3 = l3; }
  if (l4 != lastLine4) { lcd.setCursor(0,3); lcd.print("                    "); lcd.setCursor(0,3); lcd.print(l4); lastLine4 = l4; }
}

// =============================================================
// 16. 7-SEGMENT PER-LANE TIMERS (unchanged from v6)
// =============================================================
void updateTimers(int n, int s, int e, int w) {
  showCentered(timerNorth, n); showCentered(timerSouth, s);
  showCentered(timerEast,  e); showCentered(timerWest,  w);
}

void showCentered(Adafruit_7segment &disp, int number) {
  if (number < 0) { disp.clear(); disp.writeDisplay(); return; }
  disp.drawColon(false);
  disp.writeDigitRaw(0, 0x00);
  disp.writeDigitNum(1, (number / 10) % 10);
  disp.writeDigitNum(3, number % 10);
  disp.writeDigitRaw(4, 0x00);
  disp.writeDisplay();
}

// =============================================================
// 17. DEBOUNCED BUTTON (unchanged from v6)
// =============================================================
bool checkButtonPress(int pin) {
  static bool          init      = false;
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
