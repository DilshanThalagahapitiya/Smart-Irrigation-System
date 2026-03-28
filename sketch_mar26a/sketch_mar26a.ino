#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

/**
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         IRRIGATION CONTROLLER — FIRMWARE v1.1               ║
 * ║         Developed by: Thalagahapitiya                       ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  v1.1 CHANGE: Zero-sensor motor shutdown                    ║
 * ║  If ALL sensors go offline, motor is immediately forced     ║
 * ║  OFF and LCD shows "NO SENSOR-MTR OFF" to alert operator.   ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * WIRING GUIDE:
 * ------------------------------------------------------------
 * I2C 16x2 LCD:
 *   - VCC  -> 5V
 *   - GND  -> GND
 *   - SDA  -> Pin A4 (on Uno)
 *   - SCL  -> Pin A5 (on Uno)
 *
 * SENSORS (DHT22):
 *   - Sensor 1 Data -> Pin 2
 *   - Sensor 2 Data -> Pin 3
 *   - Sensor 3 Data -> Pin 4
 *   - Sensor 4 Data -> Pin 5
 *
 * OUTPUTS:
 *   - Relay (Motor) -> Pin 6
 *
 * INPUTS (Buttons):
 *   - Menu Button   -> Pin 7
 *   - Ok Button     -> Pin 8
 *   - Up Button     -> Pin 9
 *   - Down Button   -> Pin 10
 *   - Manual Button -> Pin 11
 * ------------------------------------------------------------
 *
 * MANUAL BUTTON (Pin 11) BEHAVIOUR:
 * Long press (>1.5s) -> Toggle AUTO <-> MANUAL mode
 * In AUTO mode  : short press ignored
 * In MANUAL mode: short press toggles motor ON/OFF
 *                 long press returns to AUTO
 */

// ===============================================================
//  PIN ASSIGNMENTS
// ===============================================================
const uint8_t DHT_PINS[] = {2, 3, 4, 5};
const uint8_t BTN_MENU = 7;
const uint8_t BTN_OK = 8;
const uint8_t BTN_UP = 9;
const uint8_t BTN_DOWN = 10;
const uint8_t BTN_MANUAL = 11;
const uint8_t RELAY_PIN = 6;

#define RELAY_ON HIGH
#define RELAY_OFF LOW
#define BUTTON_PRESSED LOW

// ===============================================================
//  HARDWARE CONFIG
// ===============================================================
#define DHT_TYPE DHT22
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define NUM_SENSORS 4

// ===============================================================
//  TIMING
// ===============================================================
const uint32_t DISPLAY_CYCLE_TIME = 3000;
const uint32_t DEBOUNCE_TIME = 50;
const uint32_t SENSOR_READ_GAP = 550;
const uint32_t SENSOR_FAIL_WINDOW = 5000;
const uint32_t ERROR_SHOW_TIME = 3000;
const uint32_t LONG_PRESS_TIME = 1500;

// ===============================================================
//  ANIMATION STATE
// ===============================================================
const char SPIN_FRAMES[] = {'|', '/', '-', '\\'};
uint8_t spinIdx = 0;
uint32_t lastSpinTime = 0;
const uint32_t SPIN_RATE = 100;

uint8_t blinkIdx = 0;
uint32_t lastBlinkTime = 0;
const uint32_t BLINK_RATE = 500;

uint8_t pulseIdx = 0;
uint32_t lastPulseTime = 0;
const uint32_t PULSE_RATE = 200;

void updateAnimations() {
  uint32_t now = millis();
  if (now - lastSpinTime >= SPIN_RATE) {
    spinIdx = (spinIdx + 1) % 4;
    lastSpinTime = now;
  }
  if (now - lastBlinkTime >= BLINK_RATE) {
    blinkIdx = (blinkIdx + 1) % 2;
    lastBlinkTime = now;
  }
  if (now - lastPulseTime >= PULSE_RATE) {
    pulseIdx = (pulseIdx + 1) % 3;
    lastPulseTime = now;
  }
}

// ===============================================================
//  MANUAL BUTTON
// ===============================================================
struct ManualButton {
  bool lastReading;
  bool currentState;
  uint32_t lastDebounce;
  uint32_t pressStartTime;
  bool isHeld;
  bool longPressHandled;
  bool shortPressEvent;
  bool longPressEvent;
} manBtn = {HIGH, HIGH, 0, 0, false, false, false, false};

// ===============================================================
//  MODE STATE
// ===============================================================
bool isManualMode = false;
bool manualMotorOn = false;

// ===============================================================
//  SENSOR HEALTH TRACKING
// ===============================================================
struct SensorState {
  float lastTemp;
  float lastHumidity;
  bool online;
  bool everConnected;
  uint32_t lastValidTime;
  uint8_t failStreak;
};
SensorState sensors[NUM_SENSORS];

int currentSensorIdx = 0;
uint32_t lastSensorReadTime = 0;

float medianTemp = 0.0;
float medianHumidity = 0.0;
int onlineCount = 0;

bool showingSensorError = false;
uint32_t errorShowStart = 0;
int errorSensorIdx = -1;

// ===============================================================
//  SYSTEM STATE MACHINE
// ===============================================================
enum SystemState {
  STATE_MAIN,
  STATE_SENSOR_ERROR,
  STATE_MENU_LIST,
  STATE_SET_TEMP_START,
  STATE_SET_TEMP_STOP,
  STATE_SET_TIME,
  STATE_SET_BACKLIGHT
};
enum MainDisplayCycle { CYCLE_TEMP, CYCLE_HUMIDITY };

// Forward declarations
void changeState(SystemState s);
void triggerSensorError(int idx);

SystemState currentState = STATE_MAIN;
SystemState stateBeforeError = STATE_MAIN;
MainDisplayCycle currentCycle = CYCLE_TEMP;

// ===============================================================
//  MOTOR / THRESHOLD SETTINGS
// ===============================================================
float startTemp = 32.0;
float stopTemp = 28.0;
uint32_t minRunTimeMin = 0;

bool motorActive = false;
uint32_t motorStartTime = 0;

// ===============================================================
//  BACKLIGHT
// ===============================================================
uint32_t backlightTimeoutSec = 30;
bool backlightOn = true;
uint32_t lastActivityTime = 0;

// ===============================================================
//  GLOBAL OBJECTS
// ===============================================================
DHT dhts[NUM_SENSORS] = {DHT(DHT_PINS[0], DHT_TYPE), DHT(DHT_PINS[1], DHT_TYPE),
                         DHT(DHT_PINS[2], DHT_TYPE),
                         DHT(DHT_PINS[3], DHT_TYPE)};
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
bool forceRefresh = true;

// ===============================================================
//  GENERIC BUTTON DEBOUNCE (menu buttons)
// ===============================================================
struct Button {
  uint8_t pin;
  bool lastReading;
  bool state;
  bool pressed;
  uint32_t lastDebounce;
};
Button buttons[] = {{BTN_MENU, HIGH, HIGH, false, 0},
                    {BTN_OK, HIGH, HIGH, false, 0},
                    {BTN_UP, HIGH, HIGH, false, 0},
                    {BTN_DOWN, HIGH, HIGH, false, 0}};

// ===============================================================
//  BACKLIGHT HELPERS
// ===============================================================
void wakeBacklight() {
  lastActivityTime = millis();
  if (!backlightOn) {
    lcd.backlight();
    backlightOn = true;
    forceRefresh = true;
  }
}

void handleBacklight() {
  if (backlightTimeoutSec == 0) {
    if (!backlightOn) {
      lcd.backlight();
      backlightOn = true;
    }
    return;
  }
  if (backlightOn &&
      millis() - lastActivityTime >= backlightTimeoutSec * 1000UL) {
    lcd.noBacklight();
    backlightOn = false;
  }
}

// ===============================================================
//  GENERIC BUTTON UPDATE
// ===============================================================
void updateButtons() {
  for (int i = 0; i < 4; i++) {
    bool reading = digitalRead(buttons[i].pin);
    if (reading != buttons[i].lastReading)
      buttons[i].lastDebounce = millis();
    if (millis() - buttons[i].lastDebounce > DEBOUNCE_TIME) {
      if (reading != buttons[i].state) {
        if (reading == BUTTON_PRESSED) {
          wakeBacklight();
          buttons[i].pressed = true;
        }
        buttons[i].state = reading;
      }
    }
    buttons[i].lastReading = reading;
  }
}

bool isPressed(uint8_t pin) {
  for (int i = 0; i < 4; i++) {
    if (buttons[i].pin == pin && buttons[i].pressed) {
      buttons[i].pressed = false;
      return true;
    }
  }
  return false;
}

// ===============================================================
//  MANUAL BUTTON UPDATE
// ===============================================================
void updateManualButton() {
  bool reading = digitalRead(BTN_MANUAL);
  if (reading != manBtn.lastReading)
    manBtn.lastDebounce = millis();
  manBtn.lastReading = reading;
  if (millis() - manBtn.lastDebounce <= DEBOUNCE_TIME)
    return;

  bool justPressed =
      (reading == BUTTON_PRESSED && manBtn.currentState != BUTTON_PRESSED);
  bool justReleased =
      (reading != BUTTON_PRESSED && manBtn.currentState == BUTTON_PRESSED);

  if (justPressed) {
    manBtn.pressStartTime = millis();
    manBtn.isHeld = true;
    manBtn.longPressHandled = false;
    wakeBacklight();
  }
  if (manBtn.isHeld && !manBtn.longPressHandled &&
      millis() - manBtn.pressStartTime >= LONG_PRESS_TIME) {
    manBtn.longPressHandled = true;
    manBtn.longPressEvent = true;
  }
  if (justReleased) {
    if (!manBtn.longPressHandled)
      manBtn.shortPressEvent = true;
    manBtn.isHeld = false;
  }
  manBtn.currentState = reading;
}

bool manualShortPressed() {
  if (manBtn.shortPressEvent) {
    manBtn.shortPressEvent = false;
    return true;
  }
  return false;
}
bool manualLongPressed() {
  if (manBtn.longPressEvent) {
    manBtn.longPressEvent = false;
    return true;
  }
  return false;
}

// ===============================================================
//  SENSOR AVERAGING
// ===============================================================
void recomputeAverages() {
  float sumTemp = 0, sumHum = 0;
  onlineCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i].online) {
      sumTemp += sensors[i].lastTemp;
      sumHum += sensors[i].lastHumidity;
      onlineCount++;
    }
  }
  if (onlineCount > 0) {
    medianTemp = sumTemp / onlineCount;
    medianHumidity = sumHum / onlineCount;
  }
}

void triggerSensorError(int idx) {
  if (currentState == STATE_MAIN || currentState == STATE_SENSOR_ERROR) {
    stateBeforeError = STATE_MAIN;
    currentState = STATE_SENSOR_ERROR;
    errorSensorIdx = idx;
    errorShowStart = millis();
    showingSensorError = true;
    lcd.clear();
    forceRefresh = true;
    Serial.print("Sensor ERROR: S");
    Serial.println(idx + 1);
  }
}

// ===============================================================
//  SENSOR UPDATE (main loop)
// ===============================================================
void updateSensors() {
  if (millis() - lastSensorReadTime < SENSOR_READ_GAP)
    return;
  lastSensorReadTime = millis();

  int idx = currentSensorIdx;
  float t = dhts[idx].readTemperature();
  float h = dhts[idx].readHumidity();
  bool ok = (!isnan(t) && !isnan(h));

  if (ok) {
    bool wasOffline = sensors[idx].everConnected && !sensors[idx].online;
    bool firstEverOnline = !sensors[idx].everConnected;
    sensors[idx].lastTemp = t;
    sensors[idx].lastHumidity = h;
    sensors[idx].lastValidTime = millis();
    sensors[idx].failStreak = 0;
    sensors[idx].online = true;
    sensors[idx].everConnected = true;
    if (firstEverOnline) {
      Serial.print("Sensor ONLINE (late): S");
      Serial.println(idx + 1);
      forceRefresh = true;
    } else if (wasOffline) {
      Serial.print("Sensor RECOVERED: S");
      Serial.println(idx + 1);
      forceRefresh = true;
    }
  } else {
    sensors[idx].failStreak++;
    if (sensors[idx].online && (millis() - sensors[idx].lastValidTime >= SENSOR_FAIL_WINDOW)) {
      sensors[idx].online = false;
      triggerSensorError(idx);
    }
  }

  recomputeAverages();
  currentSensorIdx = (currentSensorIdx + 1) % NUM_SENSORS;
}

// ===============================================================
//  MOTOR LOGIC
// ===============================================================
void handleMotor() {
  // ── Handle mode toggle (long press on manual button) ──────────
  if (manualLongPressed()) {
    isManualMode = !isManualMode;
    if (isManualMode) {
      manualMotorOn = motorActive;
      Serial.println(">>> Entered MANUAL mode");
    } else {
      Serial.println(">>> Returned to AUTO mode");
    }
    forceRefresh = true;
  }

  // ── Handle short press (manual mode only) ─────────────────────
  if (manualShortPressed()) {
    if (isManualMode) {
      manualMotorOn = !manualMotorOn;
      motorActive = manualMotorOn;
      if (motorActive)
        motorStartTime = millis();
      Serial.print("Manual motor: ");
      Serial.println(motorActive ? "ON" : "OFF");
      forceRefresh = true;
    } else {
      Serial.println("Short press ignored (AUTO mode)");
    }
  }

  // ── MANUAL mode: user has direct control ──────────────────────
  if (isManualMode) {
    motorActive = manualMotorOn;

  // ── AUTO mode ─────────────────────────────────────────────────
  } else {

    // ── v1.1: Zero-sensor safety shutdown ────────────────────────
    // If every sensor has gone offline, immediately cut the motor.
    // Do NOT wait for threshold logic — there is no valid reading
    // to make a decision from, so the safe state is always OFF.
    if (onlineCount == 0) {
      if (motorActive) {
        motorActive = false;
        digitalWrite(RELAY_PIN, RELAY_OFF);
        Serial.println("Motor: FORCED OFF — no sensors online");
        forceRefresh = true;
      }
      return; // Skip threshold logic entirely — nothing to compare
    }
    // ─────────────────────────────────────────────────────────────

    bool shouldBeOn = motorActive;
    if (medianTemp >= startTemp)
      shouldBeOn = true;
    else if (medianTemp <= stopTemp)
      shouldBeOn = false;

    if (shouldBeOn && !motorActive) {
      motorActive = true;
      motorStartTime = millis();
      Serial.println("Motor: AUTO ON");
      forceRefresh = true;
    } else if (!shouldBeOn && motorActive) {
      if (millis() - motorStartTime >= minRunTimeMin * 60000UL) {
        motorActive = false;
        Serial.println("Motor: AUTO OFF");
        forceRefresh = true;
      }
    }
  }

  digitalWrite(RELAY_PIN, motorActive ? RELAY_ON : RELAY_OFF);
}

// ===============================================================
//  MENU
// ===============================================================
int menuIdx = 0;
const char *menuItems[] = {"1. Start Temp", "2. Stop Temp ", "3. Motor Time",
                           "4. Backlight "};
const int MENU_COUNT = 4;

void changeState(SystemState s) {
  currentState = s;
  forceRefresh = true;
  lcd.clear();
}

// ===============================================================
//  UI UPDATE
// ===============================================================
void updateUI() {
  static SystemState prevState = (SystemState)-1;
  static MainDisplayCycle prevCycle = (MainDisplayCycle)-1;
  static float prevTemp = -999;
  static float prevStart = -999;
  static float prevStop = -999;
  static uint32_t prevMin = 9999;
  static bool prevMotor = false;
  static int prevOnline = -1;
  static uint32_t prevBL = 9999;
  static bool prevManualMode = false;
  static uint8_t prevSpinIdx = 255;
  static uint8_t prevBlinkIdx = 255;
  static uint8_t prevPulseIdx = 255;

  // Input handling
  if (currentState == STATE_SENSOR_ERROR) {
    if (isPressed(BTN_OK) || isPressed(BTN_MENU) || isPressed(BTN_UP) ||
        isPressed(BTN_DOWN))
      changeState(stateBeforeError);
    if (millis() - errorShowStart >= ERROR_SHOW_TIME)
      changeState(stateBeforeError);

  } else if (currentState == STATE_MAIN) {
    if (isPressed(BTN_MENU))
      changeState(STATE_MENU_LIST);

  } else if (currentState == STATE_MENU_LIST) {
    if (isPressed(BTN_UP)) {
      menuIdx = (menuIdx - 1 + MENU_COUNT) % MENU_COUNT;
      forceRefresh = true;
    }
    if (isPressed(BTN_DOWN)) {
      menuIdx = (menuIdx + 1) % MENU_COUNT;
      forceRefresh = true;
    }
    if (isPressed(BTN_OK)) {
      if (menuIdx == 0)
        changeState(STATE_SET_TEMP_START);
      else if (menuIdx == 1)
        changeState(STATE_SET_TEMP_STOP);
      else if (menuIdx == 2)
        changeState(STATE_SET_TIME);
      else
        changeState(STATE_SET_BACKLIGHT);
    }
    if (isPressed(BTN_MENU))
      changeState(STATE_MAIN);

  } else if (currentState == STATE_SET_TEMP_START) {
    if (isPressed(BTN_UP)) {
      startTemp += 1.0;
      forceRefresh = true;
    }
    if (isPressed(BTN_DOWN)) {
      startTemp -= 1.0;
      if (startTemp <= stopTemp)
        startTemp = stopTemp + 1.0;
      forceRefresh = true;
    }
    if (isPressed(BTN_OK) || isPressed(BTN_MENU))
      changeState(STATE_MENU_LIST);

  } else if (currentState == STATE_SET_TEMP_STOP) {
    if (isPressed(BTN_UP)) {
      stopTemp += 1.0;
      if (stopTemp >= startTemp)
        stopTemp = startTemp - 1.0;
      forceRefresh = true;
    }
    if (isPressed(BTN_DOWN)) {
      stopTemp -= 1.0;
      forceRefresh = true;
    }
    if (isPressed(BTN_OK) || isPressed(BTN_MENU))
      changeState(STATE_MENU_LIST);

  } else if (currentState == STATE_SET_TIME) {
    if (isPressed(BTN_UP)) {
      minRunTimeMin += 5;
      forceRefresh = true;
    }
    if (isPressed(BTN_DOWN)) {
      if (minRunTimeMin >= 5)
        minRunTimeMin -= 5;
      forceRefresh = true;
    }
    if (isPressed(BTN_OK) || isPressed(BTN_MENU))
      changeState(STATE_MENU_LIST);

  } else if (currentState == STATE_SET_BACKLIGHT) {
    if (isPressed(BTN_UP)) {
      if (backlightTimeoutSec == 0)
        backlightTimeoutSec = 10;
      else if (backlightTimeoutSec == 10)
        backlightTimeoutSec = 20;
      else if (backlightTimeoutSec == 20)
        backlightTimeoutSec = 30;
      else if (backlightTimeoutSec == 30)
        backlightTimeoutSec = 60;
      else if (backlightTimeoutSec == 60)
        backlightTimeoutSec = 120;
      else if (backlightTimeoutSec == 120)
        backlightTimeoutSec = 300;
      else
        backlightTimeoutSec = 0;
      wakeBacklight();
      forceRefresh = true;
    }
    if (isPressed(BTN_DOWN)) {
      if (backlightTimeoutSec == 0)
        backlightTimeoutSec = 300;
      else if (backlightTimeoutSec == 300)
        backlightTimeoutSec = 120;
      else if (backlightTimeoutSec == 120)
        backlightTimeoutSec = 60;
      else if (backlightTimeoutSec == 60)
        backlightTimeoutSec = 30;
      else if (backlightTimeoutSec == 30)
        backlightTimeoutSec = 20;
      else if (backlightTimeoutSec == 20)
        backlightTimeoutSec = 10;
      else
        backlightTimeoutSec = 0;
      wakeBacklight();
      forceRefresh = true;
    }
    if (isPressed(BTN_OK) || isPressed(BTN_MENU))
      changeState(STATE_MENU_LIST);
  }

  // Clear on state change
  if (currentState != prevState) {
    lcd.clear();
    prevState = currentState;
    forceRefresh = true;
  }

  // Render
  switch (currentState) {

  case STATE_SENSOR_ERROR:
    if (forceRefresh) {
      lcd.setCursor(0, 0);
      lcd.print("! SENSOR ERROR !");
      lcd.setCursor(0, 1);
      char buf[17];
      if (errorSensorIdx >= 0)
        snprintf(buf, sizeof(buf), "S%d DISCONNECTED ", errorSensorIdx + 1);
      else
        snprintf(buf, sizeof(buf), "CHECK SENSORS   ");
      lcd.print(buf);
    }
    break;

  case STATE_MAIN: {
    static uint32_t cycleTimer = 0;
    if (millis() - cycleTimer >= DISPLAY_CYCLE_TIME) {
      cycleTimer = millis();
      currentCycle = (currentCycle == CYCLE_TEMP) ? CYCLE_HUMIDITY : CYCLE_TEMP;
      forceRefresh = true;
    }

    bool dataChange = forceRefresh || currentCycle != prevCycle ||
                      onlineCount != prevOnline || motorActive != prevMotor ||
                      isManualMode != prevManualMode ||
                      fabsf(medianTemp - prevTemp) > 0.05f;

    bool spinChange = (motorActive && (spinIdx != prevSpinIdx));
    bool pulseChange =
        (isManualMode && motorActive && (pulseIdx != prevPulseIdx));
    bool blinkChange = (blinkIdx != prevBlinkIdx);

    if (dataChange) {
      // --- Row 0 ---
      if (isManualMode) {
        lcd.setCursor(8, 0);
        lcd.print(motorActive ? F(":ON    ") : F(": OFF   "));
      } else {
        lcd.setCursor(4, 0);
        char countBuf[6];
        snprintf(countBuf, sizeof(countBuf), "%d/4 ", onlineCount);
        lcd.print(countBuf);
        for (int i = 0; i < NUM_SENSORS; i++) {
          char mark =
              sensors[i].online ? '+' : (sensors[i].everConnected ? '!' : '?');
          lcd.print((char)('1' + i));
          lcd.print(mark);
        }
      }

      // --- Row 1 ---
      lcd.setCursor(0, 1);
      if (onlineCount == 0 && !isManualMode) {
        // v1.1: Explicit message when motor is forced off due to no sensors
        lcd.print(F("NO SENSOR-MTR OFF"));
      } else {
        char row1[17];
        if (currentCycle == CYCLE_TEMP) {
          char val[6];
          dtostrf(medianTemp, 4, 1, val);
          snprintf(row1, sizeof(row1), "T:%sC %s      ", val,
                   motorActive ? "[ON] " : "[OFF]");
        } else {
          char val[6];
          dtostrf(medianHumidity, 4, 1, val);
          snprintf(row1, sizeof(row1), "H:%s%% %s      ", val,
                   motorActive ? "[ON] " : "[OFF]");
        }
        lcd.print(row1);
      }
    }

    // --- Micro-Updates (Animations) ---
    if (blinkChange || dataChange) {
      lcd.setCursor(0, 0);
      if (isManualMode) {
        lcd.print(blinkIdx == 0 ? F("MNL MODE") : F("        "));
      } else {
        if (onlineCount < NUM_SENSORS) {
          lcd.print(blinkIdx == 0 ? F("AUTO") : F("ERR!"));
        } else {
          lcd.print(blinkIdx == 0 ? F("AUTO") : F("    "));
        }
      }
    }

    if (isManualMode) {
      if (motorActive && (pulseChange || dataChange)) {
        const char *pulses[] = {"*", ".", " "};
        lcd.setCursor(11, 0);
        lcd.print(pulses[pulseIdx]);
      }
    }

    if (motorActive && (spinChange || dataChange)) {
      lcd.setCursor(8, 1);
      lcd.print(SPIN_FRAMES[spinIdx]);
    }

    prevCycle = currentCycle;
    prevOnline = onlineCount;
    prevMotor = motorActive;
    prevTemp = medianTemp;
    prevManualMode = isManualMode;
    prevSpinIdx = spinIdx;
    prevBlinkIdx = blinkIdx;
    prevPulseIdx = pulseIdx;
    break;
  }

  case STATE_MENU_LIST:
    if (forceRefresh) {
      lcd.setCursor(0, 0);
      lcd.print("Select Settings:");
      lcd.setCursor(0, 1);
      lcd.print("> ");
      lcd.print(menuItems[menuIdx]);
    }
    break;

  case STATE_SET_TEMP_START:
    if (forceRefresh || startTemp != prevStart) {
      lcd.setCursor(0, 0);
      lcd.print(F("Motor START  T: "));
      lcd.setCursor(0, 1);
      char val[6]; dtostrf(startTemp, 4, 1, val);
      char buf[17];
      snprintf(buf, sizeof(buf), "%s C   [OK/M] ", val);
      lcd.print(buf);
    }
    prevStart = startTemp;
    break;

  case STATE_SET_TEMP_STOP:
    if (forceRefresh || stopTemp != prevStop) {
      lcd.setCursor(0, 0);
      lcd.print(F("Motor STOP   T: "));
      lcd.setCursor(0, 1);
      char val[6]; dtostrf(stopTemp, 4, 1, val);
      char buf[17];
      snprintf(buf, sizeof(buf), "%s C   [OK/M] ", val);
      lcd.print(buf);
    }
    prevStop = stopTemp;
    break;

  case STATE_SET_TIME:
    if (forceRefresh || minRunTimeMin != prevMin) {
      lcd.setCursor(0, 0);
      lcd.print("Min Motor Time: ");
      lcd.setCursor(0, 1);
      char buf[17];
      snprintf(buf, sizeof(buf), "%lu Min  [OK/M]  ", minRunTimeMin);
      lcd.print(buf);
    }
    prevMin = minRunTimeMin;
    break;

  case STATE_SET_BACKLIGHT:
    if (forceRefresh || backlightTimeoutSec != prevBL) {
      lcd.setCursor(0, 0);
      lcd.print("Backlight Off:  ");
      lcd.setCursor(0, 1);
      if (backlightTimeoutSec == 0) {
        lcd.print("Always ON [OK/M]");
      } else if (backlightTimeoutSec < 60) {
        char buf[17];
        snprintf(buf, sizeof(buf), "%lu Sec    [OK/M]", backlightTimeoutSec);
        lcd.print(buf);
      } else {
        char buf[17];
        snprintf(buf, sizeof(buf), "%lu Min    [OK/M]",
                 backlightTimeoutSec / 60);
        lcd.print(buf);
      }
    }
    prevBL = backlightTimeoutSec;
    break;
  }

  forceRefresh = false;
}

// ===============================================================
//  STARTUP SENSOR PROBE
// ===============================================================
void probeSensorsAtStartup() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Checking Sensors"));
  delay(500);

  int found = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    lcd.setCursor(0, 1);
    char scanBuf[17];
    snprintf(scanBuf, sizeof(scanBuf), "S%d: Scanning... ", i + 1);
    lcd.print(scanBuf);

    Serial.print(F("Scanning S"));
    Serial.print(i + 1);
    Serial.print(F("... "));

    bool ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
      float t = dhts[i].readTemperature();
      float h = dhts[i].readHumidity();

      if (!isnan(t) && !isnan(h)) {
        sensors[i].lastTemp = t;
        sensors[i].lastHumidity = h;
        sensors[i].online = true;
        sensors[i].everConnected = true;
        sensors[i].lastValidTime = millis();
        sensors[i].failStreak = 0;
        ok = true;
        found++;
        Serial.print(F("OK  T="));
        Serial.print(t);
        Serial.print(F("  H="));
        Serial.println(h);
        break;
      }

      if (attempt < 2) {
        delay(2100);
      }
    }

    if (!ok) {
      sensors[i].online = false;
      sensors[i].everConnected = false;
      sensors[i].lastValidTime = 0;
      Serial.println(F("NOT FOUND"));
    }

    lcd.setCursor(0, 1);
    char resBuf[17];
    snprintf(resBuf, sizeof(resBuf), "S%d:%s  %d/%d OK  ", i + 1,
             ok ? "OK" : "--", found, i + 1);
    lcd.print(resBuf);
    delay(300);
  }

  recomputeAverages();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Sensors Ready:  "));
  lcd.setCursor(0, 1);
  char summary[17];
  snprintf(summary, sizeof(summary), "%d of %d online", found, NUM_SENSORS);
  lcd.print(summary);
  Serial.print(F("Startup: "));
  Serial.print(found);
  Serial.print(F("/"));
  Serial.print(NUM_SENSORS);
  Serial.println(F(" sensors online"));
  delay(2000);
}

// ===============================================================
//  DEVELOPER CREDIT SCROLL
// ===============================================================
void showDeveloperCredit() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Irrigation Sys"));
  lcd.setCursor(0, 1);
  lcd.print(F("Dev_Dilshan."));
  delay(2000);
}

// ===============================================================
//  SETUP
// ===============================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("=============================================="));
  Serial.println(F("  Irrigation Controller v1.1"));
  Serial.println(F("  Developed by: Thalagahapitiya"));
  Serial.println(F("  v1.1: Zero-sensor motor shutdown added"));
  Serial.println(F("=============================================="));

  uint8_t mode = (BUTTON_PRESSED == LOW) ? INPUT_PULLUP : INPUT;
  for (int i = 0; i < 4; i++)
    pinMode(buttons[i].pin, mode);
  pinMode(BTN_MANUAL, mode);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  for (int i = 0; i < NUM_SENSORS; i++)
    dhts[i].begin();
  for (int i = 0; i < NUM_SENSORS; i++)
    sensors[i] = {0.0f, 0.0f, false, false, 0, 0};

  lcd.init();
  lcd.backlight();
  backlightOn = true;
  lastActivityTime = millis();

  showDeveloperCredit();
  probeSensorsAtStartup();
  lcd.clear();
}

// ===============================================================
//  LOOP
// ===============================================================
void loop() {
  updateAnimations();
  updateManualButton();
  updateButtons();
  updateSensors();
  handleMotor();
  handleBacklight();
  updateUI();
}
