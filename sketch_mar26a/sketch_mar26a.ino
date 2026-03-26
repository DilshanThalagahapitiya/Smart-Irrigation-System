#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

/**
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
 *   - Manual S/S    -> Pin 11
 * ------------------------------------------------------------
 */

/**
 * PIN ASSIGNMENTS
 */
const uint8_t DHT_PINS[]   = {2, 3, 4, 5};
const uint8_t BTN_MENU     = 7;  
const uint8_t BTN_OK       = 8;
const uint8_t BTN_UP       = 9;
const uint8_t BTN_DOWN     = 10;
const uint8_t BTN_MANUAL   = 11;
const uint8_t RELAY_PIN    = 6;
#define RELAY_ON HIGH  // Change to LOW if your relay is active-low
#define RELAY_OFF LOW  // Change to HIGH if your relay is active-low

// BUTTON CONFIGURATION
#define BUTTON_PRESSED LOW // Set to LOW for Pull-up (GND), HIGH for Pull-down (5V)

/**
 * CONFIGURATION & THRESHOLDS
 */
#define DHT_TYPE DHT22
#define LCD_ADDR 0x27 // Usually 0x27 or 0x3F
#define LCD_COLS 16
#define LCD_ROWS 2

// Timing Intervals (ms)
const uint32_t SENSOR_INTERVAL      = 2000; 
const uint32_t DISPLAY_CYCLE_TIME   = 3000;
const uint32_t DEBOUNCE_TIME        = 50;

/**
 * SYSTEM STATES (State Machine)
 */
enum SystemState {
    STATE_MAIN,
    STATE_MENU_LIST,
    STATE_SET_TEMP,
    STATE_SET_TIME
};

enum MainDisplayCycle {
    CYCLE_TEMP,
    CYCLE_HUMIDITY
};

/**
 * GLOBAL OBJECTS
 */
DHT dhts[] = {
    DHT(DHT_PINS[0], DHT_TYPE),
    DHT(DHT_PINS[1], DHT_TYPE),
    DHT(DHT_PINS[2], DHT_TYPE),
    DHT(DHT_PINS[3], DHT_TYPE)
};

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

/**
 * SYSTEM VARIABLES
 */
SystemState currentState = STATE_MAIN;
MainDisplayCycle currentCycle = CYCLE_TEMP;

float medianTemp = 0.0;
float medianHumidity = 0.0;
float thresholdTemp = 30.0;  
uint32_t minRunTimeMin = 5; 
bool forceRefresh = true;

bool motorActive = false;
bool manualOverride = false;
uint32_t motorStartTime = 0;
uint32_t lastSensorUpdate = 0;
uint32_t lastDisplayCycle = 0;
int currentSensorIdx = 0;
float lastReadTemps[4] = {0,0,0,0};
float lastReadHumids[4] = {0,0,0,0};
bool sensorValid[4] = {false,false,false,false};

// Button structure for debouncing
struct Button {
    uint8_t pin;
    bool lastReading;
    bool state;
    bool pressed;
    uint32_t lastDebounce;
};

Button buttons[] = {
    {BTN_MENU,   HIGH, HIGH, false, 0},
    {BTN_OK,     HIGH, HIGH, false, 0},
    {BTN_UP,     HIGH, HIGH, false, 0},
    {BTN_DOWN,   HIGH, HIGH, false, 0},
    {BTN_MANUAL, HIGH, HIGH, false, 0}
};

/**
 * UTILITY: MEDIAN CALCULATION
 */
float getMedian(float data[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j] > data[j + 1]) {
                float temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }
    if (n % 2 == 0) return (data[n / 2 - 1] + data[n / 2]) / 2.0;
    return data[n / 2];
}

/**
 * INPUT HANDLING
 */
void updateButtons() {
    for (int i = 0; i < 5; i++) {
        bool reading = digitalRead(buttons[i].pin);
        if (reading != buttons[i].lastReading) {
            buttons[i].lastDebounce = millis();
        }
        if ((millis() - buttons[i].lastDebounce) > DEBOUNCE_TIME) {
            if (reading != buttons[i].state) {
                if (reading == BUTTON_PRESSED) { // Transition to pressed
                    buttons[i].pressed = true;
                    Serial.print("Button Pin "); Serial.print(buttons[i].pin); Serial.println(" Pressed");
                }
                buttons[i].state = reading;
            }
        }
        buttons[i].lastReading = reading;
    }
}

bool isPressed(uint8_t pin) {
    for (int i = 0; i < 5; i++) {
        if (buttons[i].pin == pin && buttons[i].pressed) {
            buttons[i].pressed = false; 
            return true;
        }
    }
    return false;
}

/**
 * SENSOR DATA ACQUISITION
 */
void updateSensors() {
    // Read one sensor every 500ms to avoid blocking for too long
    if (millis() - lastSensorUpdate < 500) return;
    lastSensorUpdate = millis();

    float t = dhts[currentSensorIdx].readTemperature();
    float h = dhts[currentSensorIdx].readHumidity();

    if (!isnan(t) && !isnan(h)) {
        lastReadTemps[currentSensorIdx] = t;
        lastReadHumids[currentSensorIdx] = h;
        sensorValid[currentSensorIdx] = true;
    } else {
        sensorValid[currentSensorIdx] = false;
    }

    currentSensorIdx = (currentSensorIdx + 1) % 4;

    // Calculate median from valid reading
    float validTemps[4];
    float validHumids[4];
    int count = 0;
    for (int i = 0; i < 4; i++) {
        if (sensorValid[i]) {
            validTemps[count] = lastReadTemps[i];
            validHumids[count] = lastReadHumids[i];
            count++;
        }
    }

    if (count > 0) {
        medianTemp = getMedian(validTemps, count);
        medianHumidity = getMedian(validHumids, count);
    }
}

/**
 * MOTOR LOGIC (Hysteresis & Min-Run Time)
 */
void handleMotor() {
    bool shouldBeOn = false;

    // Manual Override Logic
    if (isPressed(BTN_MANUAL)) {
        if (!manualOverride) {
            manualOverride = true;
            motorActive = true; 
            Serial.println("Manual: FORCED ON");
            motorStartTime = millis();
        } else {
            manualOverride = false; 
            Serial.println("Manual: RETURN TO AUTO");
        }
        forceRefresh = true;
    }

    if (!manualOverride) {
        if (medianTemp > thresholdTemp) {
            shouldBeOn = true;
        } else if (medianTemp < (thresholdTemp - 1.0)) {
            shouldBeOn = false;
        } else {
            shouldBeOn = motorActive; 
        }

        if (shouldBeOn && !motorActive) {
            motorActive = true;
            Serial.println("Motor: AUTO ON");
            motorStartTime = millis();
        } else if (!shouldBeOn && motorActive) {
            // Check if min run time in minutes has passed
            if (millis() - motorStartTime >= (minRunTimeMin * 60000UL)) {
                motorActive = false;
                Serial.println("Motor: AUTO OFF");
            }
        }
    }

    digitalWrite(RELAY_PIN, motorActive ? RELAY_ON : RELAY_OFF);
}

/**
 * LCD UI SYSTEM
 */
int menuIdx = 0;
const char* menuItems[] = {"1. Motor Temp", "2. Motor Time"};

void changeState(SystemState newState) {
    if (currentState != newState) {
        currentState = newState;
        forceRefresh = true;
        Serial.print("Navigation -> State: "); Serial.println(newState);
    }
}

void updateUI() {
    static SystemState displayState = (SystemState)-1;
    static MainDisplayCycle lastCycle = (MainDisplayCycle)-1;
    static float lastTemp = -1;
    static float lastThresh = -1;
    static uint32_t lastMin = 0;
    static bool lastMotor = false;

    // 1. INPUT HANDLING (Happen before rendering to be snappy)
    if (currentState == STATE_MAIN) {
        if (isPressed(BTN_MENU)) changeState(STATE_MENU_LIST);
    } else if (currentState == STATE_MENU_LIST) {
        if (isPressed(BTN_UP) || isPressed(BTN_DOWN)) { 
            menuIdx = 1 - menuIdx; 
            forceRefresh = true; 
        }
        if (isPressed(BTN_OK)) {
            if (menuIdx == 0) changeState(STATE_SET_TEMP);
            else changeState(STATE_SET_TIME);
        }
        if (isPressed(BTN_MENU)) changeState(STATE_MAIN);
    } else if (currentState == STATE_SET_TEMP) {
        if (isPressed(BTN_UP)) { thresholdTemp += 1.0; forceRefresh = true; }
        if (isPressed(BTN_DOWN)) { thresholdTemp -= 1.0; forceRefresh = true; }
        if (isPressed(BTN_OK) || isPressed(BTN_MENU)) changeState(STATE_MENU_LIST);
    } else if (currentState == STATE_SET_TIME) {
        if (isPressed(BTN_UP)) { minRunTimeMin += 5; forceRefresh = true; }
        if (isPressed(BTN_DOWN)) { if (minRunTimeMin > 5) minRunTimeMin -= 5; forceRefresh = true; }
        if (isPressed(BTN_OK) || isPressed(BTN_MENU)) changeState(STATE_MENU_LIST);
    }

    // 2. REFRESH DETECTION
    if (currentState != displayState) {
        lcd.clear();
        displayState = currentState;
        forceRefresh = true;
    }
    
    // 3. RENDERING
    switch (currentState) {
        case STATE_MAIN:
            if (millis() - lastDisplayCycle >= DISPLAY_CYCLE_TIME) {
                lastDisplayCycle = millis();
                currentCycle = (currentCycle == CYCLE_TEMP) ? CYCLE_HUMIDITY : CYCLE_TEMP;
                forceRefresh = true;
            }
            if (forceRefresh || currentCycle != lastCycle || (medianTemp - lastTemp > 0.1 || lastTemp - medianTemp > 0.1) || motorActive != lastMotor) {
                lcd.setCursor(0, 0);
                lcd.print(manualOverride ? "MANUAL: FORCE ON" : "System Status:  ");
                lcd.setCursor(0, 1);
                if (currentCycle == CYCLE_TEMP) {
                    lcd.print("T: "); lcd.print(medianTemp, 1); lcd.print("C  ");
                } else {
                    lcd.print("H: "); lcd.print(medianHumidity, 1); lcd.print("%  ");
                }
                lcd.setCursor(11, 1);
                lcd.print(motorActive ? "[ON] " : "[OFF]");
            }
            break;

        case STATE_MENU_LIST:
            if (forceRefresh) {
                lcd.setCursor(0, 0);
                lcd.print("Select Settings:");
                lcd.setCursor(0, 1);
                lcd.print("> "); lcd.print(menuItems[menuIdx]); lcd.print("      ");
            }
            break;

        case STATE_SET_TEMP:
            if (forceRefresh || thresholdTemp != lastThresh) {
                lcd.setCursor(0, 0);
                lcd.print("Motor Start T:");
                lcd.setCursor(0, 1);
                lcd.print(thresholdTemp, 1); lcd.print(" C   [OK/M]");
            }
            break;

        case STATE_SET_TIME:
            if (forceRefresh || minRunTimeMin != lastMin) {
                lcd.setCursor(0, 0);
                lcd.print("Min Motor Time:");
                lcd.setCursor(0, 1);
                lcd.print(minRunTimeMin); lcd.print(" Min [OK/M]");
            }
            break;
    }

    lastCycle = currentCycle;
    lastTemp = medianTemp;
    lastThresh = thresholdTemp;
    lastMin = minRunTimeMin;
    lastMotor = motorActive;
    forceRefresh = false;
}

/**
 * MAIN SETUP AND LOOP
 */
void setup() {
    Serial.begin(9600);
    Serial.println("Irrigation System Starting...");

    // Button setup
    // Use INPUT_PULLUP if BUTTON_PRESSED is LOW. Use INPUT if BUTTON_PRESSED is HIGH.
    uint8_t mode = (BUTTON_PRESSED == LOW) ? INPUT_PULLUP : INPUT;
    for (int i = 0; i < 5; i++) pinMode(buttons[i].pin, mode);
    
    // Relay setup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);

    // Sensor setup
    for (int i = 0; i < 4; i++) dhts[i].begin();

    // LCD initialization (A4 SDA, A5 SCL)
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Irrigation Sys");
    lcd.setCursor(0, 1);
    lcd.print("Stabilizing...");
    delay(2000); 
    lcd.clear();
}

void loop() {
    updateButtons();
    updateSensors();
    handleMotor();
    updateUI();
}

