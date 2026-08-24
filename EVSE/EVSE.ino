#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#include <PZEM004Tv30.h>
#include <WebServer.h>
#include <Preferences.h>
#include "config.h"

// Vehicle states based on CP voltage
enum VehicleState {
  STATE_A,  // No vehicle (12V)
  STATE_B,  // Vehicle connected, not ready (9V)
  STATE_C,  // Vehicle connected, ready, charging (6V)
  STATE_D,  // Vehicle connected, ventilation required (3V)
  STATE_E,  // Error/No pilot (0V)
  STATE_F   // EVSE Error (-12V)
};

// Charger states
enum ChargerState {
  CHARGER_IDLE,
  CHARGER_CONNECTED,
  CHARGER_CHARGING,
  CHARGER_ERROR,
  CHARGER_FINISHED
};

// Global variables
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);
WebServer server(WEB_SERVER_PORT);
Preferences preferences;

VehicleState currentVehicleState = STATE_A;
VehicleState previousVehicleState = STATE_A;
VehicleState confirmedVehicleState = STATE_A;  // State confirmed after delay
ChargerState chargerState = CHARGER_IDLE;

// Universal state change safety timing
unsigned long stateChangeTime = 0;
VehicleState pendingState = STATE_A;
bool stateConfirmed = true;

float chargingCurrent = DEFAULT_CURRENT;
float voltage = 0;
float current = 0;
float power = 0;
float energy = 0;
float frequency = 0;
float powerFactor = 0;

// Settings
float maxCurrentLimit = MAX_CURRENT;
float minCurrentLimit = MIN_CURRENT;
bool autoStartCharging = true;
int displayUpdateInterval = 500;

unsigned long lastDisplayUpdate = 0;
unsigned long lastStateCheck = 0;
unsigned long lastPZEMRead = 0;
unsigned long chargingStartTime = 0;
unsigned long chargingDuration = 0;
unsigned long lastWiFiCheck = 0;
unsigned long wifiStartTime = 0;
bool contactorClosed = false;
bool wifiConnected = false;
bool wifiConnecting = false;
bool isAPMode = false;  // Track if we're in Access Point mode
bool displayAvailable = false;  // Track if OLED display is available

int PWM_DutyCycle = 0;

// Fault latch. Once set, the contactor stays open and no charge may start until
// the fault is cleared -- either by unplugging the vehicle or from the web UI.
bool faultLatched = false;
const char* faultReason = "";

// Contactor timing. contactorOpenTime gates re-closing so a glitch cannot
// chatter the contactor; 0 means "never opened since boot".
unsigned long contactorOpenTime = 0;

// Consecutive raw CP samples that were not State C, used to force the
// contactor open without waiting for the state confirmation delay.
int nonChargeSamples = 0;

// Over-current trip timers (0 = not currently over the threshold)
unsigned long overCurrentSoftStart = 0;
unsigned long overCurrentHardStart = 0;

// PZEM health
int pzemFailCount = 0;
bool pzemOnline = false;

// Per-session energy, derived from the PZEM's lifetime total
float sessionStartEnergy = 0;
float sessionEnergy = 0;
bool sessionEnergyValid = false;

// Non-blocking station-mode retry while running as an access point
bool staRetryActive = false;
unsigned long staRetryStart = 0;

// Display sleep
unsigned long lastActivityTime = 0;
bool displaySleeping = false;

// ========================================
// PREFERENCES (FLASH STORAGE) FUNCTIONS
// ========================================

void loadSettings() {
  preferences.begin("evse", false);
  chargingCurrent = preferences.getFloat("chargeCurrent", DEFAULT_CURRENT);
  maxCurrentLimit = preferences.getFloat("maxCurrent", MAX_CURRENT);
  minCurrentLimit = preferences.getFloat("minCurrent", MIN_CURRENT);
  autoStartCharging = preferences.getBool("autoStart", true);
  displayUpdateInterval = preferences.getInt("dispInterval", 500);
  preferences.end();

  // Validate everything that came out of flash -- a corrupt or stale value must
  // never widen the current limits beyond what the hardware is rated for.
  if (maxCurrentLimit < MIN_CURRENT || maxCurrentLimit > MAX_CURRENT) {
    maxCurrentLimit = MAX_CURRENT;
  }
  if (minCurrentLimit < MIN_CURRENT || minCurrentLimit > MAX_CURRENT) {
    minCurrentLimit = MIN_CURRENT;
  }
  if (minCurrentLimit > maxCurrentLimit) {
    minCurrentLimit = MIN_CURRENT;
    maxCurrentLimit = MAX_CURRENT;
  }
  if (chargingCurrent < minCurrentLimit || chargingCurrent > maxCurrentLimit) {
    chargingCurrent = DEFAULT_CURRENT;
  }
  if (displayUpdateInterval < 100 || displayUpdateInterval > 2000) {
    displayUpdateInterval = 500;
  }
}

void saveChargingCurrent() {
  preferences.begin("evse", false);
  preferences.putFloat("chargeCurrent", chargingCurrent);
  preferences.end();
}

void saveSettings() {
  preferences.begin("evse", false);
  preferences.putFloat("chargeCurrent", chargingCurrent);
  preferences.putFloat("maxCurrent", maxCurrentLimit);
  preferences.putFloat("minCurrent", minCurrentLimit);
  preferences.putBool("autoStart", autoStartCharging);
  preferences.putInt("dispInterval", displayUpdateInterval);
  preferences.end();
}

void setup() {
  // Serial.begin(115200);
  // Load saved settings from flash memory
  loadSettings();
  
  // Initialize pins
  pinMode(CONTACTOR_PIN, OUTPUT);
  digitalWrite(CONTACTOR_PIN, LOW);
  pinMode(CP_SENSE_PIN, INPUT);
  pinMode(CP_PWM_PIN, OUTPUT);
  
  // Initialize PWM for CP signal
  if (!ledcAttach(CP_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    while(1); // Halt if PWM fails
  }
  
  // Start with 100% duty cycle (12V)
  PWM_DutyCycle = 1023;
  ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  
  // Initialize I2C for OLED
  Wire.begin();

  // Initialize OLED display
  if (!display.begin(I2C_ADDRESS, true)) {
    // Display failed to initialize - continue without it
    displayAvailable = false;
  } else {
    // Display initialized successfully
    displayAvailable = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("EV Charger v1.0");
    display.println("Initializing...");
    display.display();
    delay(2000);
  }

  lastActivityTime = millis();

  // Start WiFi connection in non-blocking mode
  startWiFiConnection();

  // Watchdog on the loop task. Every path through loop() is non-blocking, so a
  // missed feed means something is genuinely stuck and a reset is the right
  // answer -- the contactor is driven by a GPIO that resets low.
  enableLoopWDT();
}

// Start WiFi connection in non-blocking mode
void startWiFiConnection() {
  WiFi.disconnect(true);
  delay(100);
  
  WiFi.mode(WIFI_STA);
  delay(100);
  
  WiFi.setHostname("EV-Charger");
  WiFi.setSleep(false);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  wifiConnecting = true;
  wifiConnected = false;
  isAPMode = false;
  wifiStartTime = millis();
}

// Start Access Point mode (fallback when WiFi connection fails)
void startAPMode() {
  WiFi.disconnect(true);
  delay(100);
  
  WiFi.mode(WIFI_AP);
  delay(100);
  
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  if (apStarted) {
    isAPMode = true;
    wifiConnected = true;
    wifiConnecting = false;
    setupWebServer();
  } else {
    isAPMode = false;
    wifiConnected = false;
    wifiConnecting = false;
  }
}

// Check WiFi connection status (non-blocking)
void checkWiFiConnection() {
  if (!wifiConnecting) {
    return;
  }
  
  unsigned long elapsed = (millis() - wifiStartTime) / 1000;
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiConnecting = false;
    isAPMode = false;
    setupWebServer();
    return;
  }
  
  // Check timeout - switch to AP mode if connection fails
  if (elapsed >= WIFI_TIMEOUT_SECONDS) {
    wifiConnecting = false;
    startAPMode();
    return;
  }
}

// Read the control pilot's positive peak.
//
// The pilot is a 1kHz square wave whose positive level encodes the vehicle
// state, so this samples for a fixed WINDOW of time rather than a fixed number
// of samples: a sample count only spans a full PWM period if analogRead()
// happens to be fast enough, and missing the peak once would look exactly like
// the vehicle changing state.
int checkAnalog(int analogPinToTest, unsigned long windowMicros) {
  int maximum = 0;
  unsigned long start = micros();

  do {
    int value = analogRead(analogPinToTest);
    if (value > maximum) {
      maximum = value;
    }
  } while (micros() - start < windowMicros);

  return maximum; // Return peak value for PWM signal
}

// Function to calculate PWM value for charging current
int chargingPWM(int ampsToConvert) {
  if (ampsToConvert == 0) {
    return 1023; // 100% duty cycle = 12V DC for standby
  }
  
  // Calculate duty cycle based on J1772 standard
  // Duty cycle % = (Current / 0.6) for 6-51A range
  float dutyPercent = (float)ampsToConvert / 0.6;
  
  // Apply duty cycle adjustment from config.h
  dutyPercent += DUTY_CYCLE_ADJUSTMENT;
  
  // Clamp between 10% and 96%
  if (dutyPercent < 10) dutyPercent = 10;
  if (dutyPercent > 96) dutyPercent = 96;
  
  // Convert to 10-bit PWM value (0-1023)
  int pwmValue = (int)((dutyPercent / 100.0) * 1023);
  
  return pwmValue;
}

// Inspect and repair the WiFi link. Fully non-blocking: an attempt to move from
// AP mode back to station mode is started here and finished on a later pass, so
// vehicle state checking is never stalled waiting for a radio.
void maintainWiFi(unsigned long currentMillis) {
  // Finish an in-flight station retry that was started while in AP mode
  if (staRetryActive) {
    if (WiFi.status() == WL_CONNECTED) {
      server.stop();
      WiFi.mode(WIFI_STA);
      wifiConnected = true;
      isAPMode = false;
      staRetryActive = false;
      lastWiFiCheck = currentMillis;
      setupWebServer();
    } else if (currentMillis - staRetryStart >= STA_RETRY_WINDOW) {
      // No luck this round - drop the station interface and stay an AP.
      // Restart the interval here, otherwise the next pass would immediately
      // launch another retry because the 30s window elapsed during this one.
      WiFi.mode(WIFI_AP);
      staRetryActive = false;
      lastWiFiCheck = currentMillis;
    }
    return;
  }

  if (currentMillis - lastWiFiCheck < WIFI_CHECK_INTERVAL) {
    return;
  }
  lastWiFiCheck = currentMillis;

  if (isAPMode) {
    // Kick off a station attempt alongside the AP and let the block above
    // finish it. AP clients keep working throughout.
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    staRetryActive = true;
    staRetryStart = currentMillis;
    return;
  }

  // Station mode: monitor and repair
  if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    wifiConnected = false;
    server.stop();
    startAPMode();
  } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    setupWebServer();
  } else if (!wifiConnected && !wifiConnecting) {
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiConnecting = true;
    wifiStartTime = currentMillis;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Check WiFi connection status (non-blocking)
  if (wifiConnecting) {
    checkWiFiConnection();
  }

  maintainWiFi(currentMillis);

  // Handle web server if WiFi is connected (STA or AP mode)
  if (wifiConnected) {
    server.handleClient();
  }

  // Check vehicle state every 100ms
  if (currentMillis - lastStateCheck >= STATE_CHECK_INTERVAL) {
    lastStateCheck = currentMillis;
    checkVehicleState();
    updateChargerState();
  }

  // Read PZEM data. Back off hard once the meter stops answering, because each
  // failed read blocks this loop for up to 100ms inside the PZEM library.
  unsigned long pzemInterval = (pzemFailCount >= PZEM_FAIL_LIMIT) ? PZEM_RETRY_INTERVAL : PZEM_READ_INTERVAL;
  if (currentMillis - lastPZEMRead >= pzemInterval) {
    lastPZEMRead = currentMillis;
    readPZEMData();
    checkOverCurrent();
  }

  // Update display based on configured interval (only if display is available)
  if (displayAvailable && currentMillis - lastDisplayUpdate >= (unsigned long)displayUpdateInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }

  // Update charging duration
  if (chargerState == CHARGER_CHARGING && contactorClosed) {
    chargingDuration = (currentMillis - chargingStartTime) / 1000;
  }

  feedLoopWDT();
}

void checkVehicleState() {
  int cpVoltage = checkAnalog(CP_SENSE_PIN, CP_SAMPLE_WINDOW_US);
  // Serial.println("CP Value: " + String(cpVoltage));
  
  // Determine raw vehicle state based on CP voltage
  VehicleState detectedState;
  
  if (cpVoltage >= CP_12V_MIN && cpVoltage <= CP_12V_MAX) {
    detectedState = STATE_A; // No vehicle
  } else if (cpVoltage >= CP_9V_MIN && cpVoltage <= CP_9V_MAX) {
    detectedState = STATE_B; // Vehicle connected, not ready
  } else if (cpVoltage >= CP_6V_MIN && cpVoltage <= CP_6V_MAX) {
    detectedState = STATE_C; // Vehicle ready, charging
  } else if (cpVoltage >= CP_3V_MIN && cpVoltage <= CP_3V_MAX) {
    detectedState = STATE_D; // Ventilation required
  } else if (cpVoltage >= CP_0V_MIN && cpVoltage <= CP_0V_MAX) {
    detectedState = STATE_E; // No power
  } else {
    detectedState = STATE_F; // Error
  }

  // SAFETY: stopping does not wait for the confirmation delay.
  // That delay exists to keep noise from STARTING a charge. Applying it in the
  // other direction would leave the cable energised for two full seconds after
  // the connector is pulled or the pilot faults, so instead the contactor opens
  // as soon as CONTACTOR_TRIP_SAMPLES consecutive readings are not State C
  // (~200ms), which is still fast while ignoring a single bad sample.
  if (detectedState != STATE_C) {
    if (nonChargeSamples < CONTACTOR_TRIP_SAMPLES) {
      nonChargeSamples++;
    }
  } else {
    nonChargeSamples = 0;
  }

  if (contactorClosed && nonChargeSamples >= CONTACTOR_TRIP_SAMPLES) {
    stopCharging();
    if (chargerState == CHARGER_CHARGING) {
      chargerState = CHARGER_CONNECTED;
    }
  }

  // Universal state change confirmation logic
  if (detectedState != confirmedVehicleState) {
    // Detected state is different from confirmed state
    if (detectedState == pendingState) {
      // Same pending state detected again - check if enough time has passed
      unsigned long timeSinceChange = millis() - stateChangeTime;
      if (timeSinceChange >= STATE_CHANGE_DELAY) {
        // State has been stable for required duration - confirm it
        previousVehicleState = confirmedVehicleState;
        confirmedVehicleState = pendingState;
        currentVehicleState = confirmedVehicleState;
        stateConfirmed = true;
        noteActivity();
      } else {
        // Still waiting for confirmation
        stateConfirmed = false;
      }
    } else {
      // New different state detected - start new confirmation timer
      pendingState = detectedState;
      stateChangeTime = millis();
      stateConfirmed = false;
    }
  } else {
    // Detected state matches confirmed state - all is stable
    currentVehicleState = confirmedVehicleState;
    pendingState = confirmedVehicleState;
    stateConfirmed = true;
  }
}

// Latch a fault: open the contactor and refuse to charge until cleared.
// Used for operator intent (emergency stop) and for conditions that need a
// human to look at the installation (over-current). Transient pilot faults
// deliberately do NOT latch -- they recover on their own once the pilot is
// valid again, and the confirmation delay already guards the restart.
void latchFault(const char* reason) {
  stopCharging();
  faultLatched = true;
  faultReason = reason;
  chargerState = CHARGER_ERROR;
  noteActivity();
}

void clearFault() {
  faultLatched = false;
  faultReason = "";
  if (chargerState == CHARGER_ERROR) {
    chargerState = CHARGER_IDLE;
  }
}

void updateChargerState() {
  // A latched fault outranks everything below it
  if (faultLatched) {
    stopCharging();
    // Unplugging the vehicle is the physical acknowledgement of the fault
    if (stateConfirmed && currentVehicleState == STATE_A) {
      clearFault();
    } else {
      // Hold a steady +12V pilot: tells the vehicle the EVSE is present but
      // not available, so it stops asking for power instead of erroring out.
      PWM_DutyCycle = 1023;
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      chargerState = CHARGER_ERROR;
      return;
    }
  }

  // Only act on CONFIRMED states to prevent spurious changes
  if (!stateConfirmed) {
    // State is not yet confirmed - don't make any changes
    return;
  }

  int ampsPWM = chargingPWM(chargingCurrent);

  switch (currentVehicleState) {
    case STATE_A:
      // No vehicle connected
      if (chargerState != CHARGER_IDLE) {
        stopCharging();
        chargerState = CHARGER_IDLE;
      }
      PWM_DutyCycle = 1023; // 100% duty cycle = 12V DC
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      break;
      
    case STATE_B:
      // Vehicle connected but not ready
      stopCharging();
      chargerState = CHARGER_CONNECTED;
      PWM_DutyCycle = ampsPWM;
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      break;
      
    case STATE_C:
      // Vehicle ready to charge
      PWM_DutyCycle = ampsPWM;
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);

      if (!autoStartCharging) {
        stopCharging();
        chargerState = CHARGER_CONNECTED;
      } else if (chargerState != CHARGER_CHARGING) {
        // startCharging() re-runs the safety checks and may decline (for
        // example during the re-close hold-off), so only claim to be charging
        // once the contactor has actually closed.
        if (startCharging()) {
          chargerState = CHARGER_CHARGING;
        } else {
          chargerState = CHARGER_CONNECTED;
        }
      }
      break;
      
    case STATE_D:
      // Ventilation required (not supported in this version)
      stopCharging();
      chargerState = CHARGER_ERROR;
      PWM_DutyCycle = ampsPWM;
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      break;
      
    case STATE_E:
      // No power available
      stopCharging();
      chargerState = CHARGER_ERROR;
      PWM_DutyCycle = 1023; // 12V DC
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      break;
      
    case STATE_F:
      // Error state
      stopCharging();
      chargerState = CHARGER_ERROR;
      PWM_DutyCycle = 0; // Turn off PWM
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      break;
  }
}

// Returns true only if the contactor actually closed.
bool startCharging() {
  // Safety checks - every one of these must hold before energising the cable
  if (faultLatched) {
    return false;
  }
  if (!stateConfirmed || currentVehicleState != STATE_C) {
    return false; // Abort if state not confirmed or not in STATE_C
  }
  // Re-close hold-off: never slam the contactor straight back after a stop
  if (contactorOpenTime != 0 && millis() - contactorOpenTime < CONTACTOR_RECLOSE_DELAY) {
    return false;
  }

  // All safety checks passed - proceed with charging
  int ampsPWM = chargingPWM(chargingCurrent);
  PWM_DutyCycle = ampsPWM;
  ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  delay(100);
  digitalWrite(CONTACTOR_PIN, HIGH);
  contactorClosed = true;
  chargingStartTime = millis();
  chargingDuration = 0;

  // Start a new energy session from the meter's lifetime total
  sessionStartEnergy = energy;
  sessionEnergy = 0;
  sessionEnergyValid = pzemOnline;
  return true;
}

void stopCharging() {
  if (contactorClosed) {
    digitalWrite(CONTACTOR_PIN, LOW);
    contactorClosed = false;
    contactorOpenTime = millis();
  }
  overCurrentSoftStart = 0;
  overCurrentHardStart = 0;
}

void readPZEMData() {
  // One read is enough to trigger a single Modbus transaction; the library
  // caches the whole frame for 200ms, so the calls below are free.
  float v = pzem.voltage();

  if (isnan(v)) {
    if (pzemFailCount < PZEM_FAIL_LIMIT) {
      pzemFailCount++;
    }
    if (pzemFailCount >= PZEM_FAIL_LIMIT) {
      pzemOnline = false;
      voltage = 0;
      current = 0;
      power = 0;
      frequency = 0;
      powerFactor = 0;
    }
    return;
  }

  pzemFailCount = 0;
  pzemOnline = true;

  voltage = v;
  current = pzem.current();
  power = pzem.power();
  energy = pzem.energy();
  frequency = pzem.frequency();
  powerFactor = pzem.pf();

  // Check for NaN values
  if (isnan(current)) current = 0;
  if (isnan(power)) power = 0;
  if (isnan(energy)) energy = 0;
  if (isnan(frequency)) frequency = 0;
  if (isnan(powerFactor)) powerFactor = 0;

  // Track energy delivered in this charging session
  if (contactorClosed) {
    if (!sessionEnergyValid) {
      // Meter was offline when the session started - start counting from here
      sessionStartEnergy = energy;
      sessionEnergyValid = true;
    }
    sessionEnergy = energy - sessionStartEnergy;
    if (sessionEnergy < 0) sessionEnergy = 0;  // meter was reset mid-session
  }
}

// Compare measured current against what the pilot advertises to the vehicle.
// A vehicle that ignores the pilot duty cycle, or a fault in the pilot chain,
// shows up here as sustained over-current -- nothing else in the firmware was
// watching for it.
void checkOverCurrent() {
  if (!contactorClosed || !pzemOnline) {
    overCurrentSoftStart = 0;
    overCurrentHardStart = 0;
    return;
  }

  unsigned long now = millis();
  float softLimit = chargingCurrent * OVERCURRENT_SOFT_RATIO + OVERCURRENT_MARGIN;
  float hardLimit = chargingCurrent * OVERCURRENT_HARD_RATIO + OVERCURRENT_MARGIN;

  if (current > hardLimit) {
    if (overCurrentHardStart == 0) {
      overCurrentHardStart = now;
    } else if (now - overCurrentHardStart >= OVERCURRENT_HARD_TIME) {
      latchFault("Over-current");
      return;
    }
  } else {
    overCurrentHardStart = 0;
  }

  if (current > softLimit) {
    if (overCurrentSoftStart == 0) {
      overCurrentSoftStart = now;
    } else if (now - overCurrentSoftStart >= OVERCURRENT_SOFT_TIME) {
      latchFault("Over-current");
      return;
    }
  } else {
    overCurrentSoftStart = 0;
  }
}

// Something worth looking at happened - keep the panel awake
void noteActivity() {
  lastActivityTime = millis();
}

void updateDisplay() {
  // Power saving: blank the panel when the charger has been sitting idle.
  // Any state change calls noteActivity() and brings it straight back.
  if (DISPLAY_SLEEP_TIMEOUT > 0 && !contactorClosed && !faultLatched &&
      millis() - lastActivityTime >= DISPLAY_SLEEP_TIMEOUT) {
    if (!displaySleeping) {
      display.clearDisplay();
      display.display();
      display.oled_command(SH110X_DISPLAYOFF);
      displaySleeping = true;
    }
    return;
  }
  if (displaySleeping) {
    display.oled_command(SH110X_DISPLAYON);
    displaySleeping = false;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Title
  display.println("EV CHARGER");
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SH110X_WHITE);

  // State
  display.setCursor(0, 14);

  if (faultLatched) {
    display.print("FAULT:");
    display.println(faultReason);
  } else if (!stateConfirmed) {
    // Show state confirmation countdown if waiting
    unsigned long timeWaiting = millis() - stateChangeTime;
    unsigned long remainingTime = (STATE_CHANGE_DELAY - timeWaiting) / 1000;
    display.print("State: ");
    display.print(getVehicleStateName(pendingState));
    display.print(" (");
    display.print(remainingTime);
    display.println("s)");
  } else {
    display.print("State: ");
    display.println(getChargerStateName(chargerState));
  }

  // Voltage and Current
  display.setCursor(0, 24);
  if (pzemOnline) {
    display.print("V:");
    display.print(voltage, 1);
    display.print("V I:");
    display.print(current, 2);
    display.println("A");
  } else {
    display.println("Meter: offline");
  }

  // Power and PWM info
  display.setCursor(0, 34);
  display.print("P:");
  display.print(power, 1);
  display.print("W PWM:");
  display.println(PWM_DutyCycle);

  // Energy: this session while one is running, lifetime total otherwise
  display.setCursor(0, 44);
  if (sessionEnergy > 0 || contactorClosed) {
    display.print("S:");
    display.print(sessionEnergy, 2);
  } else {
    display.print("E:");
    display.print(energy, 2);
  }
  display.print("kWh Set:");
  display.print(chargingCurrent, 0);
  display.println("A");

  // IP Address (always show if WiFi connected) or Charging duration
  display.setCursor(0, 54);
  if (wifiConnecting) {
    // Show WiFi connecting status
    unsigned long elapsed = (millis() - wifiStartTime) / 1000;
    display.print("WiFi: ");
    display.print(elapsed);
    display.print("s...");
  } else if (wifiConnected) {
    // Show connection info based on mode
    if (isAPMode) {
      display.print("AP: ");
      display.println(WiFi.softAPIP());
    } else if (WiFi.status() == WL_CONNECTED) {
      display.print("IP: ");
      display.println(WiFi.localIP());
    }
  } else if (chargerState == CHARGER_CHARGING || chargingDuration > 0) {
    display.print("Time: ");
    int hours = chargingDuration / 3600;
    int minutes = (chargingDuration % 3600) / 60;
    int seconds = chargingDuration % 60;
    
    if (hours > 0) {
      display.print(hours);
      display.print("h ");
    }
    display.print(minutes);
    display.print("m ");
    display.print(seconds);
    display.print("s");
  } else {
    display.print("WiFi: Offline");
  }
  
  display.display();
}

String getVehicleStateName(VehicleState state) {
  switch (state) {
    case STATE_A: return "A: No Vehicle";
    case STATE_B: return "B: Connected";
    case STATE_C: return "C: Charging";
    case STATE_D: return "D: Vent Req";
    case STATE_E: return "E: No Power";
    case STATE_F: return "F: EVSE Error";
    default: return "Unknown";
  }
}

String getChargerStateName(ChargerState state) {
  switch (state) {
    case CHARGER_IDLE: return "Idle";
    case CHARGER_CONNECTED: return "Connected";
    case CHARGER_CHARGING: return "Charging";
    case CHARGER_ERROR: return "Error";
    case CHARGER_FINISHED: return "Finished";
    default: return "Unknown";
  }
}

// ========================================
// WEB SERVER FUNCTIONS
// ========================================

// Both pages below are fully static and are streamed straight out of flash with
// server.send_P(). Nothing is templated into them and no String is built per
// request -- every dynamic value arrives through /data instead. That matters on
// a charger that stays powered for months: the old approach copied ~8KB to the
// heap and reallocated it a dozen times on every page load, which fragments the
// heap until the web server stops responding.
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>EV Charger Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 20px;
      background-color: #f0f0f0;
    }
    .container {
      max-width: 600px;
      margin: 0 auto;
      background-color: white;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    }
    h1 {
      color: #2c3e50;
      margin-bottom: 10px;
    }
    .status {
      background-color: #3498db;
      color: white;
      padding: 15px;
      border-radius: 5px;
      margin: 20px 0;
    }
    .status.charging {
      background-color: #27ae60;
    }
    .status.error {
      background-color: #e74c3c;
    }
    .readings-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 15px;
      margin: 20px 0;
    }
    .reading-card {
      background-color: #f8f9fa;
      padding: 15px;
      border-radius: 5px;
      border-left: 4px solid #3498db;
    }
    .reading-label {
      font-weight: bold;
      color: #7f8c8d;
      font-size: 0.9em;
    }
    .reading-value {
      font-size: 1.8em;
      color: #2c3e50;
      font-weight: bold;
    }
    .control-section {
      margin: 30px 0;
      padding: 20px;
      background-color: #ecf0f1;
      border-radius: 5px;
    }
    .slider-container {
      margin: 20px 0;
    }
    .slider {
      width: 80%;
      height: 25px;
      -webkit-appearance: none;
      appearance: none;
      background: #d3d3d3;
      outline: none;
      border-radius: 15px;
    }
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 35px;
      height: 35px;
      background: #3498db;
      cursor: pointer;
      border-radius: 50%;
    }
    .slider::-moz-range-thumb {
      width: 35px;
      height: 35px;
      background: #3498db;
      cursor: pointer;
      border-radius: 50%;
    }
    .current-display {
      font-size: 2.5em;
      color: #3498db;
      font-weight: bold;
      margin: 10px 0;
    }
    button {
      background-color: #3498db;
      color: white;
      border: none;
      padding: 15px 30px;
      font-size: 1.1em;
      border-radius: 5px;
      cursor: pointer;
      margin: 10px;
      transition: background-color 0.3s;
    }
    button:hover {
      background-color: #2980b9;
    }
    button:active {
      background-color: #21618c;
    }
    .emergency-stop {
      background-color: #e74c3c;
    }
    .emergency-stop:hover {
      background-color: #c0392b;
    }
    .fault-banner {
      display: none;
      background-color: #e74c3c;
      color: white;
      padding: 15px;
      border-radius: 5px;
      margin: 20px 0;
      font-weight: bold;
    }
    .clear-fault {
      background-color: #e67e22;
    }
    .clear-fault:hover {
      background-color: #d35400;
    }
    .quick-buttons {
      margin: 15px 0;
    }
    .quick-btn {
      padding: 10px 20px;
      margin: 5px;
      font-size: 0.9em;
    }
    .nav-buttons {
      margin: 20px 0;
      display: flex;
      gap: 10px;
      justify-content: center;
    }
    .nav-btn {
      background-color: #95a5a6;
      color: white;
      border: none;
      padding: 10px 20px;
      border-radius: 5px;
      cursor: pointer;
      text-decoration: none;
      display: inline-block;
    }
    .nav-btn:hover {
      background-color: #7f8c8d;
    }
    .nav-btn.active {
      background-color: #3498db;
    }
  </style>
  <script>
    // Timestamp of the last slider touch. The 2s poll must not yank the slider
    // out from under someone who is still dragging it.
    var sliderTouched = 0;

    function txt(id, value) {
      document.getElementById(id).innerText = value;
    }

    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          txt('voltage', data.meter ? data.voltage : '--');
          txt('current', data.meter ? data.current : '--');
          txt('power', data.meter ? data.power : '--');
          txt('energy', data.meter ? data.energy : '--');
          txt('sessionEnergy', data.meter ? data.sessionEnergy : '--');
          txt('setCurrent', data.setCurrent);
          txt('chargerState', data.chargerState);
          txt('vehicleState', data.vehicleState);
          txt('uptime', data.uptime);
          txt('pwm', data.pwm);
          txt('wifiMode', data.wifiMode);
          txt('ip', data.ip);

          if (Date.now() - sliderTouched > 10000) {
            document.getElementById('currentSlider').value = data.setCurrent;
            txt('sliderValue', data.setCurrent);
          }

          var banner = document.getElementById('faultBanner');
          if (data.fault) {
            banner.style.display = 'block';
            txt('faultReason', data.faultReason);
          } else {
            banner.style.display = 'none';
          }

          var statusDiv = document.getElementById('statusDiv');
          statusDiv.className = 'status';
          if (data.chargerState === 'Charging') {
            statusDiv.className = 'status charging';
          } else if (data.chargerState === 'Error') {
            statusDiv.className = 'status error';
          }
        })
        .catch(error => console.error('Error fetching data:', error));
    }

    // State-changing endpoints are POST so that a browser prefetch, a bookmark
    // or a network scanner cannot trip the charger by fetching a URL.
    function post(url) {
      return fetch(url, { method: 'POST' })
        .then(updateData)
        .catch(error => console.error('Request failed:', error));
    }

    function applyCurrent() {
      sliderTouched = 0;
      post('/setCurrent?value=' + document.getElementById('currentSlider').value);
    }

    function quickSet(value) {
      sliderTouched = 0;
      post('/setCurrent?value=' + value);
    }

    function emergencyStop() {
      if (confirm('Are you sure you want to emergency stop?')) {
        post('/emergencyStop');
      }
    }

    function clearFault() {
      post('/clearFault');
    }

    // Update every 2 seconds
    setInterval(updateData, 2000);

    window.onload = function() {
      var slider = document.getElementById('currentSlider');
      slider.addEventListener('input', function() {
        sliderTouched = Date.now();
        txt('sliderValue', slider.value);
      });
      updateData();
    };
  </script>
</head>
<body>
  <div class="container">
    <h1>EV Charger Control</h1>
    
    <div class="nav-buttons">
      <a href="/" class="nav-btn active">Dashboard</a>
      <a href="/settings" class="nav-btn">Settings</a>
    </div>
    
    <div id="faultBanner" class="fault-banner">
      FAULT LATCHED: <span id="faultReason"></span><br>
      Charging is blocked until this is cleared or the vehicle is unplugged.
      <br>
      <button class="clear-fault" onclick="clearFault()">Clear Fault</button>
    </div>

    <div id="statusDiv" class="status">
      <h2>Status: <span id="chargerState">--</span></h2>
      <div>Vehicle State: <span id="vehicleState">--</span></div>
    </div>

    <div class="readings-grid">
      <div class="reading-card">
        <div class="reading-label">Voltage</div>
        <div class="reading-value"><span id="voltage">--</span> V</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Current</div>
        <div class="reading-value"><span id="current">--</span> A</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Power</div>
        <div class="reading-value"><span id="power">--</span> W</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">This Session</div>
        <div class="reading-value"><span id="sessionEnergy">--</span> kWh</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Total Energy</div>
        <div class="reading-value"><span id="energy">--</span> kWh</div>
      </div>
    </div>

    <div class="control-section">
      <h3>Charging Current Limit</h3>
      <div class="current-display"><span id="setCurrent">--</span> A</div>

      <div class="slider-container">
        <input type="range" min="6" max="32" value="8"
               class="slider" id="currentSlider" name="value">
      </div>
      <div style="font-size: 1.5em; margin: 10px 0;">
        <span id="sliderValue">8</span> A
      </div>
      <button type="button" onclick="applyCurrent()">Set Current</button>

      <div class="quick-buttons">
        <strong>Quick Set:</strong><br>
        <button class="quick-btn" onclick="quickSet(6)">6A</button>
        <button class="quick-btn" onclick="quickSet(8)">8A</button>
        <button class="quick-btn" onclick="quickSet(10)">10A</button>
        <button class="quick-btn" onclick="quickSet(16)">16A</button>
        <button class="quick-btn" onclick="quickSet(20)">20A</button>
        <button class="quick-btn" onclick="quickSet(32)">32A</button>
      </div>
    </div>

    <div>
      <button class="emergency-stop" onclick="emergencyStop()">
        EMERGENCY STOP
      </button>
    </div>

    <div style="margin-top: 20px; color: #7f8c8d; font-size: 0.9em;">
      <span id="wifiMode">--</span> | IP: <span id="ip">--</span> |
      Uptime: <span id="uptime">--</span> | PWM: <span id="pwm">--</span>
    </div>
  </div>
</body>
</html>
)rawliteral";

// HTML template for settings page
static const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>EV Charger Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 20px;
      background-color: #f0f0f0;
    }
    .container {
      max-width: 600px;
      margin: 0 auto;
      background-color: white;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    }
    h1 {
      color: #2c3e50;
      margin-bottom: 10px;
    }
    .nav-buttons {
      margin: 20px 0;
      display: flex;
      gap: 10px;
      justify-content: center;
    }
    .nav-btn {
      background-color: #95a5a6;
      color: white;
      border: none;
      padding: 10px 20px;
      border-radius: 5px;
      cursor: pointer;
      text-decoration: none;
      display: inline-block;
    }
    .nav-btn:hover {
      background-color: #7f8c8d;
    }
    .nav-btn.active {
      background-color: #3498db;
    }
    .settings-section {
      margin: 30px 0;
      padding: 20px;
      background-color: #ecf0f1;
      border-radius: 5px;
      text-align: left;
    }
    .settings-section h3 {
      margin-top: 0;
      color: #2c3e50;
      border-bottom: 2px solid #3498db;
      padding-bottom: 10px;
    }
    .setting-item {
      margin: 20px 0;
      padding: 15px;
      background-color: white;
      border-radius: 5px;
    }
    .setting-label {
      font-weight: bold;
      color: #2c3e50;
      display: block;
      margin-bottom: 8px;
    }
    .setting-description {
      font-size: 0.85em;
      color: #7f8c8d;
      margin-bottom: 10px;
    }
    input[type="number"] {
      width: 100%;
      padding: 10px;
      border: 2px solid #bdc3c7;
      border-radius: 5px;
      font-size: 1em;
      box-sizing: border-box;
    }
    input[type="number"]:focus {
      outline: none;
      border-color: #3498db;
    }
    .checkbox-container {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    input[type="checkbox"] {
      width: 24px;
      height: 24px;
      cursor: pointer;
    }
    button {
      background-color: #3498db;
      color: white;
      border: none;
      padding: 15px 30px;
      font-size: 1.1em;
      border-radius: 5px;
      cursor: pointer;
      margin: 10px;
      transition: background-color 0.3s;
    }
    button:hover {
      background-color: #2980b9;
    }
    button:active {
      background-color: #21618c;
    }
    .save-btn {
      background-color: #27ae60;
    }
    .save-btn:hover {
      background-color: #229954;
    }
    .reset-btn {
      background-color: #e67e22;
    }
    .reset-btn:hover {
      background-color: #d35400;
    }
    .info-box {
      background-color: #d6eaf8;
      border-left: 4px solid #3498db;
      padding: 15px;
      margin: 20px 0;
      border-radius: 5px;
      text-align: left;
    }
    .success-message {
      background-color: #d5f4e6;
      border-left: 4px solid #27ae60;
      padding: 15px;
      margin: 20px 0;
      border-radius: 5px;
      display: none;
    }
  </style>
  <script>
    function showSuccess() {
      var msg = document.getElementById('successMessage');
      msg.style.display = 'block';
      setTimeout(function() {
        msg.style.display = 'none';
      }, 3000);
    }

    // The page ships static; current values are fetched once on load.
    function loadValues() {
      return fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('defaultCurrent').value = data.setCurrent;
          document.getElementById('maxCurrent').value = data.maxCurrent;
          document.getElementById('minCurrent').value = data.minCurrent;
          document.getElementById('autoStart').checked = data.autoStart;
          document.getElementById('displayInterval').value = data.displayInterval;
          document.getElementById('wifiMode').innerText = data.wifiMode;
          document.getElementById('ip').innerText = data.ip;
        })
        .catch(error => console.error('Error fetching settings:', error));
    }

    function saveSettings(event) {
      event.preventDefault();
      var form = document.getElementById('settingsForm');
      fetch('/saveSettings', { method: 'POST', body: new URLSearchParams(new FormData(form)) })
        .then(loadValues)
        .then(showSuccess)
        .catch(error => console.error('Save failed:', error));
    }

    function resetDefaults() {
      if (confirm('Reset all settings to factory defaults?')) {
        fetch('/resetSettings', { method: 'POST' })
          .then(loadValues)
          .then(showSuccess)
          .catch(error => console.error('Reset failed:', error));
      }
    }

    window.onload = loadValues;
  </script>
</head>
<body>
  <div class="container">
    <h1>Settings</h1>
    
    <div class="nav-buttons">
      <a href="/" class="nav-btn">Dashboard</a>
      <a href="/settings" class="nav-btn active">Settings</a>
    </div>

    <div id="successMessage" class="success-message">
      Settings saved successfully!
    </div>

    <form id="settingsForm" action="/saveSettings" method="POST" onsubmit="saveSettings(event)">
      <div class="settings-section">
        <h3>Current Limits</h3>

        <div class="setting-item">
          <label class="setting-label" for="defaultCurrent">Default Charging Current</label>
          <div class="setting-description">Initial current setting when charger starts (6-32A)</div>
          <input type="number" id="defaultCurrent" name="defaultCurrent"
                 min="6" max="32" step="1" value="8">
        </div>

        <div class="setting-item">
          <label class="setting-label" for="maxCurrent">Maximum Current Limit</label>
          <div class="setting-description">Hardware maximum current limit (6-32A)</div>
          <input type="number" id="maxCurrent" name="maxCurrent"
                 min="6" max="32" step="1" value="32">
        </div>

        <div class="setting-item">
          <label class="setting-label" for="minCurrent">Minimum Current Limit</label>
          <div class="setting-description">Hardware minimum current limit (6-32A)</div>
          <input type="number" id="minCurrent" name="minCurrent"
                 min="6" max="32" step="1" value="6">
        </div>
      </div>

      <div class="settings-section">
        <h3>Charging Behavior</h3>
        
        <div class="setting-item">
          <label class="setting-label">Auto-Start Charging</label>
          <div class="setting-description">Automatically start charging when vehicle is ready (State C)</div>
          <div class="checkbox-container">
            <input type="checkbox" id="autoStart" name="autoStart" value="1">
            <label for="autoStart">Enable automatic charging</label>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <h3>Display Settings</h3>
        
        <div class="setting-item">
          <label class="setting-label" for="displayInterval">Display Update Interval</label>
          <div class="setting-description">How often to update the OLED display (milliseconds)</div>
          <input type="number" id="displayInterval" name="displayInterval"
                 min="100" max="2000" step="100" value="500">
        </div>
      </div>

      <div class="info-box">
        <strong>Note:</strong> All settings on this page are saved to flash memory
        and persist after reboot.
      </div>

      <div>
        <button type="submit" class="save-btn">Save Settings</button>
        <button type="button" class="reset-btn" onclick="resetDefaults()">Reset to Defaults</button>
      </div>
    </form>

    <div style="margin-top: 20px; color: #7f8c8d; font-size: 0.9em;">
      EV Charger v1.0 | <span id="wifiMode">--</span> | IP: <span id="ip">--</span>
    </div>
  </div>
</body>
</html>
)rawliteral";

// Helper function to format uptime
String formatUptime(unsigned long seconds) {
  int days = seconds / 86400;
  int hours = (seconds % 86400) / 3600;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  
  String uptime = "";
  if (days > 0) uptime += String(days) + "d ";
  if (hours > 0) uptime += String(hours) + "h ";
  uptime += String(minutes) + "m ";
  uptime += String(secs) + "s";
  
  return uptime;
}

void sendNoCacheHeaders() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

// Route handler: Main page
void handleRoot() {
  sendNoCacheHeaders();
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

// Route handler: Settings page
void handleSettings() {
  sendNoCacheHeaders();
  server.send_P(200, "text/html", SETTINGS_HTML);
}

// Route handler: Set charging current
void handleSetCurrent() {
  if (server.hasArg("value")) {
    String currentStr = server.arg("value");
    float newCurrent = currentStr.toFloat();

    // Validate current range using configured limits
    if (newCurrent >= minCurrentLimit && newCurrent <= maxCurrentLimit) {
      chargingCurrent = newCurrent;
      saveChargingCurrent();
      noteActivity();

      // Update PWM immediately if in appropriate state
      if (currentVehicleState == STATE_B || currentVehicleState == STATE_C) {
        int ampsPWM = chargingPWM(chargingCurrent);
        PWM_DutyCycle = ampsPWM;
        ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      }

      server.sendHeader("Location", "/");
      server.send(303);
    } else {
      String errorMsg = "Current out of range (" + String((int)minCurrentLimit) + "-" + String((int)maxCurrentLimit) + "A)";
      server.send(400, "text/plain", errorMsg);
    }
  } else {
    server.send(400, "text/plain", "Missing value parameter");
  }
}

// Route handler: Emergency stop.
// This LATCHES - without the latch the state machine simply re-closed the
// contactor on its next 100ms tick, because the vehicle is still sitting in a
// confirmed State C and nothing recorded that a human had said stop.
void handleEmergencyStop() {
  latchFault("Emergency stop");
  server.sendHeader("Location", "/");
  server.send(303);
}

// Route handler: Clear a latched fault
void handleClearFault() {
  clearFault();
  noteActivity();
  server.sendHeader("Location", "/");
  server.send(303);
}

// Route handler: JSON data endpoint (drives both pages)
void handleData() {
  // Built into a fixed stack buffer: this runs every 2 seconds forever, so it
  // must not touch the heap.
  char json[512];
  String ipAddress = isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String uptime = formatUptime(millis() / 1000);

  snprintf(json, sizeof(json),
           "{\"voltage\":%.1f,\"current\":%.2f,\"power\":%.1f,\"energy\":%.2f,"
           "\"sessionEnergy\":%.2f,\"setCurrent\":%d,\"chargerState\":\"%s\","
           "\"vehicleState\":\"%s\",\"uptime\":\"%s\",\"pwm\":%d,"
           "\"wifiMode\":\"Mode: %s\",\"ip\":\"%s\",\"meter\":%s,"
           "\"contactor\":%s,\"fault\":%s,\"faultReason\":\"%s\","
           "\"maxCurrent\":%d,\"minCurrent\":%d,\"autoStart\":%s,"
           "\"displayInterval\":%d}",
           voltage, current, power, energy,
           sessionEnergy, (int)chargingCurrent, getChargerStateName(chargerState).c_str(),
           getVehicleStateName(currentVehicleState).c_str(), uptime.c_str(), PWM_DutyCycle,
           isAPMode ? "Access Point" : "Station", ipAddress.c_str(), pzemOnline ? "true" : "false",
           contactorClosed ? "true" : "false", faultLatched ? "true" : "false", faultReason,
           (int)maxCurrentLimit, (int)minCurrentLimit, autoStartCharging ? "true" : "false",
           displayUpdateInterval);

  sendNoCacheHeaders();
  server.send(200, "application/json", json);
}

// Route handler: Save settings
void handleSaveSettings() {
  // Update max/min limits first so the charging current is validated against
  // the limits that are being saved in this same request.
  if (server.hasArg("maxCurrent")) {
    float newMax = server.arg("maxCurrent").toFloat();
    if (newMax >= MIN_CURRENT && newMax <= MAX_CURRENT) {
      maxCurrentLimit = newMax;
    }
  }

  if (server.hasArg("minCurrent")) {
    float newMin = server.arg("minCurrent").toFloat();
    if (newMin >= MIN_CURRENT && newMin <= MAX_CURRENT) {
      minCurrentLimit = newMin;
    }
  }

  if (minCurrentLimit > maxCurrentLimit) {
    minCurrentLimit = MIN_CURRENT;
    maxCurrentLimit = MAX_CURRENT;
  }

  if (server.hasArg("defaultCurrent")) {
    float newCurrent = server.arg("defaultCurrent").toFloat();
    if (newCurrent >= minCurrentLimit && newCurrent <= maxCurrentLimit) {
      chargingCurrent = newCurrent;
    }
  }

  // Keep the active current inside the (possibly new) limits
  if (chargingCurrent < minCurrentLimit) chargingCurrent = minCurrentLimit;
  if (chargingCurrent > maxCurrentLimit) chargingCurrent = maxCurrentLimit;

  // Update auto-start charging
  autoStartCharging = server.hasArg("autoStart");

  // Update display interval
  if (server.hasArg("displayInterval")) {
    int newInterval = server.arg("displayInterval").toInt();
    if (newInterval >= 100 && newInterval <= 2000) {
      displayUpdateInterval = newInterval;
    }
  }

  saveSettings();
  noteActivity();

  // Update PWM if currently in STATE_B or STATE_C
  if (currentVehicleState == STATE_B || currentVehicleState == STATE_C) {
    int ampsPWM = chargingPWM(chargingCurrent);
    PWM_DutyCycle = ampsPWM;
    ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  }

  server.sendHeader("Location", "/settings");
  server.send(303);
}

// Route handler: Reset settings to defaults
void handleResetSettings() {
  chargingCurrent = DEFAULT_CURRENT;
  maxCurrentLimit = MAX_CURRENT;
  minCurrentLimit = MIN_CURRENT;
  autoStartCharging = true;
  displayUpdateInterval = 500;

  saveSettings();
  noteActivity();

  // Update PWM if needed
  if (currentVehicleState == STATE_B || currentVehicleState == STATE_C) {
    int ampsPWM = chargingPWM(chargingCurrent);
    PWM_DutyCycle = ampsPWM;
    ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  }

  server.sendHeader("Location", "/settings");
  server.send(303);
}

// Route handler: 404 Not Found
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// Setup web server routes.
// Anything that changes charger state is POST-only, so a browser prefetch, a
// stale bookmark or a network scanner walking the LAN cannot trip the charger
// by issuing a plain GET.
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/data", HTTP_GET, handleData);
  server.on("/setCurrent", HTTP_POST, handleSetCurrent);
  server.on("/emergencyStop", HTTP_POST, handleEmergencyStop);
  server.on("/clearFault", HTTP_POST, handleClearFault);
  server.on("/saveSettings", HTTP_POST, handleSaveSettings);
  server.on("/resetSettings", HTTP_POST, handleResetSettings);
  server.onNotFound(handleNotFound);
  server.begin();
}