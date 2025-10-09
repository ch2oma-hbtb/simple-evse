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
ChargerState chargerState = CHARGER_IDLE;

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
bool serialDebugEnabled = true;

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

int PWM_DutyCycle = 0;

// ========================================
// PREFERENCES (FLASH STORAGE) FUNCTIONS
// ========================================

void loadSettings() {
  preferences.begin("evse", false); // Open in read-write mode
  
  // Load charging current from flash, default to DEFAULT_CURRENT if not found
  chargingCurrent = preferences.getFloat("chargeCurrent", DEFAULT_CURRENT);
  
  // Validate the loaded value
  if (chargingCurrent < MIN_CURRENT || chargingCurrent > MAX_CURRENT) {
    Serial.println("Invalid stored current, using default");
    chargingCurrent = DEFAULT_CURRENT;
  } else {
    Serial.print("Loaded charging current from memory: ");
    Serial.print(chargingCurrent);
    Serial.println("A");
  }
  
  preferences.end();
}

void saveChargingCurrent() {
  preferences.begin("evse", false); // Open in read-write mode
  preferences.putFloat("chargeCurrent", chargingCurrent);
  preferences.end();
  
  Serial.print("Saved charging current to memory: ");
  Serial.print(chargingCurrent);
  Serial.println("A");
}

void setup() {
  Serial.begin(115200);
  Serial.println("EV Charger Starting...");
  
  // Load saved settings from flash memory
  loadSettings();
  
  // Initialize pins
  pinMode(CONTACTOR_PIN, OUTPUT);
  digitalWrite(CONTACTOR_PIN, LOW);
  pinMode(CP_SENSE_PIN, INPUT);
  pinMode(CP_PWM_PIN, OUTPUT);
  
  // Initialize PWM for CP signal - UPDATED FOR NEW ESP32 CORE
  if (!ledcAttach(CP_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach PWM to pin!");
    while(1);
  }
  
  // Start with 100% duty cycle (12V)
  PWM_DutyCycle = 1023; // 100% duty cycle for standby
  ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  
  // Initialize I2C for OLED
  Wire.begin();

  // Initialize OLED display
  if (!display.begin(I2C_ADDRESS, true)) {
    Serial.println(F("SH1106 allocation failed"));
    while (1); // Halt if display fails
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("EV Charger v1.0");
  display.println("Initializing...");
  display.display();
  delay(2000);

  // Start WiFi connection in non-blocking mode
  Serial.println("Starting WiFi connection...");
  startWiFiConnection();

  Serial.println("Setup complete!");
}

// Start WiFi connection in non-blocking mode
void startWiFiConnection() {
  // Disconnect any previous connections
  WiFi.disconnect(true);
  delay(100);
  
  // Set WiFi mode to station
  WiFi.mode(WIFI_STA);
  delay(100);
  
  // Optional: Set hostname
  WiFi.setHostname("EV-Charger");
  
  // Set power save mode off for better reliability
  WiFi.setSleep(false);
  
  // Begin WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  wifiConnecting = true;
  wifiStartTime = millis();
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
    Serial.println();
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
    
    setupWebServer();
    return;
  }
  
  // Check timeout
  if (elapsed >= WIFI_TIMEOUT_SECONDS) {
    wifiConnected = false;
    wifiConnecting = false;
    Serial.println();
    Serial.println("WiFi connection timeout!");
    Serial.println("Continuing without WiFi...");
    return;
  }
  
  // Retry every 20 seconds
  if (elapsed > 0 && elapsed % 20 == 0) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 1000) {
      lastRetry = millis();
      Serial.println();
      Serial.println("Retrying WiFi connection...");
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

// Function to read analog with peak detection (from example)
int checkAnalog(int analogPinToTest, int noSamples) {
  int maximum = 0;
  int minimum = 5000;
  int value;
  
  for (int i = 0; i <= noSamples; i++) {
    value = analogRead(analogPinToTest);
    if (value <= minimum) {
      minimum = value;
    }
    if (value >= maximum) {
      maximum = value;
    }
  }
  
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
  
  // Clamp between 10% and 96%
  if (dutyPercent < 10) dutyPercent = 10;
  if (dutyPercent > 96) dutyPercent = 96;
  
  // Convert to 10-bit PWM value (0-1023)
  int pwmValue = (int)((dutyPercent / 100.0) * 1023);
  
  return pwmValue;
}

void loop() {
  unsigned long currentMillis = millis();

  // Check WiFi connection status (non-blocking)
  if (wifiConnecting) {
    checkWiFiConnection();
  }

  // Check WiFi connection status every 30 seconds (if already connected)
  if (currentMillis - lastWiFiCheck >= 30000) {
    lastWiFiCheck = currentMillis;
    
    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      // WiFi was connected but now lost
      Serial.println("WiFi connection lost! Attempting to reconnect...");
      wifiConnected = false;
      // Stop the web server when connection is lost
      server.stop();
    } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
      // WiFi reconnected
      Serial.println("WiFi reconnected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      
      // Restart web server after reconnection
      setupWebServer();
    }
    
    // Attempt to reconnect if disconnected
    if (!wifiConnected && WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.println("Reconnecting to WiFi...");
      wifiConnecting = true;
      wifiStartTime = millis();
    }
  }

  // Handle web server only if WiFi is connected
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  
  // Check vehicle state every 100ms
  if (currentMillis - lastStateCheck >= 100) {
    lastStateCheck = currentMillis;
    checkVehicleState();
    updateChargerState();
  }
  
  // Read PZEM data every 250ms
  if (currentMillis - lastPZEMRead >= PZEM_READ_INTERVAL) {
    lastPZEMRead = currentMillis;
    readPZEMData();
  }
  
  // Update display based on configured interval
  if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }
  
  // Update charging duration
  if (chargerState == CHARGER_CHARGING && contactorClosed) {
    chargingDuration = (currentMillis - chargingStartTime) / 1000;
  }
}

void checkVehicleState() {
  // Use peak detection like in the example
  int cpVoltage = checkAnalog(CP_SENSE_PIN, 100);
  // Serial.println(cpVoltage);
  
  previousVehicleState = currentVehicleState;
  
  // Determine vehicle state based on CP voltage
  if (cpVoltage >= CP_12V_MIN && cpVoltage <= CP_12V_MAX) {
    currentVehicleState = STATE_A; // No vehicle
  } else if (cpVoltage >= CP_9V_MIN && cpVoltage <= CP_9V_MAX) {
    currentVehicleState = STATE_B; // Vehicle connected, not ready
  } else if (cpVoltage >= CP_6V_MIN && cpVoltage <= CP_6V_MAX) {
    currentVehicleState = STATE_C; // Vehicle ready, charging
  } else if (cpVoltage >= CP_3V_MIN && cpVoltage <= CP_3V_MAX) {
    currentVehicleState = STATE_D; // Ventilation required
  } else if (cpVoltage >= CP_0V_MIN && cpVoltage <= CP_0V_MAX) {
    currentVehicleState = STATE_E; // No power
  } else {
    currentVehicleState = STATE_F; // Error
  }
  
  // Debug output
  if (currentVehicleState != previousVehicleState) {
    if (serialDebugEnabled) {
      Serial.print("State changed: ");
      Serial.print(getVehicleStateName(previousVehicleState));
      Serial.print(" -> ");
      Serial.println(getVehicleStateName(currentVehicleState));
      Serial.print("CP Voltage (ADC): ");
      Serial.println(cpVoltage);
    }
  }
}

void updateChargerState() {
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
      PWM_DutyCycle = ampsPWM; // Set PWM for current limit
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      if (serialDebugEnabled) {
        Serial.print("STATE_B - Setting PWM to: ");
        Serial.print(PWM_DutyCycle);
        Serial.print(" for ");
        Serial.print(chargingCurrent);
        Serial.println("A");
      }
      break;
      
    case STATE_C:
      // Vehicle ready to charge
      if (autoStartCharging && chargerState != CHARGER_CHARGING) {
        startCharging();
        chargerState = CHARGER_CHARGING;
      } else if (!autoStartCharging) {
        stopCharging();
        chargerState = CHARGER_CONNECTED;
      }
      PWM_DutyCycle = ampsPWM;
      ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
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

void startCharging() {
  Serial.println("Starting charging...");
  int ampsPWM = chargingPWM(chargingCurrent);
  PWM_DutyCycle = ampsPWM;
  ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  delay(100);
  digitalWrite(CONTACTOR_PIN, HIGH);
  contactorClosed = true;
  chargingStartTime = millis();
  chargingDuration = 0;
  Serial.println("Charging started!");
}

void stopCharging() {
  if (contactorClosed) {
    Serial.println("Stopping charging...");
    digitalWrite(CONTACTOR_PIN, LOW);
    contactorClosed = false;
    Serial.println("Charging stopped!");
  }
}

void readPZEMData() {
  voltage = pzem.voltage();
  current = pzem.current();
  power = pzem.power();
  energy = pzem.energy();
  frequency = pzem.frequency();
  powerFactor = pzem.pf();
  
  // Check for NaN values
  if (isnan(voltage)) voltage = 0;
  if (isnan(current)) current = 0;
  if (isnan(power)) power = 0;
  if (isnan(energy)) energy = 0;
  if (isnan(frequency)) frequency = 0;
  if (isnan(powerFactor)) powerFactor = 0;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Title
  display.println("EV CHARGER");
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SH110X_WHITE);
  
  // State
  display.setCursor(0, 14);
  display.print("State: ");
  display.println(getChargerStateName(chargerState));
  
  // Voltage and Current
  display.setCursor(0, 24);
  display.print("V:");
  display.print(voltage, 1);
  display.print("V I:");
  display.print(current, 2);
  display.println("A");
  
  // Power and PWM info
  display.setCursor(0, 34);
  display.print("P:");
  display.print(power, 1);
  display.print("W PWM:");
  display.println(PWM_DutyCycle);
  
  // Energy
  display.setCursor(0, 44);
  display.print("E:");
  display.print(energy, 2);
  display.print("kWh Set:");
  display.print(chargingCurrent);
  display.println("A");

  // IP Address (always show if WiFi connected) or Charging duration
  display.setCursor(0, 54);
  if (wifiConnecting) {
    // Show WiFi connecting status
    unsigned long elapsed = (millis() - wifiStartTime) / 1000;
    display.print("WiFi: ");
    display.print(elapsed);
    display.print("s...");
  } else if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    display.print("IP: ");
    display.println(WiFi.localIP());
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
    case STATE_A: return "A-No Vehicle";
    case STATE_B: return "B-Connected";
    case STATE_C: return "C-Charging";
    case STATE_D: return "D-Vent Req";
    case STATE_E: return "E-No Power";
    case STATE_F: return "F-EVSE Error";
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

// HTML template for the web interface
const char* getHTMLTemplate() {
  return R"rawliteral(
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
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('voltage').innerText = data.voltage;
          document.getElementById('current').innerText = data.current;
          document.getElementById('power').innerText = data.power;
          document.getElementById('energy').innerText = data.energy;
          document.getElementById('setCurrent').innerText = data.setCurrent;
          document.getElementById('currentSlider').value = data.setCurrent;
          document.getElementById('sliderValue').innerText = data.setCurrent;
          document.getElementById('chargerState').innerText = data.chargerState;
          document.getElementById('vehicleState').innerText = data.vehicleState;
          document.getElementById('uptime').innerText = data.uptime;
          document.getElementById('pwm').innerText = data.pwm;
          
          // Update status class
          let statusDiv = document.getElementById('statusDiv');
          statusDiv.className = 'status';
          if (data.chargerState === 'Charging') {
            statusDiv.className = 'status charging';
          } else if (data.chargerState === 'Error') {
            statusDiv.className = 'status error';
          }
        })
        .catch(error => console.error('Error fetching data:', error));
    }
    
    // Update every 2 seconds
    setInterval(updateData, 2000);
    
    // Initial update
    window.onload = updateData;
  </script>
</head>
<body>
  <div class="container">
    <h1>EV Charger Control</h1>
    
    <div class="nav-buttons">
      <a href="/" class="nav-btn active">Dashboard</a>
      <a href="/settings" class="nav-btn">Settings</a>
    </div>
    
    <div id="statusDiv" class="status %STATUS_CLASS%">
      <h2>Status: <span id="chargerState">%STATE%</span></h2>
      <div>Vehicle State: <span id="vehicleState">%VEHICLE_STATE%</span></div>
    </div>

    <div class="readings-grid">
      <div class="reading-card">
        <div class="reading-label">Voltage</div>
        <div class="reading-value"><span id="voltage">%VOLTAGE%</span> V</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Current</div>
        <div class="reading-value"><span id="current">%CURRENT%</span> A</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Power</div>
        <div class="reading-value"><span id="power">%POWER%</span> W</div>
      </div>
      <div class="reading-card">
        <div class="reading-label">Energy</div>
        <div class="reading-value"><span id="energy">%ENERGY%</span> kWh</div>
      </div>
    </div>

    <div class="control-section">
      <h3>Charging Current Limit</h3>
      <div class="current-display"><span id="setCurrent">%SET_CURRENT%</span> A</div>
      
      <form action="/setCurrent" method="GET">
        <div class="slider-container">
          <input type="range" min="6" max="32" value="%SET_CURRENT%" 
                 class="slider" id="currentSlider" name="value" 
                 oninput="document.getElementById('sliderValue').innerText=this.value">
        </div>
        <div style="font-size: 1.5em; margin: 10px 0;">
          <span id="sliderValue">%SET_CURRENT%</span> A
        </div>
        <button type="submit">Set Current</button>
      </form>

      <div class="quick-buttons">
        <strong>Quick Set:</strong><br>
        <a href="/setCurrent?value=6"><button class="quick-btn">6A</button></a>
        <a href="/setCurrent?value=8"><button class="quick-btn">8A</button></a>
        <a href="/setCurrent?value=10"><button class="quick-btn">10A</button></a>
        <a href="/setCurrent?value=16"><button class="quick-btn">16A</button></a>
        <a href="/setCurrent?value=20"><button class="quick-btn">20A</button></a>
        <a href="/setCurrent?value=32"><button class="quick-btn">32A</button></a>
      </div>
    </div>

    <div>
      <a href="/emergencyStop">
        <button class="emergency-stop" 
                onclick="return confirm('Are you sure you want to emergency stop?')">
          EMERGENCY STOP
        </button>
      </a>
    </div>

    <div style="margin-top: 20px; color: #7f8c8d; font-size: 0.9em;">
      IP: %IP% | Uptime: <span id="uptime">%UPTIME%</span> | PWM: <span id="pwm">%PWM%</span>
    </div>
  </div>
</body>
</html>
)rawliteral";
}

// HTML template for settings page
const char* getSettingsHTMLTemplate() {
  return R"rawliteral(
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
      const msg = document.getElementById('successMessage');
      msg.style.display = 'block';
      setTimeout(() => {
        msg.style.display = 'none';
      }, 3000);
    }
    
    function resetDefaults() {
      if (confirm('Reset all settings to factory defaults?')) {
        window.location.href = '/resetSettings';
      }
    }
    
    // Check if we came from a save operation
    window.onload = function() {
      const urlParams = new URLSearchParams(window.location.search);
      if (urlParams.get('saved') === 'true') {
        showSuccess();
      }
    }
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

    <form action="/saveSettings" method="GET">
      <div class="settings-section">
        <h3>Current Limits</h3>
        
        <div class="setting-item">
          <label class="setting-label" for="defaultCurrent">Default Charging Current</label>
          <div class="setting-description">Initial current setting when charger starts (6-32A)</div>
          <input type="number" id="defaultCurrent" name="defaultCurrent" 
                 min="6" max="32" step="1" value="%DEFAULT_CURRENT%">
        </div>

        <div class="setting-item">
          <label class="setting-label" for="maxCurrent">Maximum Current Limit</label>
          <div class="setting-description">Hardware maximum current limit (6-32A)</div>
          <input type="number" id="maxCurrent" name="maxCurrent" 
                 min="6" max="32" step="1" value="%MAX_CURRENT%">
        </div>

        <div class="setting-item">
          <label class="setting-label" for="minCurrent">Minimum Current Limit</label>
          <div class="setting-description">Hardware minimum current limit (6-32A)</div>
          <input type="number" id="minCurrent" name="minCurrent" 
                 min="6" max="32" step="1" value="%MIN_CURRENT%">
        </div>
      </div>

      <div class="settings-section">
        <h3>Charging Behavior</h3>
        
        <div class="setting-item">
          <label class="setting-label">Auto-Start Charging</label>
          <div class="setting-description">Automatically start charging when vehicle is ready (State C)</div>
          <div class="checkbox-container">
            <input type="checkbox" id="autoStart" name="autoStart" value="1" %AUTO_START_CHECKED%>
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
                 min="100" max="2000" step="100" value="%DISPLAY_INTERVAL%">
        </div>
      </div>

      <div class="settings-section">
        <h3>Debug Options</h3>
        
        <div class="setting-item">
          <label class="setting-label">Serial Debug Output</label>
          <div class="setting-description">Enable detailed debug messages on Serial Monitor</div>
          <div class="checkbox-container">
            <input type="checkbox" id="serialDebug" name="serialDebug" value="1" %SERIAL_DEBUG_CHECKED%>
            <label for="serialDebug">Enable debug messages</label>
          </div>
        </div>
      </div>

      <div class="info-box">
        <strong>Note:</strong> Charging current settings are automatically saved to flash memory 
        and will persist after reboot. Other settings (max/min limits, auto-start, display interval, 
        and debug mode) are applied immediately but will reset to defaults on reboot.
      </div>

      <div>
        <button type="submit" class="save-btn">Save Settings</button>
        <button type="button" class="reset-btn" onclick="resetDefaults()">Reset to Defaults</button>
      </div>
    </form>

    <div style="margin-top: 20px; color: #7f8c8d; font-size: 0.9em;">
      EV Charger v1.0 | IP: %IP%
    </div>
  </div>
</body>
</html>
)rawliteral";
}

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

// Generate HTML page with current values (only called on first load)
String getHTML() {
  String html = String(getHTMLTemplate());
  
  // Determine status class
  String statusClass = "status";
  if (chargerState == CHARGER_CHARGING) {
    statusClass = "status charging";
  } else if (chargerState == CHARGER_ERROR) {
    statusClass = "status error";
  }
  
  // Replace all placeholders - do this more efficiently
  html.replace("%STATUS_CLASS%", statusClass);
  html.replace("%STATE%", getChargerStateName(chargerState));
  html.replace("%VEHICLE_STATE%", getVehicleStateName(currentVehicleState));
  html.replace("%VOLTAGE%", String(voltage, 1));
  html.replace("%CURRENT%", String(current, 2));
  html.replace("%POWER%", String(power, 1));
  html.replace("%ENERGY%", String(energy, 2));
  html.replace("%SET_CURRENT%", String((int)chargingCurrent));
  html.replace("%IP%", WiFi.localIP().toString());
  html.replace("%UPTIME%", formatUptime(millis() / 1000));
  html.replace("%PWM%", String(PWM_DutyCycle));
  
  return html;
}

// Route handler: Main page
void handleRoot() {
  // Add cache control headers to improve performance
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", getHTML());
}

// Route handler: Set charging current
void handleSetCurrent() {
  if (server.hasArg("value")) {
    String currentStr = server.arg("value");
    float newCurrent = currentStr.toFloat();
    
    // Validate current range using configured limits
    if (newCurrent >= minCurrentLimit && newCurrent <= maxCurrentLimit) {
      chargingCurrent = newCurrent;
      
      // Save to flash memory
      saveChargingCurrent();
      
      if (serialDebugEnabled) {
        Serial.print("Web: Current set to ");
        Serial.print(chargingCurrent);
        Serial.println("A");
      }
      
      // Update PWM immediately if in appropriate state
      if (currentVehicleState == STATE_B || currentVehicleState == STATE_C) {
        int ampsPWM = chargingPWM(chargingCurrent);
        PWM_DutyCycle = ampsPWM;
        ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
      }
      
      // Redirect back to main page
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

// Route handler: Emergency stop
void handleEmergencyStop() {
  Serial.println("Web: Emergency stop activated!");
  stopCharging();
  chargerState = CHARGER_ERROR;
  
  // Redirect back to main page
  server.sendHeader("Location", "/");
  server.send(303);
}

// Route handler: JSON data endpoint (optimized for AJAX updates)
void handleData() {
  // Use more efficient string building
  String json = "{";
  json.reserve(256); // Pre-allocate memory to reduce fragmentation
  json += "\"voltage\":";
  json += String(voltage, 1);
  json += ",\"current\":";
  json += String(current, 2);
  json += ",\"power\":";
  json += String(power, 1);
  json += ",\"energy\":";
  json += String(energy, 2);
  json += ",\"setCurrent\":";
  json += String((int)chargingCurrent);
  json += ",\"chargerState\":\"";
  json += getChargerStateName(chargerState);
  json += "\",\"vehicleState\":\"";
  json += getVehicleStateName(currentVehicleState);
  json += "\",\"uptime\":\"";
  json += formatUptime(millis() / 1000);
  json += "\",\"pwm\":";
  json += String(PWM_DutyCycle);
  json += "}";
  
  // Add headers to prevent caching of dynamic data
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "application/json", json);
}

// Generate settings HTML page
String getSettingsHTML() {
  String html = String(getSettingsHTMLTemplate());
  
  // Replace all placeholders
  html.replace("%DEFAULT_CURRENT%", String((int)chargingCurrent));
  html.replace("%MAX_CURRENT%", String((int)maxCurrentLimit));
  html.replace("%MIN_CURRENT%", String((int)minCurrentLimit));
  html.replace("%AUTO_START_CHECKED%", autoStartCharging ? "checked" : "");
  html.replace("%DISPLAY_INTERVAL%", String(displayUpdateInterval));
  html.replace("%SERIAL_DEBUG_CHECKED%", serialDebugEnabled ? "checked" : "");
  html.replace("%IP%", WiFi.localIP().toString());
  
  return html;
}

// Route handler: Settings page
void handleSettings() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", getSettingsHTML());
}

// Route handler: Save settings
void handleSaveSettings() {
  bool updated = false;
  
  // Update default charging current
  if (server.hasArg("defaultCurrent")) {
    float newCurrent = server.arg("defaultCurrent").toFloat();
    if (newCurrent >= 6 && newCurrent <= 32) {
      chargingCurrent = newCurrent;
      
      // Save to flash memory
      saveChargingCurrent();
      
      updated = true;
      if (serialDebugEnabled) {
        Serial.print("Settings: Default current set to ");
        Serial.print(chargingCurrent);
        Serial.println("A");
      }
    }
  }
  
  // Update max current limit
  if (server.hasArg("maxCurrent")) {
    float newMax = server.arg("maxCurrent").toFloat();
    if (newMax >= 6 && newMax <= 32) {
      maxCurrentLimit = newMax;
      updated = true;
      if (serialDebugEnabled) {
        Serial.print("Settings: Max current limit set to ");
        Serial.print(maxCurrentLimit);
        Serial.println("A");
      }
    }
  }
  
  // Update min current limit
  if (server.hasArg("minCurrent")) {
    float newMin = server.arg("minCurrent").toFloat();
    if (newMin >= 6 && newMin <= 32) {
      minCurrentLimit = newMin;
      updated = true;
      if (serialDebugEnabled) {
        Serial.print("Settings: Min current limit set to ");
        Serial.print(minCurrentLimit);
        Serial.println("A");
      }
    }
  }
  
  // Update auto-start charging
  autoStartCharging = server.hasArg("autoStart");
  if (serialDebugEnabled) {
    Serial.print("Settings: Auto-start charging ");
    Serial.println(autoStartCharging ? "enabled" : "disabled");
  }
  
  // Update display interval
  if (server.hasArg("displayInterval")) {
    int newInterval = server.arg("displayInterval").toInt();
    if (newInterval >= 100 && newInterval <= 2000) {
      displayUpdateInterval = newInterval;
      updated = true;
      if (serialDebugEnabled) {
        Serial.print("Settings: Display update interval set to ");
        Serial.print(displayUpdateInterval);
        Serial.println("ms");
      }
    }
  }
  
  // Update serial debug
  bool oldDebug = serialDebugEnabled;
  serialDebugEnabled = server.hasArg("serialDebug");
  if (oldDebug != serialDebugEnabled) {
    Serial.print("Settings: Serial debug ");
    Serial.println(serialDebugEnabled ? "enabled" : "disabled");
  }
  
  // Update PWM if currently in STATE_B or STATE_C
  if (updated && (currentVehicleState == STATE_B || currentVehicleState == STATE_C)) {
    int ampsPWM = chargingPWM(chargingCurrent);
    PWM_DutyCycle = ampsPWM;
    ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  }
  
  // Redirect back to settings page with success parameter
  server.sendHeader("Location", "/settings?saved=true");
  server.send(303);
}

// Route handler: Reset settings to defaults
void handleResetSettings() {
  chargingCurrent = DEFAULT_CURRENT;
  maxCurrentLimit = MAX_CURRENT;
  minCurrentLimit = MIN_CURRENT;
  autoStartCharging = true;
  displayUpdateInterval = 500;
  serialDebugEnabled = true;
  
  // Save default current to flash memory
  saveChargingCurrent();
  
  Serial.println("Settings: Reset to factory defaults");
  
  // Update PWM if needed
  if (currentVehicleState == STATE_B || currentVehicleState == STATE_C) {
    int ampsPWM = chargingPWM(chargingCurrent);
    PWM_DutyCycle = ampsPWM;
    ledcWrite(CP_PWM_PIN, PWM_DutyCycle);
  }
  
  // Redirect back to settings page with success parameter
  server.sendHeader("Location", "/settings?saved=true");
  server.send(303);
}

// Route handler: 404 Not Found
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// Setup web server routes
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/setCurrent", HTTP_GET, handleSetCurrent);
  server.on("/emergencyStop", HTTP_GET, handleEmergencyStop);
  server.on("/data", HTTP_GET, handleData);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/saveSettings", HTTP_GET, handleSaveSettings);
  server.on("/resetSettings", HTTP_GET, handleResetSettings);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Web server started!");
}