#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// PIN CONFIGURATION
// ========================================

// Control Pilot (CP) pins
#define CP_SENSE_PIN    A0   // Analog pin for CP voltage sensing
#define CP_PWM_PIN      D10  // PWM pin for CP signal generation

// Power control
#define CONTACTOR_PIN   D3   // Digital pin for contactor control

// PZEM-004T Energy Monitor pins
#define PZEM_RX_PIN     20   // PZEM RX (connect to PZEM TX)
#define PZEM_TX_PIN     21   // PZEM TX (connect to PZEM RX)

// ========================================
// OLED DISPLAY SETTINGS
// ========================================

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define I2C_ADDRESS     0x3C

// ========================================
// PWM SETTINGS FOR CP SIGNAL
// ========================================

#define PWM_FREQ        1000  // 1kHz PWM frequency
#define PWM_RESOLUTION  10    // 10-bit resolution (0-1023)

// ========================================
// CHARGING CURRENT LIMITS (Amps)
// ========================================

#define MAX_CURRENT     32    // Maximum charging current
#define MIN_CURRENT     6     // Minimum charging current
#define DEFAULT_CURRENT 8     // Default startup current

// PWM Duty Cycle Adjustment
// Adjust this value if the measured duty cycle differs from expected
// Positive values increase duty cycle, negative values decrease it
#define DUTY_CYCLE_ADJUSTMENT 3.0  // Percentage adjustment (+2%)

// ========================================
// CP VOLTAGE THRESHOLDS (ADC values after voltage divider)
// ========================================

// STATE_A - No vehicle connected (~12V)
#define CP_12V_MIN      4000
#define CP_12V_MAX      4095

// STATE_B - Vehicle connected, not ready (~9V)
#define CP_9V_MIN       3700
#define CP_9V_MAX       3999

// STATE_C - Vehicle ready, charging (~6V)
#define CP_6V_MIN       3000
#define CP_6V_MAX       3699

// STATE_D - Ventilation required (~3V)
#define CP_3V_MIN       2800
#define CP_3V_MAX       2999

// STATE_E - No power/error (~0V)
#define CP_0V_MIN       2000
#define CP_0V_MAX       2799

// ========================================
// WIFI CONFIGURATION
// ========================================

// STATION MODE (Connect to existing WiFi network)
// IMPORTANT: Verify these settings!
// - SSID is case-sensitive
// - Password must be exact
// - Router must support 2.4GHz (ESP32 doesn't support 5GHz)
// - Check router security settings (WPA2 recommended)

const char* WIFI_SSID = "Pond";
const char* WIFI_PASSWORD = "84383972Aa";
const int WIFI_TIMEOUT_SECONDS = 30;  // WiFi connection timeout before switching to AP mode

// ACCESS POINT MODE (Fallback hotspot if WiFi connection fails)
// This creates a WiFi hotspot that you can connect to directly
const char* AP_SSID = "EVSE-Charger";
const char* AP_PASSWORD = "evse1234";  // Minimum 8 characters for WPA2
const IPAddress AP_IP(192, 168, 4, 1);    // AP IP address
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// WiFi troubleshooting tips:
// 1. Double-check SSID and password
// 2. Ensure router is on 2.4GHz band
// 3. Check if MAC filtering is enabled on router
// 4. Try moving closer to the router
// 5. Verify router is not at maximum client limit
// 6. If WiFi fails, charger will create hotspot (AP mode) automatically

// ========================================
// WEB SERVER CONFIGURATION
// ========================================

#define WEB_SERVER_PORT 80

// ========================================
// TIMING CONFIGURATION (milliseconds)
// ========================================

#define STATE_CHECK_INTERVAL    100   // How often to check vehicle state
#define DISPLAY_UPDATE_INTERVAL 1000   // How often to update OLED display
#define PZEM_READ_INTERVAL      250   // How often to read PZEM data

// Safety delays to prevent accidental contactor closure
#define STATE_CHANGE_DELAY      2000  // Wait 2 seconds after state change before allowing actions
#define STATE_C_READY_DELAY     3000  // Wait 3 seconds in STATE_C before closing contactor

#endif // CONFIG_H
