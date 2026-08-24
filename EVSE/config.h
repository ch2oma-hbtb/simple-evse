#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// PIN CONFIGURATION
// ========================================

// Board: Seeed XIAO ESP32C3. Its variant does no pin remapping, so the Dx/Ax
// names below are plain aliases for GPIO numbers and the raw integers used for
// the PZEM are literal GPIOs:
//   A0 -> GPIO2 (ADC1_CH2)    D3 -> GPIO5    D10 -> GPIO10
//   PZEM 20/21 -> GPIO20/21 (UART0)    OLED SDA/SCL -> GPIO6/GPIO7
// This does not hold on Arduino-style boards that define BOARD_HAS_PIN_REMAP,
// where raw pin numbers are remapped -- re-derive from the variant's
// pins_arduino.h if the board ever changes.

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

// PZEM polling. The library blocks for up to 100ms when the meter does not
// answer, so polling fast is expensive when the meter is missing or unpowered.
#define PZEM_READ_INTERVAL      1000  // How often to read PZEM data
#define PZEM_FAIL_LIMIT         5     // Consecutive failed reads before the meter is declared offline
#define PZEM_RETRY_INTERVAL     10000 // Polling interval once the meter is offline

// WiFi link maintenance (all non-blocking)
#define WIFI_CHECK_INTERVAL     30000 // How often to inspect / repair the WiFi link
#define STA_RETRY_WINDOW        8000  // How long an AP-mode retry waits for the station link

// State change confirmation delay.
// A newly detected state must be read continuously for this long before it is
// confirmed, which keeps voltage spikes and a partially seated connector from
// starting a charge. This gates STARTING only -- stopping is always immediate,
// see the raw-reading check in checkVehicleState().
#define STATE_CHANGE_DELAY      2000  // Wait 2 seconds for any state change confirmation

// ========================================
// SAFETY
// ========================================

// Contactor re-close hold-off. Once the contactor opens for any reason it stays
// open at least this long, so a single noisy reading cannot make it chatter.
#define CONTACTOR_RECLOSE_DELAY 3000

// CP sampling window, in microseconds. The pilot is a 1kHz square wave and the
// state is encoded in its positive peak, so the sampler must span several full
// periods to be sure of catching that peak. Expressed as a time window rather
// than a sample count so it does not depend on how fast analogRead() happens
// to be. 3000us = 3 periods.
#define CP_SAMPLE_WINDOW_US     3000

// How many consecutive out-of-State-C samples force the contactor open.
// At STATE_CHECK_INTERVAL (100ms) per sample, 2 means the cable is de-energised
// within ~200ms of the pilot changing, while a single bad reading is ignored.
#define CONTACTOR_TRIP_SAMPLES  2

// Over-current protection: measured current (PZEM) vs. the current the pilot
// advertises to the vehicle. MARGIN absorbs meter tolerance at low currents.
#define OVERCURRENT_MARGIN      1.0   // Amps added on top of both thresholds
#define OVERCURRENT_SOFT_RATIO  1.10  // 110% of the advertised current...
#define OVERCURRENT_SOFT_TIME   5000  // ...must persist this long to trip
#define OVERCURRENT_HARD_RATIO  1.50  // 150% trips quickly
#define OVERCURRENT_HARD_TIME   1000

// ========================================
// DISPLAY POWER SAVING
// ========================================

// Blank the OLED after this long without a state change while idle (0 disables).
// It wakes on any vehicle/charger state change. Saves panel life and a little heat.
#define DISPLAY_SLEEP_TIMEOUT   600000  // 10 minutes

#endif // CONFIG_H
