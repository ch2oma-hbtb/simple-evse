# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a single-phase J1772 EV charging station (EVSE) running on a **Seeed XIAO ESP32C3** (ESP32-C3, RISC-V, single core, 4 MB flash). The whole program is one Arduino sketch: `EVSE/EVSE.ino` plus `EVSE/config.h`. There are no tests, no build scripts, and no dependency manifest — libraries come from the globally installed Arduino libraries directory.

The board is **not** an ESP32-S3 board, despite the `Dx`/`Ax` pin names in `config.h` suggesting an Arduino-style board. Confirm before assuming: the FQBN and build options are recorded in the running firmware image itself and can be read back with `esptool read_flash 0x10000 0x180000 out.bin` followed by `strings out.bin | grep esp32:esp32`.

## Build & upload

The exact FQBN the board is programmed with, options included:

```
esp32:esp32:XIAO_ESP32C3:UploadSpeed=921600,CDCOnBoot=default,CPUFreq=160,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=default,DebugLevel=none,EraseFlash=none
```

Either build through **Arduino IDE.app** (open `EVSE/EVSE.ino`, board *XIAO_ESP32C3*), or with `arduino-cli` if it is installed — it reuses the IDE's already-installed core and libraries once `directories.data` points at `~/Library/Arduino15` and `directories.user` at `~/Documents/Arduino`:

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:XIAO_ESP32C3 EVSE
```

```bash
arduino-cli compile --upload --fqbn esp32:esp32:XIAO_ESP32C3 -p /dev/cu.usbmodem14401 EVSE
```

Upload goes over the C3's built-in USB-Serial-JTAG via esptool; no button press or DFU mode is needed.

The sketch folder name must stay `EVSE` to match `EVSE.ino` — Arduino refuses to open a sketch whose folder name differs.

Toolchain in use: `esp32:esp32` core **3.3.3** (this is why `ledcAttach(pin, freq, res)` — the 3.x LEDC API — is used rather than `ledcSetup`/`ledcAttachPin`).

**Watch the flash budget**: the sketch occupies ~81% of the 1.31 MB app partition under the `default` 4 MB partition scheme. There is not a lot of headroom left for new features.

Required libraries (already present in `~/Documents/Arduino/libraries`): `Adafruit_SH110X`, `Adafruit_GFX_Library`, `Adafruit_BusIO`, `PZEM004Tv30`.

Serial debugging is disabled — `Serial.begin()` in `setup()` and the `Serial.println` in `checkVehicleState()` are commented out. Uncomment both when diagnosing CP readings; the raw ADC value is the number you almost always need.

## Architecture

Single-threaded cooperative `loop()`, driven by `millis()` deltas — no RTOS tasks, no interrupts. Four cadences: vehicle state `STATE_CHECK_INTERVAL` (100 ms), PZEM read `PZEM_READ_INTERVAL` (1 s, backing off to `PZEM_RETRY_INTERVAL` once the meter stops answering — each failed read blocks ~100 ms inside the library), display `displayUpdateInterval` (default 500 ms), WiFi health `WIFI_CHECK_INTERVAL` (30 s). Everything else (web server) runs via `server.handleClient()` on every pass. **Nothing in `loop()` may block**; see the WiFi section.

### The two state machines

`VehicleState` (A–F) is the J1772 pilot state read from hardware. `ChargerState` (idle/connected/charging/error/finished) is what the EVSE decides to do about it. `checkVehicleState()` produces the former; `updateChargerState()` consumes it and is the only place the contactor is *closed* and pilot duty is set. The contactor can be *opened* from three places — `updateChargerState()`, the fast trip in `checkVehicleState()`, and `latchFault()` — all of them through `stopCharging()`, which is the single point that records `contactorOpenTime` and resets the over-current timers.

### CP sensing, and the asymmetry between starting and stopping

`checkAnalog()` samples `CP_SENSE_PIN` for a fixed **time window** (`CP_SAMPLE_WINDOW_US`, 3 ms ≈ 3 pilot periods) and returns the **maximum**, because the control pilot is a 1 kHz square wave whose positive peak encodes vehicle state. The window is a duration rather than a sample count on purpose: a count only spans a full period if `analogRead()` happens to be fast enough, and missing the peak once looks exactly like the vehicle changing state. That peak is compared against the `CP_*_MIN/MAX` bands in `config.h`, which are **raw 12-bit ADC counts, not volts**; they encode the specific resistor divider/offset network on the board and must be re-measured if the analog front end changes.

**Starting a charge is debounced; stopping is not.** These are deliberately separate paths:

- *Starting*: a newly detected state becomes `pendingState`, and only after `STATE_CHANGE_DELAY` (2 s) of continuously reading the same state does it become `confirmedVehicleState`. `updateChargerState()` returns immediately when `!stateConfirmed`, so nothing actuates during the wait. `startCharging()` re-checks the fault latch, `stateConfirmed`, `currentVehicleState == STATE_C`, and the re-close hold-off before closing the contactor, and returns `false` if it declines — callers must not assume it closed.
- *Stopping*: `checkVehicleState()` counts consecutive raw samples that are not State C and opens the contactor once `CONTACTOR_TRIP_SAMPLES` (2, so ~200 ms) is reached, without waiting for confirmation. Routing a stop through the 2 s debounce would leave the cable energised for two full seconds after the connector is pulled.

Do not "simplify" these back into one symmetric debounce.

`CONTACTOR_RECLOSE_DELAY` (3 s) keeps a glitch from chattering the contactor: once open, it cannot re-close until that elapses.

### Faults

`latchFault(reason)` opens the contactor and blocks all charging until cleared, holding a steady +12 V pilot so the vehicle sees "present but unavailable" rather than an error. It is used only for **emergency stop** (operator intent) and **over-current** (needs a human) — conditions that must not silently resolve themselves. Transient pilot faults (states D/E/F) deliberately do *not* latch; they stop the charge and recover on their own, because the confirmation delay already guards the restart and nuisance latches on a charger left plugged in overnight are their own hazard. A latch clears when the vehicle is unplugged (confirmed State A) or via `POST /clearFault`.

Over-current compares the PZEM's measured current against what the pilot advertises: `OVERCURRENT_SOFT_RATIO` for `OVERCURRENT_SOFT_TIME`, or `OVERCURRENT_HARD_RATIO` for `OVERCURRENT_HARD_TIME`. Protection is inactive when the meter is offline — there is no measurement to judge, and the firmware does not stop charging for a meter dropout.

### Current → PWM

`chargingPWM(amps)` implements the J1772 duty rule `duty% = amps / 0.6`, adds `DUTY_CYCLE_ADJUSTMENT` (a per-hardware calibration fudge in `config.h`, currently +3%), clamps to 10–96%, and scales to the 10-bit range. `1023` (100% duty, steady +12 V) is the standby/no-vehicle signal, and is what STATE_A and STATE_E emit.

### WiFi: STA with AP fallback

Connection is non-blocking during startup (`startWiFiConnection` → polled by `checkWiFiConnection`). After `WIFI_TIMEOUT_SECONDS` the unit falls back to a `EVSE-Charger` softAP at 192.168.4.1. `maintainWiFi()` runs every pass and handles both directions on a `WIFI_CHECK_INTERVAL` cadence — dropping to AP when the station link dies, and retrying the station link while in AP mode. `setupWebServer()` is re-invoked after every successful (re)connection; `server.stop()` precedes each mode switch.

The AP→STA retry is **split across loop passes**: it starts an `WIFI_AP_STA` attempt, then a later pass either completes the switch or gives up after `STA_RETRY_WINDOW`. Keep it that way — the earlier version blocked the whole loop for 5 s every 30 s, during which vehicle state was not being read at all.

`loop()` is watchdogged (`enableLoopWDT()` / `feedLoopWDT()`), so any new blocking work is a reset risk rather than a silent stall. A reset is safe: `CONTACTOR_PIN` comes up low.

### Web UI

Both pages are **fully static** `R"rawliteral(...)"` constants in flash (`DASHBOARD_HTML`, `SETTINGS_HTML`), streamed with `server.send_P()`. There is no templating and no per-request `String` — every dynamic value arrives through `/data`, which the dashboard polls every 2 s and the settings page fetches once on load. `handleData()` builds its JSON with `snprintf` into a stack buffer for the same reason. This matters because the unit stays powered for months; the previous template-substitution approach copied ~8 KB to the heap and reallocated it a dozen times per page load.

Adding a UI field means: the static HTML, the `/data` JSON in `handleData()`, and the JS that consumes it. Do not reintroduce `%PLACEHOLDER%` substitution.

Everything that changes charger state is **POST-only** (`/setCurrent`, `/emergencyStop`, `/clearFault`, `/saveSettings`, `/resetSettings`) so a prefetch, bookmark, or LAN scanner cannot trip the charger with a GET. ESP32's `WebServer` merges URL query args and urlencoded bodies, so both `POST /setCurrent?value=16` and a form body work.

The dashboard's 2 s poll will not overwrite the current slider within 10 s of the user touching it (`sliderTouched`).

### Persistence

All user settings survive reboot in NVS namespace `evse`: `chargeCurrent`, `maxCurrent`, `minCurrent`, `autoStart`, `dispInterval`, via `loadSettings()`/`saveSettings()`. `loadSettings()` re-validates everything it reads — a corrupt or stale value must never widen the current limits past what the hardware is rated for, so bounds are re-checked against `MIN_CURRENT`/`MAX_CURRENT` rather than trusted.

## Gotchas

**Pin names in `config.h` mix styles, but resolve consistently on this board.** The `XIAO_ESP32C3` variant does no pin remapping — its `Dx`/`Ax` names are plain aliases — so both the named and raw-integer pins are literal GPIO numbers:

| Constant | Name | GPIO |
|---|---|---|
| `CP_SENSE_PIN` | `A0` | 2 (ADC1_CH2) |
| `CONTACTOR_PIN` | `D3` | 5 |
| `CP_PWM_PIN` | `D10` | 10 |
| `PZEM_RX_PIN` / `PZEM_TX_PIN` | raw `20` / `21` | 20 / 21 (UART0) |
| OLED | `SDA` / `SCL` | 6 / 7 |

Note this reasoning does not transfer between boards: on an Arduino-style variant with `BOARD_HAS_PIN_REMAP` the same raw `20`/`21` would be remapped to different GPIOs. Re-derive the table from the variant's `pins_arduino.h` if the board ever changes.

**Missing display is tolerated, missing PWM is fatal.** A failed `display.begin()` just sets `displayAvailable = false` and the unit runs headless; a failed `ledcAttach()` halts in `while(1)`.

**The OLED sleeps.** After `DISPLAY_SLEEP_TIMEOUT` with no state change and nothing charging, the panel is blanked and powered down; `noteActivity()` wakes it. Call `noteActivity()` from anything a user should see.

**No vehicle diode check.** `checkAnalog()` only returns the positive peak, so the negative half of the pilot is never inspected and a missing or shorted EV diode cannot be detected. That would need hardware support for sensing the negative half.

**`config.h` holds plaintext WiFi credentials** and is committed to the repository. Don't echo them into output, logs, or commit messages.

## Verifying changes

There are no tests, so compile every non-trivial edit (see Build & upload for the command) and check it still fits the app partition.

The web UI can be exercised without hardware: extract `DASHBOARD_HTML`/`SETTINGS_HTML` from the sketch with a regex on the `R"rawliteral(...)"` blocks and serve them from a local HTTP server alongside a mock `/data` endpoint that also accepts the POST routes. That catches the failure mode this design is most prone to — JS referencing an element id or JSON key that the static HTML or `handleData()` does not actually provide.
