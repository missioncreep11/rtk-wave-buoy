# RTK GPS Ocean Wave and Tide Measurements

A centimeter-level precision GPS data logging system for oceanographic applications, combining RTK GPS positioning with IMU sensor data collection. Developed at UCSD Scripps Institution of Oceanography and the Department of Mechnical Engineering both at U.C. San Diego.

## Branch: `Base+PowerLog` (vs `main`)

This branch extends `main` with a refactored ESP32 buoy firmware, real-time power logging, and more reliable cellular NTRIP on the SIM7000. Use this branch for field buoys that include the INA228 on the Qwiic bus.

### Summary of changes

| Area | `main` | `Base+PowerLog` |
|------|--------|-----------------|
| **ESP32 sketch layout** | `esp32/legacy/buoy_combo.ino` + `buoy_combo.h` | `esp32/buoy_combo/` (active sketch); legacy sketches under `esp32/legacy/` |
| **Power monitoring** | None | **Adafruit INA228** on Qwiic (I2C); periodic `[PWR]` current, bus voltage, and power |
| **Modem / NTRIP** | Stock `Botletics_modem_LTE` + `TCPconnect()` | **`BuoyModem`** — plain `AT+CIP*` TCP (avoids SSL path that breaks NTRIP port 2101) |
| **LTE registration** | `CFUN=1` + Hologram APN only | **`configureNetwork()`** — LTE CAT-M, band 12 (US AT&T/T-Mobile), `CGATT`, auto operator search, boot diagnostics |
| **GNSS (ZED-F9P)** | Basic UART `begin()` | Multi-baud search, **RTCM3 on UART1**, config saved to flash |
| **Status output** | Minimal | `[NET]` uses **CGREG** + CSQ; `[GPS]` fix/RTK/sats every 5 s; `[RTCM]` throughput; modem `[DIAG]` if registration stalls |
| **Bench / USB power** | — | `[PWR]` notes when INA228 is not on the active battery rail (expected on USB) |
| **Sample UBX logs** | — | Example `dataLog00014`–`00024` `.ubx` / parsed CSV in `ubx_parsers/` |

### New or improved behavior

- **Power logging:** INA228 on SDA 23 / SCL 22 (ESP32 Thing Plus Qwiic). Logs every 5 s when the battery rail is energized; skips misleading readings on USB bench power.
- **Reliable NTRIP:** `BuoyModem::tcpConnectPlain()` / `tcpSendPlain()` implement the SIM7000 CIP stack for non-SSL casters; validates caster HTTP response before marking NTRIP connected.
- **Hologram / CAT-M:** On boot, modem is held in LTE-only CAT-M mode with band **12** by default (`LTE_CATM_BAND` in `buoy_combo.h`; use **13** for Verizon).
- **Registration debug:** Boot prints CPIN, CFUN, CREG, CGREG, CSQ, CGATT, COPS, CNACT; repeats diagnostics every 30 s while CGREG is not registered.
- **Graceful shutdown:** GPIO 0 button — closes NTRIP, disables GPRS, powers down modem, light sleep (Bluetooth stays on).

### Libraries (additional on this branch)

- **Adafruit INA228** (Arduino Library Manager)

---

## Overview

This system achieves centimeter-level positioning accuracy by receiving Real-Time Kinematic (RTK) corrections via the NTRIP protocol over cellular or WiFi networks. It has three main components:

1. **SparkFun ZED-F9P** — High-precision RTK GNSS receiver
2. **ESP32 + SIM7000 LTE Module** — Handles cellular connectivity and streams NTRIP corrections to the ZED-F9P
3. **OpenLog Artemis (OLA)** — Logs high-precision GPS data and IMU measurements to SD card

---

## Hardware

- SparkFun ZED-F9P-02B-00 (RTK GNSS receiver)
- Botletics SIM7000 LTE Shield
- SparkFun OpenLog Artemis (with built-in ICM-20948 IMU + AK09916 magnetometer)
- SparkFun ESP32 Thing Plus
- Adafruit INA228 breakout (Qwiic, for buoy battery/modem power logging on ESP32)
- u-blox ANN-MB1-00-00 antenna with grounding plate
- LiPo Battery 3.7V 6000mAh (for OpenLog Artemis + GPS)
- LiPo Battery 3.7V 850mAh (for ESP32 + cellular modem)
- Hologram SIM Card
- MicroSD Card (FAT32 formatted)

### Wiring

- ZED-F9P → OpenLog Artemis via Qwiic/I2C (address 0x42)
- ZED-F9P → ESP32 via serial UART (GPIO 12 TX / 27 RX) for RTCM correction injection
- INA228 → ESP32 Qwiic (I2C) for power monitoring
- LTE antenna → SIM7000 (required for cellular registration)
- GNSS antenna → ZED-F9P antenna port

---

## Software Setup

### Arduino IDE — Board Packages

**SparkFun Apollo3 Boards** by SparkFun Electronics
- Must be version **2.2.1** (other versions may fail to compile)
- Add this URL under *File → Preferences → Additional Board Manager URLs* if the package doesn't appear:
  ```
  https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
  ```

**ESP32** by Espressif Systems

### Arduino IDE — Partition Scheme

The `esp32_rtk_wifi` sketch uses both WiFi and BLE, which together exceed the default 1.28MB app partition. Before compiling, set:

`Tools → Partition Scheme → No OTA (Large APP)`

This gives ~2MB for the app. This must be set each time you use a new machine.

### Arduino IDE — Libraries

Install from the Arduino Library Manager:

- BotleticsSIM7000 by Botletics
- Adafruit INA228 by Adafruit
- SparkFun u-blox GNSS Arduino Library by SparkFun Electronics
- SparkFun u-blox GNSS v3 by SparkFun Electronics
- SparkFun 9DoF IMU Breakout ICM 20948 Arduino Library by SparkFun Electronics
- SdFat - Adafruit Fork by Bill Greiman
- MicroNMEA by Steve Marple

### Additional Software

Download **u-center** from u-blox for GPS monitoring and configuration.

---

## Repository Structure

```
OpenLog_Artemis_GNSS_Logging_Modified/   # OLA firmware (Arduino sketch)
    OpenLog_Artemis_GNSS_Logging_Modified.ino
    upload_ola_firmware.py               # Apple Silicon: compile + upload script

esp32/                                   # ESP32 sketches
    buoy_combo/                          # Main buoy sketch (cellular NTRIP + power log) — Base+PowerLog
        buoy_combo.ino
        buoy_combo.h
    legacy/                              # Older / alternate sketches
        buoy_combo.ino                   # (main branch copy; superseded by esp32/buoy_combo/)
        buoy_combo.h
        esp32_polaris_wifi.ino
        esp32_rtk_wifi.ino
        esp32_botletic.ino
        esp32_rtk.ino
        esp32_rtk_button.ino
        ...

accelerometer/                           # Datasheets and calibration notebook
    ICM-20948-Datasheet-v1.3.pdf
    AK09916-Magnetometer-Datasheet.pdf
    magcal_notebook.ipynb                # Magnetometer calibration (hard + soft iron)

ubx_parsers/                             # UBX binary to CSV conversion
    ubx_parser.py
    v2_ubx_parser.py
    v3_ubx_parser.py

matlab/                                  # Legacy MATLAB scripts (see Data Processing note)

tutorials/                               # Student guides and reference docs
    IMU_STUDENT_GUIDE.md                 # IMU data logging (Arduino IDE + CLI)
    MAGNETOMETER_CALIBRATION.md          # Hard and soft iron calibration walkthrough
    SERIAL_MONITOR_SETUP.md              # CoolTerm (macOS) and TeraTerm (Windows)
    GETTING_UPDATES.md                   # How to pull instructor updates
    ACCELEROMETER_PERFORMANCE_GUIDE.md
    LOGGING_ANALYSIS.md
    VSCODE_ARDUINO_SETUP.md
```

---

## Configuration

### secrets.h

Create a `secrets.h` file in the `esp32/buoy_combo/` folder (Arduino sketch tab) with your NTRIP credentials:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

// WiFi credentials (used by esp32_rtk_wifi.ino)
const char* ssid     = "your_network";
const char* password = "your_password";

// NTRIP caster credentials (Point One Nav Polaris)
const char* casterHost   = "polaris.pointonenav.com";
const uint16_t casterPort = 2101;
const char* mountPoint   = "POLARIS";
const char* casterUser   = "your_username";
const char* casterUserPW = "your_password";

#endif
```

> **Credentials:** Username and password are provided by the instructor. Do not commit `secrets.h` to the repository.

---

## Deployment Procedure

> **Battery assignment — verify before use:**  
> The Hardware section above lists the **6000mAh** battery for the OpenLog Artemis + GPS and the **850mAh** battery for the ESP32 + cellular modem. There is a known inconsistency in the steps below — confirm the correct assignment with your instructor before connecting batteries.

### Step 1: Upload OpenLog Artemis Firmware

1. Connect the OpenLog Artemis to your computer via USB-C
2. Upload the `OpenLog_Artemis_GNSS_Logging_Modified` sketch:
   - Board: **RedBoard Artemis ATP** (Tools → Board)
   - Port: select the correct port (Tools → Port)
3. Open the Serial Monitor at **115200 baud**
4. Note the datalog and imulog file numbers
5. Connect the battery — verify polarity before plugging in (red = positive)

### Step 2: Upload ESP32 Sketch

1. Connect the ESP32 to your computer via micro-B USB
2. Open and upload the sketch in `esp32/buoy_combo/`:
   - Board: **SparkFun ESP32 Thing Plus** (or compatible ESP32)
   - Partition Scheme: **No OTA (Large APP)** if using WiFi/BLE sketches; default is usually fine for `buoy_combo`
   - Port: select the correct port (Tools → Port)
3. Open the Serial Monitor at **115200 baud** (USB debug; modem UART remains 9600)
4. Connect the battery — verify polarity before plugging in (red = positive)

### Step 3: Verify Connections

Watch the ESP32 Serial Monitor for each stage (allow 1–3 minutes for first LTE registration):

**Modem config (boot):**
```
[MODEM] LTE CAT-M only, band 12
[DIAG] CPIN: +CPIN: READY
[DIAG] CGREG (LTE data — used by [NET]): +CGREG: ...
```

**Network:**
```
[NET] CSQ=25 CGREG=1 (home)
[NET] connected
```

**GPRS** (may take more than one attempt):
```
[GPRS] enabled
```

**NTRIP:**
```
[NTRIP] connecting to <host>:2101
[NTRIP] connected
[RTCM] ... B/10s backlog=0
```

**GPS / RTK:**
```
[GPS] fix=3 rtk=FIXED sats=...
```

**Power (battery powered only):**
```
[PWR] I=... mA  V=... V  P=... mW
```
On USB bench power, `[PWR] (bench/USB — INA228 not on active battery rail)` is normal.

### Step 4: Monitor Fix Status (Optional)

Open u-center to watch fix status in real time. The ZED-F9P LED also indicates status:

| LED | Fix status |
|-----|-----------|
| Solid on | 3D (no RTK) |
| Blinking | RTK Float |
| Off | RTK Fixed (cm-level) |

### Step 5: Collect Data

Once in RTK Fixed mode, data is being recorded to the SD card. Note your start and end times.

### Step 6: End Logging

1. In the OpenLog Serial Monitor, press any key to open the menu
2. Press `q` to quit and close the log file
3. Press `y` to confirm
4. Disconnect all batteries
5. Remove the microSD card from the OpenLog Artemis

---

## Data Processing

### Parse UBX to CSV

Edit `ubx_parsers/v2_ubx_parser.py` to set your filenames:

```python
if __name__ == "__main__":
    ubx_filename = "dataLog00008.ubx"
    csv_filename = "dataLog00008_parsed.csv"
```

Run:
```bash
pip install pyubx2
python3 ubx_parsers/v2_ubx_parser.py
```

### MATLAB (legacy)

Example MATLAB scripts for some aspects of data visualization can be found in the `matlab/` folder. This course is focused on Python, which is more complete and is recommended over the MATLAB scripts.

### Magnetometer Calibration (Python)

Open `accelerometer/magcal_notebook.ipynb` in VS Code using the `mae223` kernel. The notebook walks through hard iron offset computation and full hard + soft iron correction using an ellipsoid fit.

---

## Output Data Format

### GPS CSV

| Column | Description |
|--------|-------------|
| timestamp | UTC time |
| latitude | degrees |
| longitude | degrees |
| altitude_ellipsoid | Height above ellipsoid (m) |
| fix_type | GNSS fix type (0–5) |
| carrier_solution | RTK status: 0=None, 1=Float, 2=Fix |
| h_acc | Horizontal accuracy (m) |
| v_acc | Vertical accuracy (m) |

### IMU CSV

| Column | Description |
|--------|-------------|
| Timestamp | yyyy/MM/dd HH:mm:ss.SS |
| Sensor | "IMU" |
| AccX/Y/Z | Acceleration (milli-g) |
| GyrX/Y/Z | Angular rate (deg/s) |
| MagX/Y/Z | Magnetic field (µT) |
| Temp | Temperature (°C) |

---

## OLA Menu

Press any key in the Serial Monitor to open the menu:

```
1) Configure Logging
2) Configure GNSS Device
3) Configure IMU Sensor
4) Configure Qwiic Bus
5) Configure Power Options
f) Open New Log File
g) Reset GNSS
r) Reset all settings to default
q) Quit: Close log file and power down
x) Return to logging
```

---

## Troubleshooting

**GPS not detected**
- Check Qwiic cable connections
- Verify I2C address is 0x42
- Confirm antenna has a clear sky view

**NTRIP connection fails**
- Check cellular signal (CSQ first value should be well below 99; 99 = no signal)
- Verify SIM card is inserted and active (Hologram dashboard)
- Check credentials in `secrets.h`
- Confirm LTE antenna on SIM7000; try outdoors or near a window
- If on Verizon, set `LTE_CATM_BAND` to `13` in `buoy_combo.h`

**Modem not registered (`[NET] CSQ=99 CGREG=0`)**
- Wait up to 3 minutes after boot for `CGREG=2` (searching) then `1` or `5`
- Full power cycle: unplug USB/battery 30 s, then retry
- Review boot `[DIAG]` lines (CPIN must be `READY`, CGREG should not stay `0,0` with CSQ `99,99`)
- See branch notes above for CAT-M band and `configureNetwork()`

**No RTK fix**
- Ensure clear sky view with no obstructions
- Confirm RTCM data is arriving (watch Serial Monitor for "RTCM → ZED" messages)
- RTK convergence can take a few minutes after RTCM starts flowing

**IMU data not logging**
- Enable IMU logging via menu option 3
- Check IMU initialization messages in Serial output

**SD card errors**
- Format as FAT32
- Reseat the card
- Check available space

---

## Power

| Component | Current draw |
|-----------|-------------|
| ZED-F9P | 85 mA |
| ANN-MB1 antenna | 15 mA |
| SIM7000 | 110–170 mA |
| OpenLog Artemis | TBD |
| ESP32 Thing Plus | TBD |
