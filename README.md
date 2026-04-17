# RTK GPS Ocean Wave and Tide Measurements

A centimeter-level precision GPS data logging system for oceanographic applications, combining RTK GPS positioning with IMU sensor data collection. Developed at UCSD Scripps Institution of Oceanography.

## Overview

This system achieves centimeter-level positioning accuracy by receiving Real-Time Kinematic (RTK) corrections via NTRIP protocol over cellular networks. It consists of three main components:

1. **Sparkfun ZED-F9P** - High precision RTK GPS receiver board based around the Ublox ZED-F9P RTK
2. **ESP32 + SIM7000 LTE Module** - Handles cellular connectivity and NTRIP correction streaming to SparkFun ZED-F9P
3. **OpenLog Artemis Data Logger** - Records high-precision GPS data and IMU measurements to SD card

---

## Hardware Requirements

- SparkFun ZED-F9P-02B-00 (RTK GPS receiver module)
- Botletics SIM7000 LTE Shield + board
- SparkFun OpenLog Artemis (data logger with built-in ICM-20948 IMU)
- SparkFun ESP32 Thing Plus
- u-blox ANN-MB1-00-00 Antenna with grounding plate
- LiPo Battery 3.7V 6000mAh 22.2Wh (for OpenLog Artemis + GPS)
- LiPo Battery 3.7V 850mAh 3.145Wh (for ESP32 + cellular modem)
- Hologram SIM Card
- MicroSD Card (FAT32 formatted)

### Wiring

- ZED-F9P connects to OpenLog Artemis via Qwiic/I2C (address 0x42)
- ZED-F9P connects to ESP32 via I2C for RTCM correction injection
- Antenna connects to ZED-F9P antenna port

---

## Software Requirements

### Arduino IDE Setup

#### Board Packages

**SparkFun Apollo3 Boards** by SparkFun Electronics
- Must be version 2.2.1
- If you get board-related errors, add this URL to Additional Board Manager URLs:
  ```
  https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
  ```

**ESP32** by Espressif Systems

#### Partition Scheme (Important)

The `esp32_rtk_wifi` sketch uses both WiFi and BLE, which together exceed the default 1.28MB app partition. Before compiling, set:

`Tools → Partition Scheme → No OTA (Large APP)`

This gives ~2MB for the app. You must set this each time you use a new machine. OTA is not needed for field buoys.

#### Libraries

Install these from the Arduino Library Manager:

- BotleticsSIM7000 by Botletics
- SparkFun u-blox GNSS Arduino Library by SparkFun Electronics
- SparkFun u-blox GNSS v3 by SparkFun Electronics
- SparkFun 9DoF IMU Breakout ICM 20948 Arduino Library by SparkFun Electronics
- SdFat - Adafruit Fork by Bill Greiman
- MicroNMEA by Steve Marple (may not be needed)

### Additional Software

Download **u-center** from u-blox for GPS monitoring and configuration.

---

## Repository Structure

```
OpenLog_Artemis_GNSS_Logging_Modified/   # OLA firmware (Arduino sketch)
    OpenLog_Artemis_GNSS_Logging_Modified.ino
    upload_ola_firmware.py               # Apple Silicon: compile + upload script

esp32/                                   # ESP32 sketches
    buoy_combo.ino                       # Main buoy sketch (cellular NTRIP)
    buoy_combo.h
    esp32_rtk_wifi.ino                   # WiFi NTRIP + BLE command interface
    esp32_rtk.ino
    esp32_botletic.ino

accelerometer/                           # ICM-20948 and AK09916 datasheets
    ICM-20948-Datasheet-v1.3.pdf
    AK09916-Magnetometer-Datasheet.pdf

ubx_parsers/                             # UBX binary to CSV conversion
    v3_ubx_parser.py
    v2_ubx_parser.py

tutorials/                               # Student guides and reference docs
    IMU_STUDENT_GUIDE.md                 # IMU data logging guide (Arduino IDE + CLI)
    SERIAL_MONITOR_SETUP.md              # TeraTerm (Windows) and CoolTerm (macOS)
    GETTING_UPDATES.md                   # How to pull instructor updates
    ACCELEROMETER_PERFORMANCE_GUIDE.md
    VSCODE_ARDUINO_SETUP.md
    LOGGING_ANALYSIS.md
```

---

## Configuration

### secrets.h

Create a `secrets.h` file in the buoy_combo folder with your NTRIP credentials:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

// WiFi credentials (used by esp32_rtk_wifi.ino)
const char* ssid = "UCSD-GUEST";
const char* password = "";

// NTRIP caster credentials
const char* casterHost = "rtk2go.com";
const uint16_t casterPort = 2101;
const char* mountPoint = "BASE_CCS";
const char* casterUser = "your_email";
const char* casterUserPW = "";

#endif
```

### UCSD NTRIP Server

For UCSD users on protected WiFi:
- Address: 132.239.117.27
- Port: 2101
- Username: fieldcrew
- Password: iod.1234

---

## Procedure

### Step 1: Upload OpenLog Artemis Sketch

1. Plug in the Artemis data logger to your computer using USB-C
2. Upload the OpenLog_Artemis_GNSS_Logging sketch
   - Set the board to **RedBoard Artemis ATP** (Tools > Board)
   - Select the correct port (Tools > Port)
3. Open the Serial Monitor. Baud rate should be 115200
4. Take note of the datalog number and imulog number
5. Plug in the 850mAh battery to the Artemis data logger. Make sure the polarity is correct (red goes to positive)

### Step 2: Upload ESP32/Buoy Combo Sketch

1. Plug in the ESP32 to your computer with a micro-B cable
2. Plug in the 6000mAh battery to the ESP32. Make sure the polarity is correct (red goes to positive)
3. Upload the buoy_combo sketch
   - Set the board to **Adafruit ESP32 Feather** (Tools > Board)
   - Select the correct port (Tools > Port)
4. Open the Serial Monitor and verify the baud rate is 9600

### Step 3: Verify Connections

Watch the ESP32 Serial Monitor for:

**Network Status:**
```
=== Checking Network Status ===
Signal strength (RSSI): 31 (Excellent)
Network status 5: Registered roaming
Network connection confirmed!
```

**GPRS Configuration:**
```
=== Starting GPRS Enable Process ===
GPRS enabled successfully!
GPRS verification complete
```

Note: GPRS configuration may take more than 1 try.

**NTRIP Connection:**
```
Attempting NTRIP connection via LTE TCP...
TCP connection established, sending NTRIP request...
Found '200' - HTTP OK
NTRIP connection successful!
```

### Step 4: Monitor with u-center (Optional)

1. Open u-center
2. Make sure the port is selected
3. Under Receiver, ensure NTRIP Client is enabled (check mark next to it)
4. On the right side, it will show either 3D mode, floating, or fixed
5. Wait until it says "fixed" to get cm level accuracy

**Note:** u-center does not "enable" the connection - it just lets you see the fix status more easily and view real-time data. If u-center is not connected, you can tell the fix status from the LED on the ZED-F9P:
- LED solid on = 3D mode
- LED blinking = floating
- LED off = fixed mode (cm-level accuracy)

### Step 5: Begin Testing

Once in fixed mode, you're ready to start your test. Data is being recorded to the SD card continuously. Record the start time and end time of your test so you know what data to look at later.

### Step 6: End Testing

1. Type any key into the Serial Monitor in the OpenLog sketch
2. Type `q`
3. Type `y`
4. This ends the logging to the SD card
5. Unplug all batteries and disconnect everything
6. Remove the micro SD card from the Artemis data logger

---

## Data Processing

### Parse UBX File

Take the .ubx file and run it through `v2_ubx_parser.py`

Edit the script to set your filenames:
```python
if __name__ == "__main__":
    ubx_filename = "dataLog00008.ubx"
    csv_filename = "v2_dataLog00008_parsed_positions.csv"
```

Run:
```bash
python3 ./v2_ubx_parser.py
```

Required Python package:
```bash
pip install pyubx2
```

### Plot GPS Data

Use the output CSV file with `v3_fall_parsed_position_graph.m`

Change `file_name` in the script:
```matlab
filename = 'v2_dataLog00008_parsed_positions.csv';
```

### Plot IMU Data

Use the IMU log file with `simple_IMU_plotting.m`

Change the filename in the script:
```matlab
data = readtable('imuLog00013.csv');
```

---

## Output Data Format

### GPS CSV Columns

- timestamp - UTC time
- latitude - latitude (degrees)
- longitude - longitude (degrees)
- altitude_ellipsoid - Height above ellipsoid (meters)
- fix_type - GNSS fix type (0-5)
- carrier_solution - RTK status: 0=None, 1=Float, 2=Fix
- h_acc - Horizontal accuracy (meters)
- v_acc - Vertical accuracy (meters)

### IMU CSV Columns

- Timestamp - System timestamp (yyyy/MM/dd HH:mm:ss.SS)
- Sensor - Sensor identifier ("IMU")
- AccX, AccY, AccZ - Acceleration (milli-g)
- GyrX, GyrY, GyrZ - Angular rate (deg/s)
- MagX, MagY, MagZ - Magnetic field (uT)
- Temp - Temperature (C)

---

## Menu System

Press any key in the OpenLog Serial Monitor to access the menu:

```
Menu: Main Menu
1) Configure Logging
2) Configure GNSS Device
3) Configure IMU Sensor
4) Configure Qwiic Bus
5) Configure Power Options
f) Open New Log File
g) Reset GNSS
r) Reset all OLA settings to default
q) Quit: Close log file and power down
d) Debug Menu
i) IMU Debug
x) Return to logging
```

---

## Troubleshooting

### GPS Not Detected
- Check I2C connections (Qwiic cables)
- Verify power to ZED-F9P module
- Make sure antenna has clear sky view
- Check I2C address (default 0x42)

### NTRIP Connection Fails
- Verify cellular signal strength (RSSI should be > 10)
- Check that SIM card is properly inserted and activated
- Confirm NTRIP credentials in secrets.h

### No RTK Fix
- Make sure you have a clear sky view with no obstructions
- Check that RTCM data is being received (watch Serial Monitor)
- Be patient (could take a few minutes)

### IMU Data Not Logging
- Enable IMU logging in menu (option 3)
- Check IMU initialization messages in Serial output
- Verify SPI connections to ICM-20948

### SD Card Errors
- Format card as FAT32
- Use a good quality microSD card
- Make sure card is properly seated
- Check that there's enough free space

---

## Power Estimates

- ZED-F9P: 85 mA
- OpenLog Artemis: 
- ESP32 Thing Plus:
- SIM7000: 110-170mA
- ANN-MB1: 15mA


