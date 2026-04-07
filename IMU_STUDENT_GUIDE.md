# IMU Data Logger - Student Guide

Record accelerometer, gyroscope, magnetometer, and temperature data using the OpenLog Artemis data logger. No GPS or external hardware required.

This guide covers two approaches to uploading firmware:
- **Arduino IDE** — works on all platforms (Windows, macOS, Linux)
- **Arduino CLI** — the setup documented here is specific to **Apple Silicon Mac (M1/M2/M3)**

---

## What You Need

- SparkFun OpenLog Artemis (has a built-in ICM-20948 IMU)
- MicroSD card (FAT32 formatted)
- USB-C cable + USB-serial adapter
- Computer with Arduino IDE or Arduino CLI installed
- MATLAB or Python (for plotting)

---

## 1. Upload the Firmware

### Option A: Arduino IDE (all platforms)

#### Install the Board Package

1. Open Arduino IDE
2. Go to **File > Preferences**
3. In "Additional Board Manager URLs", add:
   ```
   https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search for **SparkFun Apollo3** and install **version 2.2.1** (must be this exact version)

#### Install Libraries

Open **Sketch > Include Library > Manage Libraries** and install:

- **SparkFun u-blox GNSS v3** by SparkFun Electronics
- **SparkFun 9DoF IMU Breakout ICM 20948 Arduino Library** by SparkFun Electronics
- **SdFat - Adafruit Fork** by Bill Greiman

#### Board Settings

- **Tools > Board:** SparkFun Apollo3 > RedBoard Artemis ATP
- **Tools > Port:** Select the port that appears when you plug in the board

#### Upload

1. Insert a FAT32-formatted microSD card into the OpenLog Artemis
2. Connect the OpenLog Artemis to your computer via USB-C
3. Open the sketch: `OpenLog_Artemis_GNSS_Logging_Modified/OpenLog_Artemis_GNSS_Logging_Modified.ino`
4. Click **Upload**
5. Open **Tools > Serial Monitor** and set baud rate to **115200**

---

### Option B: Arduino CLI (Apple Silicon Mac — M1/M2/M3)

> The setup documented here is specific to Apple Silicon Macs. The SparkFun Apollo3 toolchain ships as x86_64, which requires Rosetta 2 to run on Apple Silicon. The Arduino CLI approach below handles this correctly.
>
> **Why use this approach?** Once the CLI is set up, Claude Code can handle firmware compilation and uploads directly — you describe what you need and Claude runs the commands. You do not need to use the command line yourself. This is useful if you are working on an Apple Silicon Mac and want Claude to manage firmware updates on your behalf.

#### Install Arduino CLI and Rosetta 2

```bash
brew install arduino-cli
softwareupdate --install-rosetta --agree-to-license
```

#### Configure and Install Board Cores

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
arduino-cli core update-index
arduino-cli core install "SparkFun:apollo3@2.2.1"
```

> Apollo3 must be version 2.2.1 exactly — v2.1.1 has a known I2C bug.

#### Install Libraries

```bash
arduino-cli lib install "SparkFun u-blox GNSS v3"
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library"
arduino-cli lib install "SdFat"
```

#### Upload Firmware

1. Insert a FAT32-formatted microSD card into the OpenLog Artemis
2. Connect via USB-serial adapter
3. Run the upload script:

```bash
python3 upload_ola_firmware.py
```

This automatically detects the port, compiles, and uploads. Successful output looks like:

```
Found port: /dev/cu.usbserial-10

--- Compiling sketch ---
Sketch uses 254764 bytes (25%) of program storage space.

--- Uploading firmware ---
Artemis SVL Bootloader
Got SVL Bootloader Version: 5
[##################################################] Upload complete

Done. Firmware uploaded successfully.
```

#### Read Serial Output

`arduino-cli monitor` does not work non-interactively on macOS. Use pyserial instead:

```bash
pip3 install pyserial  # one-time install

python3 -c "
import serial, time
s = serial.Serial('/dev/cu.usbserial-10', 115200, timeout=1)
start = time.time()
while time.time() - start < 10:
    data = s.read(256)
    if data:
        print(data.decode('utf-8', errors='replace'), end='', flush=True)
s.close()
"
```

> Always use `/dev/cu.*` not `/dev/tty.*` on macOS — the `tty` variant blocks indefinitely.

---

## 2. What to Expect on Boot

You will see output like this:

```
Artemis OpenLog GNSS v3.2
Starting IMU initialization...
IMU online
SD card online
Data logging online
Created log file: dataLog00001.ubx
Opening IMU file: imuLog00001.csv
IMU file opened successfully
GNSS offline
```

**"GNSS offline" is normal.** It just means no GPS module is connected. The IMU logs independently.

Take note of the IMU log file number (e.g., `imuLog00001.csv`) — you will need this later.

---

## 3. Recording Data

Once the boot messages finish, the logger is recording IMU data to the SD card at 10 Hz.

**Default settings:**
- IMU sample rate: 100 ms (10 Hz)
- All sensors enabled: accelerometer, gyroscope, magnetometer, temperature

Each line of terminal output looks like:

```
2026/04/07 15:36:55.00,IMU,510.74,-133.30,-860.35,-1,-2,1,6,-53,-15,37
```

Format: `Timestamp, IMU, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ, Temp`

### Collecting a Dataset

1. Place the logger on a flat surface for a baseline, or hold it and move/rotate it
2. Note the start time
3. Record for at least 30 seconds
4. Note the end time

---

## 4. Stop Logging and Retrieve Data

1. Open the Serial Monitor (or serial terminal)
2. Type any character and press Enter — this opens the menu
3. Press **q** then **y** to quit and safely close the log file
4. Unplug USB
5. Remove the microSD card
6. Insert the SD card into your computer and copy the `imuLogXXXXX.csv` file

> **Important:** Always quit through the menu before removing the SD card. Pulling power without quitting can corrupt the file.

---

## 5. Plot the Data

### Using MATLAB

Open `matlab/simple_IMU_plotting.m` and change the filename on line 7:

```matlab
data = readtable('imuLog00001.csv');  % <-- your file number here
```

Run the script. You will get four subplots:

| Plot | Axes | Units | What It Shows |
|------|------|-------|---------------|
| Accelerometer | X, Y, Z | m/s² | Linear acceleration (gravity appears as ~9.81 on Z when flat) |
| Gyroscope | X, Y, Z | deg/s | Rotational rate (near zero when still) |
| Magnetometer | X, Y, Z | µT | Magnetic field direction/magnitude |
| Temperature | — | °C | Board temperature |

### Using Python

```python
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

with open('imuLog00001.csv', newline='') as f:
    f.readline()  # skip header
    rows = [line.strip().split(',') for line in f if line.strip()]

t       = [datetime.strptime(r[0], '%Y/%m/%d %H:%M:%S.%f') for r in rows]
AccX    = np.array([float(r[2]) for r in rows]) / 1000 * 9.81  # milli-g to m/s²
AccY    = np.array([float(r[3]) for r in rows]) / 1000 * 9.81
AccZ    = np.array([float(r[4]) for r in rows]) / 1000 * 9.81
GyrX    = np.array([float(r[5]) for r in rows])
GyrY    = np.array([float(r[6]) for r in rows])
GyrZ    = np.array([float(r[7]) for r in rows])

fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t, AccX, label='X')
axes[0].plot(t, AccY, label='Y')
axes[0].plot(t, AccZ, label='Z')
axes[0].set_ylabel('Acceleration (m/s²)')
axes[0].set_title('Accelerometer')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t, GyrX, label='X')
axes[1].plot(t, GyrY, label='Y')
axes[1].plot(t, GyrZ, label='Z')
axes[1].set_ylabel('Angular Rate (deg/s)')
axes[1].set_title('Gyroscope')
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.show()
```

> Note: The CSV uses `\r\n` line endings (Windows-style from the SD card). The `newline=''` argument in `open()` handles this correctly.

---

## 6. Menu Reference

Press any key in the Serial Monitor to open the menu during logging.

| Key | Action |
|-----|--------|
| **3** | Configure IMU — enable/disable sensors, change sample rate |
| **1** | Configure logging — toggle SD logging, terminal output |
| **q** | Quit — close log file and power down safely |
| **f** | Start a new log file (without restarting) |
| **x** | Return to logging |

### IMU Settings (Menu 3)

```
1) IMU Logging           : Enabled/Disabled
2) Log Accelerometer     : Enabled/Disabled
3) Log Gyroscope         : Enabled/Disabled
4) Log Magnetometer      : Enabled/Disabled
5) Log Temperature       : Enabled/Disabled
6) IMU Log Rate (ms)     : 100
```

You can change the sample rate (option 6) to values between 10 ms (100 Hz) and 5000 ms (0.2 Hz). Settings are saved and persist across reboots.

---

## IMU CSV File Format

| Column | Description | Units |
|--------|-------------|-------|
| Timestamp | Date and time | yyyy/MM/dd HH:mm:ss.SS |
| Sensor | Always "IMU" | — |
| AccX, AccY, AccZ | Acceleration | milli-g |
| GyrX, GyrY, GyrZ | Angular velocity | deg/s |
| MagX, MagY, MagZ | Magnetic field | µT |
| Temp | Board temperature | °C |

> Accelerometer values are in milli-g (1000 = 1g = 9.81 m/s²).

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| "SD card offline" | Format SD card as FAT32. Make sure it's seated properly. |
| "IMU initialization failed" | Press reset. If it persists, verify board is genuine OpenLog Artemis with ICM-20948. |
| Serial Monitor shows nothing | Check baud rate is 115200. Try pressing reset. |
| CSV file is empty | Did you quit properly (q, then y)? Data may not have been flushed. |
| "GNSS offline" | **Expected.** GPS is not used in this exercise. |
| IMU values show "%.2f" | Firmware is outdated. Re-upload the latest version. |
| Menu opened accidentally | Press **x** to return to logging. |
| `bad CPU type` (Apple Silicon CLI) | Install Rosetta 2: `softwareupdate --install-rosetta --agree-to-license` |
| Serial reads hang (Apple Silicon CLI) | Use `/dev/cu.*` not `/dev/tty.*` |
| Port busy (Apple Silicon CLI) | Kill any other process using the port and retry. |
