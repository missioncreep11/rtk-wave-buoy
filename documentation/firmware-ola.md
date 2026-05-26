# OpenLog Artemis firmware

Sketch: [`OpenLog_Artemis_GNSS_Logging_Modified/`](../OpenLog_Artemis_GNSS_Logging_Modified/).

## Role

Logs high-precision GNSS (UBX) and IMU measurements to microSD while the ZED-F9P runs RTK. The ESP32 handles NTRIP and telemetry separately.

## Arduino setup

1. Board: **SparkFun RedBoard Artemis ATP** (Apollo3 package **2.2.1** recommended)
2. Additional board URL if needed:
   ```
   https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
   ```
3. Libraries: SparkFun u-blox GNSS, ICM-20948, SdFat, MicroNMEA
4. Connect USB-C, select port, upload sketch
5. Serial monitor **115200** — note datalog / imulog file numbers at boot

Apple Silicon Macs can use [`upload_ola_firmware.py`](../OpenLog_Artemis_GNSS_Logging_Modified/upload_ola_firmware.py) for compile/upload.

## SD card

- Format **FAT32**
- Reseat if mount errors appear

## Menu

Press any key in the serial monitor:

| Key | Action |
|-----|--------|
| 1–5 | Configure logging, GNSS, IMU, Qwiic, power |
| f | New log file |
| g | Reset GNSS |
| r | Reset settings |
| q | Quit — close log and power down |
| x | Return to logging |

## End of deployment

1. Open menu (`q` then `y` to confirm quit)
2. Remove batteries
3. Remove microSD for processing — see [data-processing.md](data-processing.md)

## Output formats

**GPS CSV (parsed from UBX):** timestamp, lat, lon, altitude, fix_type, carrier_solution, h_acc, v_acc

**IMU CSV:** timestamp, Acc/Gyr/Mag axes, temperature
