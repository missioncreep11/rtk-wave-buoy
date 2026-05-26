# Hardware

## Components

| Part | Function |
|------|----------|
| SparkFun ZED-F9P-02B | RTK GNSS receiver |
| Botletics SIM7000 LTE shield | Cellular data on ESP32 |
| SparkFun ESP32 Thing Plus | Main buoy controller |
| SparkFun OpenLog Artemis | SD logging, ICM-20948 IMU |
| Adafruit INA228 (Qwiic) | Bus voltage, current, power on buoy power rail |
| u-blox ANN-MB1 antenna + ground plane | GNSS antenna |
| Hologram SIM | LTE data plan |
| LiPo batteries | Separate packs for OLA/GPS stack and ESP32/modem |
| microSD (FAT32) | OLA log storage |

## Wiring

- **ZED-F9P → OLA:** Qwiic/I2C (address 0x42)
- **ZED-F9P → ESP32:** UART for RTCM injection (see sketch pin defines in `buoy_combo.ino`)
- **INA228 → ESP32:** Qwiic I2C (SDA 23 / SCL 22 on Thing Plus Qwiic)
- **LTE antenna → SIM7000** (required for registration)
- **GNSS antenna → ZED-F9P** (clear sky view)

## Power notes

- **6000 mAh** — typically OpenLog Artemis + GPS stack
- **850 mAh** — typically ESP32 + cellular modem

Confirm battery assignment with your deployment checklist before field use.

| Component | Approx. draw |
|-----------|----------------|
| ZED-F9P | ~85 mA |
| GNSS antenna | ~15 mA |
| SIM7000 | ~110–170 mA |
| ESP32 + OLA | varies with logging rate |

On **USB bench power**, INA228 may not sit on the active battery rail. Serial will show a bench/USB message — that is expected.

## ZED-F9P LED (fix hint)

| LED | Meaning |
|-----|---------|
| Solid on | 3D fix, no RTK |
| Blinking | RTK float |
| Off | RTK fixed (cm-level) |

## Cellular band

In `buoy_combo.h`, `LTE_CATM_BAND` defaults to **12** (US AT&T/T-Mobile). Use **13** for Verizon-dominated deployments.
