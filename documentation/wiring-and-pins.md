# Wiring and pin reference

ESP32 Thing Plus (WRL-15663) pin assignments for **`esp32/buoy_combo`**. Override defaults in `buoy_combo.ino` before `#include "buoy_combo.h"` if your harness differs.

Architecture context: [system-architecture.md](system-architecture.md).  
**KiCad wiring diagram:** [hardware-spec.md](hardware-spec.md).

---

## ESP32 pin table

| Signal | GPIO | Connects to | Notes |
|--------|------|-------------|--------|
| `TX_MODEM` | 17 | SIM7000 RX | UART1 @ 115200 (Botletics) |
| `RX_MODEM` | 16 | SIM7000 TX | |
| `TX_GPS` | 12 | ZED-F9P RX1 (MOSI pad) | UART2 — RTCM + UBX |
| `RX_GPS` | 27 | ZED-F9P TX1 (MISO pad) | |
| `I2C_SDA` | 23 | Qwiic SDA (INA228) | 400 kHz |
| `I2C_SCL` | 22 | Qwiic SCL | |
| `BOTLETICS_PWRKEY` | 18 | SIM7000 PWRKEY | Power on / PWRKEY cycle |
| `MODEM_RST` / `RST` | 5 | SIM7000 RST | Hard recover pulls LOW ~2 s |
| `SHUTDOWN_BTN` | 0 | Momentary button → GND | `INPUT_PULLUP`, falling edge ISR |
| `STATUS_LED` | 13 | On-board LED | Registration / NTRIP status |

Definitions: `buoy_combo.h` (GPS/I2C/modem), `buoy_combo.ino` (`BOTLETICS_PWRKEY`, `RST`).

---

## UART wiring (modem and GNSS)

```text
ESP32 UART1 (modem)          SIM7000
  GPIO 17 TX  ─────────────► RX
  GPIO 16 RX  ◄───────────── TX

ESP32 UART2 (GNSS)           ZED-F9P UART1 pads
  GPIO 12 TX  ─────────────► RX1 / MOSI
  GPIO 27 RX  ◄───────────── TX1 / MISO
```

- GNSS UART is probed at 38400, 115200, 230400, 460800 in `initialize_gnss_uart_f()`.
- RTCM3 is enabled on ZED **UART1** input; UBX output on UART1 for PVT polling.

---

## I2C (ESP32 side)

| Device | Address | Bus |
|--------|---------|-----|
| Adafruit INA228 | 0x40 (default) or 0x45 (ALT pin) | ESP32 Qwiic |

The ZED-F9P on the **OLA** Qwiic chain (0x42) is **not** on the ESP32 I2C bus in the deployed combo — ESP32 talks to the ZED over UART2 only.

---

## Power wiring (system)

Per KiCad **rtk-wave-buoy** ([diagram](hardware-spec.md)):

```text
BT1 (3S2P, 75 Wh) ──┐
                      ├── Pack bus (10.8–12.8 V) ── INA228 ── OKI-78SR-3.3 ── 3.3 V
BT2 (3S2P, 75 Wh) ──┘         ~150 Wh total              │
                                                    ESP32 · modem · ZED · OLA
```

- **2× 75 Wh** Li-ion **3S2P** packs in **parallel** (~**150 Wh**).
- INA228 on the **pack bus** (see [hardware-spec.md](hardware-spec.md)).
- On **USB bench power**, `bus_v` in telemetry may not reflect packs — serial shows a bench note via `print_power_status_f()`.

---

## OLA (separate MCU)

| Connection | Notes |
|------------|--------|
| ZED-F9P ↔ OLA | Qwiic I2C (logging firmware) |
| microSD | FAT32 in OLA |
| 3.3 V | From pack DC-DC (schematic); OLA/modem on logic rail |

OLA pins are defined in `OpenLog_Artemis_GNSS_Logging_Modified/` — flash the Artemis board, not the ESP32. See [../README.md](../README.md) §2.

---

## Modem control lines

| Action | Mechanism |
|--------|-----------|
| Cold boot in `setup()` | `modem.powerOn(BOTLETICS_PWRKEY)` |
| Hard recover | `MODEM_RST` LOW 2 s → HIGH, then `configureNetwork(true)` |
| Power cycle recover | `AT+CPOWD=1`, PWRKEY off/on, `modemLinkBegin()`, `configureNetwork(true)` |
| User shutdown | GPIO 0 → `gracefulShutdown()` (`AT+CPOWD=1`, ESP light sleep) |

Details: [failure-paths.md](failure-paths.md).

---

## Bench checks

1. Serial monitor **115200** on ESP32 USB.
2. Confirm boot prints `TX_GPS` / `RX_GPS` pins 12 / 27.
3. `[DIAG] CPIN: READY` after modem config.
4. I2C: expect INA228 at 0x40 unless ALT strapped.

---

## Related

- [firmware-walkthrough.md](firmware-walkthrough.md) — which code uses each peripheral
- [deployment-checklist.md](deployment-checklist.md) — antenna and SIM before deploy
