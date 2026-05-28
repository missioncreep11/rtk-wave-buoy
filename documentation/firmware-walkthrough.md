# Firmware walkthrough (`buoy_combo`)

Map of `esp32/buoy_combo/buoy_combo.ino` and `buoy_combo.h` for reading the source. Recovery detail stays in [failure-paths.md](failure-paths.md).

---

## Files

| File | Contents |
|------|----------|
| `buoy_combo.ino` | Pin aliases, globals, `setup()`, `loop()`, modem passthrough on USB Serial |
| `buoy_combo.h` | `BuoyModem` class, network/NTRIP/health/telemetry, GNSS/INA init |
| `secrets.h` | NTRIP caster, Hologram device key, `TELEMETRY_INTERVAL_MS` (from `.example`) |

`BOTLETICS_SSL` must be **0** in the sketch (plain TCP NTRIP on port 2101).

---

## `setup()` sequence

```text
Serial + BluetoothSerial begin
SHUTDOWN_BTN interrupt
initialize_ina228_f()
initialize_gnss_uart_f()     → UART2 ZED, RTCM3 on UART1
modem RST high, powerOn(PWRKEY), delay 5s
modemLinkBegin()             → UART1 @ 9600 then 115200
modem.configureNetwork()     → boot path: CFUN=1 first, (boot) band log
```

On modem failure, sketch blocks in `while(1)`.

---

## `loop()` sequence (each ~10 ms iteration)

| Order | Call | Role |
|-------|------|------|
| 1 | USB Serial → `modemSS` | Manual AT passthrough (returns early) |
| 2 | `network_status_check_f()` | Poll `CGREG` / `CSQ`; registration lost handling |
| 3 | `enable_gprs_f()` | PDP + CIP when registered |
| 4 | `beginNTRIPClient()` | If GPRS up and NTRIP down (30 s retry) |
| 5 | `handleNTRIPData()` | Read caster TCP → write RTCM to `gpsSerial` |
| 6 | `monitor_connection_health()` | Stale path, CGREG, NTRIP streak, escalated recover |
| 7 | `post_telemetry_f()` | Hologram JSON on interval |
| 8 | Every 5 s | `print_power_status_f()`, `[GPS] fix/rtk/sats` |
| 9 | `updateStatusLED()` | LED vs registration/NTRIP |
| 10 | `gracefulShutdown()` | If GPIO 0 pressed |

---

## Key functions

### Modem / network (`BuoyModem` in `buoy_combo.h`)

| Function | Purpose |
|----------|---------|
| `configureNetwork(afterRecover)` | Hologram APN, LTE CAT-M bands; boot vs recover paths |
| `configureLteCatM(afterRecover)` | `CBANDCFG`, `CFUN=0` only when `afterRecover==true` |
| `ensurePdpActive()` / `bringUpCipStack()` | PDP context and `AT+CIP*` stack |
| `tcpConnectPlain()` / `tcpSendPlain()` | Plain TCP for NTRIP (not SSL) |
| `sendHologramCloudMessage()` | Hologram Cloud Socket framing |
| `printDiagnostics()` | CPIN, CFUN, CGREG, CSQ, CNACT snapshot |

### Connection lifecycle

| Function | Purpose |
|----------|---------|
| `network_status_check_f()` | Registration state; triggers invalidate on confirmed loss |
| `enable_gprs_f()` | Enable packet data when `CGREG` 1 or 5 |
| `invalidateDataPath()` | Drop NTRIP + GPRS flags — `[DATA] invalidate` |
| `refreshGprs_f()` | Soft PDP/CIP refresh — `[GPRS] refresh` |
| `modemRecoverEscalated_f()` | RST then PWRKEY on repeated triggers |
| `modemHardRecover_f()` / `modemPowerCycleRecover_f()` | Level 1 / 2 recover |

### NTRIP / GNSS

| Function | Purpose |
|----------|---------|
| `beginNTRIPClient()` | HTTP/NTRIP handshake to caster; sets `ntripConnected` |
| `handleNTRIPData()` | Pump RTCM; updates `lastReceivedRTCM_ms`, `noteCellularActivity()` |
| `initialize_gnss_uart_f()` | Baud scan, `setPortInput(RTCM3)`, UBX out |
| `post_telemetry_f()` | Build JSON, close NTRIP, Hologram send, queue NTRIP retry |

### Sensors / UI

| Function | Purpose |
|----------|---------|
| `initialize_ina228_f()` | I2C INA228 on pack bus |
| `print_power_status_f()` | `[PWR]` bus V, mW, or bench message |
| `updateStatusLED()` | Blink vs solid from net/NTRIP state |
| `gracefulShutdown()` / `shutdownISR()` | User power-down |

---

## Important globals

| Variable | Meaning |
|----------|---------|
| `networkConnected` | `CGREG` registered (1 or 5) |
| `gprsEnabled` | PDP/CIP up |
| `ntripConnected` | TCP to caster open |
| `gpsUARTOnline` | ZED UART2 working |
| `lastReceivedRTCM_ms` | Last RTCM byte time (health grace) |
| `consecutiveNtripFailures` | Count toward escalated recover |
| `s_modemRecoverNextPowerCycle` | Next escalated call uses PWRKEY |

---

## Tunables

Defaults in `buoy_combo.h`; override before `#include "buoy_combo.h"` if needed:

- `LTE_CATM_BAND` — 12 (AT&T/T-Mo), 13 (Verizon)
- `TELEMETRY_INTERVAL_MS` — `secrets.h`
- Recovery/stale timers — table in [failure-paths.md](failure-paths.md)

---

## Where to change behavior

| Goal | Location |
|------|----------|
| Caster / Hologram | `secrets.h` |
| LTE band | `LTE_CATM_BAND` in `buoy_combo.h` |
| Telemetry rate | `TELEMETRY_INTERVAL_MS` in `secrets.h` |
| Recovery timeouts | `#define` block top of `buoy_combo.h` |
| Pins | `buoy_combo.h` / `buoy_combo.ino` |

---

## Related

- [wiring-and-pins.md](wiring-and-pins.md)
- [at-command-primer.md](at-command-primer.md)
- [failure-paths.md](failure-paths.md)
- [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md)
