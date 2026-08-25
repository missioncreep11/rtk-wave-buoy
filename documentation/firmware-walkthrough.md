# Firmware walkthrough (`buoy_combo`)

Map of `esp32/buoy_combo/buoy_combo.ino` and `buoy_combo.h` for reading the source. Recovery detail stays in [failure-paths.md](failure-paths.md).

---

## Files

| File | Contents |
|------|----------|
| `buoy_combo.ino` | Pin aliases, globals, `setup()`, `loop()`, modem passthrough on USB Serial |
| `buoy_combo.h` | `BuoyModem` class, network/NTRIP/health/telemetry, GNSS/INA init |
| `secrets.h` | NTRIP caster, Hologram device key, `TELEMETRY_INTERVAL_MS` (from `.example`) |

All TCP is plain `AT+CIP*` via `BuoyModem::tcp*Plain()` — the sketch never calls the library's SSL-gated paths, so the `BOTLETICS_SSL` default inside BotleticsSIM7000.h (=1, unconditionally redefined there) has no effect here.

---

## `setup()` sequence

```text
BLE NUS init (advertising as RTK-Buoy-XXXX)
STATUS_LED + SHUTDOWN_BTN interrupt
initializeIna228()
initializeGnssUart()     → UART2 ZED, RTCM3 on UART1
modem RST high, powerOn(PWRKEY), delay 5s
modemLinkBegin()             → probe 115200, fall back 9600 → relock 115200
modem.configureNetwork()     → boot path: CFUN=1 first, (boot) band log
```

On modem failure, sketch blocks in `while(1)`.

---

## `loop()` sequence (each ~10 ms iteration)

| Order | Call | Role |
|-------|------|------|
| 1 | USB Serial → `modemSS` | Manual AT passthrough (returns early) |
| 2 | `networkStatusCheck()` | Poll `CGREG` / `CSQ`; registration lost handling |
| 3 | `setupGprs()` | Enable packet data when registered (proceeds on CSQ=99; only CSQ=0 gates) |
| 4 | `beginNTRIPClient()` | If GPRS up and NTRIP down (30 s retry); detects chunked vs raw body |
| 5 | `handleNTRIPData()` | Read caster TCP (chunked NTRIP/2.0 or raw ICY) → write RTCM to `gpsSerial` |
| 6 | `monitorConnectionHealth()` | Stale path, CGREG, NTRIP streak, escalated recover |
| 7 | `postTelemetry()` | Hologram JSON on interval |
| 8 | Every 5 s | `printPowerStatus()`, `[GPS] fix/rtk/sats` |
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
| `tcpConnectPlain()` / `tcpSendPlain()` | Plain TCP for NTRIP/Hologram (multiplexed links 0/1) |
| `sendHologramCloudMessage()` | Hologram Cloud Socket framing on link 1 — NTRIP stays open |
| `buildGGA(label)` | GGA sentence from one cached PVT poll; sent on connect + every 10 s |
| `printDiagnostics()` | CPIN, CFUN, CGREG, CSQ, CNACT snapshot |

### Connection lifecycle

| Function | Purpose |
|----------|---------|
| `networkStatusCheck()` | Registration state; triggers invalidate on confirmed loss |
| `setupGprs()` | Enable packet data when `CGREG` 1 or 5 |
| `invalidateDataPath()` | Drop NTRIP + GPRS flags — `[DATA] invalidate` |
| `refreshGprs()` | Soft PDP/CIP refresh — `[GPRS] refresh` |
| `modemRecoverEscalated()` | RST then PWRKEY on repeated triggers |
| `modemHardRecover()` / `modemPowerCycleRecover()` | Level 1 / 2 recover |

### NTRIP / GNSS

| Function | Purpose |
|----------|---------|
| `beginNTRIPClient()` | HTTP/NTRIP handshake; picks chunked (NTRIP/2.0) vs raw body from headers |
| `handleNTRIPData()` | Pump RTCM either framing; updates `lastReceivedRtcmMs`, `noteCellularActivity()` |
| `initializeGnssUart()` | Baud scan, `setPortInput(RTCM3)`, UBX out |
| `postTelemetry()` | Build JSON, Hologram send on link 1 (CIPMUX=1 — NTRIP stays up) |

### Sensors / UI

| Function | Purpose |
|----------|---------|
| `initializeIna228()` | I2C INA228 on pack bus |
| `printPowerStatus()` | `[PWR]` bus V, mW, or bench message |
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
| `lastReceivedRtcmMs` | Last RTCM byte time (health grace) |
| `consecutiveNtripFailures` | Count toward escalated recover |
| `ntripChunkedStream` | Body framing from caster headers: true = NTRIP/2.0 chunked, false = raw ICY |

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
