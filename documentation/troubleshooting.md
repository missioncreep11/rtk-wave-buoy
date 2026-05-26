# Troubleshooting

## LTE / modem

| Symptom | Things to try |
|---------|----------------|
| `[NET] CSQ=99 CGREG=0` | Wait 1–3 min; move outdoors; check SIM active on Hologram |
| `[NET] registration lost` | Modem dropped LTE; waits for CGREG then re-enables GPRS |
| CPIN not READY | Reseat SIM; power cycle 30 s |
| Stays on band wrong carrier | Set `LTE_CATM_BAND` to `13` for Verizon in `buoy_combo.h` |
| No LTE antenna | Attach antenna to SIM7000 |

Boot `[DIAG]` lines show CPIN, CFUN, CREG, CGREG, CSQ, CGATT, COPS.

## NTRIP

| Symptom | Things to try |
|---------|----------------|
| TCP connect failed | Check `casterHost`, port **2101**, credentials in `secrets.h` |
| HTTP error from caster | Wrong mountpoint or user/password |
| No RTCM / no RTK | Confirm `[NTRIP] connected` and `[RTCM]` activity; clear sky view |
| Drops after telemetry | Expected brief reconnect after HTTP POST |
| RTK lost, Hologram shows LTE but **0 bytes** for hours | Zombie PDP — firmware should log `[GPRS] refresh` or `[MODEM] hard recover`; check serial for `fail streak` |

### Automatic data-path recovery (`buoy_combo`)

After a weekend-style outage (CGREG OK, no IP traffic), firmware now:

1. Re-checks **CGREG every 30 s** even when already registered (was only polled until first connect).
2. **Refreshes GPRS** (tear down PDP + CIP, re-enable) if no RTCM/telemetry for **5 min** (`DATA_PATH_STALE_MS`), cooldown **2 min** between refreshes. Does **not** refresh on CNACT `0.0.0.0` while NTRIP/RTCM is still active (CIP and CNACT are separate on SIM7000).
3. Treats **CGREG=0** as network loss only after **2** consecutive bad polls **and** no RTCM in the last **2 min** — a single `[HEALTH] net=0` while `[RTCM]` is active is ignored.
4. **Hard-resets the modem** (RST + `configureNetwork`) after **10** consecutive NTRIP failures, cooldown **10 min**.

Serial lines to watch: `[HEALTH]`, `[DATA] invalidate`, `[GPRS] refresh`, `[MODEM] hard recover`, `[NTRIP] fail streak=N`.

Tune timeouts in `buoy_combo.h` (`DATA_PATH_STALE_MS`, `NTRIP_FAILURES_BEFORE_HARD_RESET`, etc.).

## GPS / RTK

| Symptom | Things to try |
|---------|----------------|
| fix=0 or low sats | Antenna view; check UART wiring ESP32 → ZED |
| fix=3 but rtk=none | RTCM not arriving; wait several minutes after NTRIP up |
| RTK float only | Normal before fix; multipath or long baseline to caster |

Use u-center for detailed receiver state. ZED LED: off = RTK fixed.

## Telemetry

| Symptom | Things to try |
|---------|----------------|
| `[TELEM] POST failed` | ngrok running; URL includes `/api/ingest`; update URL after ngrok restart |
| GET on ingest returns 405 | Normal — buoy must **POST** JSON |
| No local map | Run `test_ingest.ps1` first; check `local_portal/buoy.db` updates |
| Hologram OK but no GitHub update | Route URL, PAT, `event_type: buoy-telemetry` — see [github-pages.md](github-pages.md) |

## OpenLog / SD

| Symptom | Things to try |
|---------|----------------|
| GPS not on OLA | Qwiic cable; I2C address 0x42 |
| SD errors | FAT32 format; reseat card; free space |
| No IMU log | Enable via OLA menu option 3 |

## Power (INA228)

| Symptom | Things to try |
|---------|----------------|
| INA228 not found | Qwiic cable to ESP32 |
| Bench/USB message | Expected when not on battery rail |

## Software

- Do not commit `secrets.h`
- ngrok free URLs change on restart — update `telemetryUrl` and re-flash
- Arduino: Apollo3 **2.2.1** for OLA; ESP32 partition scheme for WiFi/BLE legacy sketches only
