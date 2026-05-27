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

Active path on this hardware: **ngrok TCP → local Flask** (see [local-portal.md](local-portal.md)).
HTTPS and Hologram cloud paths are blocked — see "Why not HTTPS?" below.

| Symptom | Things to try |
|---------|----------------|
| `[TELEM] TCP connect failed` | Stale ngrok address. Restart ngrok, paste new host:port into `secrets.h`, re-flash |
| `[TELEM] TCP send failed` | Modem dropped the socket; usually auto-recovers next cycle |
| `[TELEM] POST failed` (no other line) | Check `telemetryUrl` starts with `tcp://`; ensure `ngrok tcp 8080` is running |
| Server logs 401 | `BUOY_SECRET` env var on PC must match `telemetrySecret` in `secrets.h` exactly |
| GET on ingest returns 405 | Normal — buoy must **POST** JSON |
| No local map update | Run `test_ingest.ps1` first; check `local_portal/buoy.db` updates |
| Raw JSON stale in browser | Hard-refresh; URL-encode `+` in branch name (`Base%2BPowerLog`); match `docs/config.js` branch |

### Why not HTTPS? (B03 firmware findings)

Diagnosis trail on the SIM7000A reporting `Revision:1351B03SIM7000A`:

| Stack attempted | Result |
|-----------------|--------|
| `AT+SH*` (Botletics `postData` HTTPS) | `+CME ERROR: operation not allowed` on `SHCONN`, even after `CIPCLOSE`, clock sync, DNS config, double `SHDISC` |
| `AT+HTTP*` + `AT+HTTPSSL=1` (CNACT bearer) | `+HTTPACTION: 1,603,0` (DNS / TLS handshake fail) |
| `AT+CAOPEN` raw SSL socket | `+CME ERROR: operation not allowed` |
| `AT+SAPBR` + `AT+HTTPSSL=1` (legacy bearer) | `+HTTPACTION: 1,603,0` even with explicit DNS |
| **`AT+CIPSTART` plain TCP** | **Works** (same path as NTRIP) |

Conclusion: B03 has no working HTTPS post-NTRIP. AT+SH*, AT+CAOPEN, and CNACT-bound HTTPS were added in B05+. Until the modem is upgraded or swapped, all telemetry goes over plain TCP via ngrok.

### Hologram Cloud Socket — also blocked

Newer Hologram accounts (post-Routes) do not issue a Cloud Services Router device key:

- Dashboard → device → no "Configuration" tab
- `POST /api/1/devices/<id>/csr` → 404
- `GET /api/1/devices/<id>/csr` → 404
- `POST /api/1/csr/rdm?deviceid=<id>` → 400
- `GET /api/1/links/cellular/<id>` → record has no key field
- Spacebridge: `tunnelable: 0` on the SIM record

Keep `hologramDeviceKey = ""` until/unless Hologram exposes CSR creation again.

### Re-enabling Cloudflare → GitHub Pages later

The Worker (`rtk-buoy-proxy`) and the GitHub Actions workflow still exist. To
revive that path after a modem upgrade:

1. Flash B05+ firmware on the SIM7000A (`firmware-esp32.md`).
2. Set `telemetryUrl = "https://rtk-buoy-proxy.haberkiran.workers.dev"` (line preserved in `secrets.h` comments).
3. Confirm `[TELEM] POST OK` and check Pages updates.

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
- ngrok free TCP addresses (`tcp://N.tcp.ngrok.io:NNNNN`) change on restart — update `telemetryUrl` and re-flash
- Arduino: Apollo3 **2.2.1** for OLA; ESP32 partition scheme for WiFi/BLE legacy sketches only
