# Troubleshooting

## LTE / modem

| Symptom | Things to try |
|---------|----------------|
| `[NET] CSQ=99 CGREG=0` | Wait 1–3 min; move outdoors; check SIM active on Hologram |
| `[NET] CSQ=0 CGREG=0 (not registered)` for minutes | Modem stuck off-network; firmware hard-resets after **5 min** (`registration timeout`); power-cycle if needed before re-flash |
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
5. **Hard-resets the modem** when **unregistered too long** while disconnected: **5 min** at `CSQ=0` / `CGREG=0` (or denied), **10 min** if still searching (`CGREG=2`) with usable signal. Covers the case where LTE drops and `[HEALTH]` stops running because `networkConnected` is false.

Serial lines to watch: `[HEALTH]`, `[DATA] invalidate`, `[GPRS] refresh`, `[MODEM] hard recover`, `[MODEM] hard recover: registration timeout`, `[NTRIP] fail streak=N`.

Tune timeouts in `buoy_combo.h` (`DATA_PATH_STALE_MS`, `NTRIP_FAILURES_BEFORE_HARD_RESET`, `UNREGISTERED_HARD_RECOVER_MS`, etc.).

## GPS / RTK

| Symptom | Things to try |
|---------|----------------|
| fix=0 or low sats | Antenna view; check UART wiring ESP32 → ZED |
| fix=3 but rtk=none | RTCM not arriving; wait several minutes after NTRIP up |
| RTK float only | Normal before fix; multipath or long baseline to caster |

Use u-center for detailed receiver state. ZED LED: off = RTK fixed.

## Telemetry

Two supported paths on this hardware:

- **Hologram relay → Cloudflare Worker → GitHub Pages** — public dashboard, no PC needed. See [github-pages.md](github-pages.md).
- **ngrok TCP → local Flask** — development, full payload visible on your PC. See [local-portal.md](local-portal.md).

### Hologram relay path (`[TELEM] Hologram ...`)

| Symptom | Things to try |
|---------|----------------|
| `[TELEM] Hologram failed` | Cloud Socket didn't return `[0,0]`. Most often the modem dropped the socket — usually auto-recovers next cycle. Confirm `hologramDeviceKey` in `secrets.h` matches the Hologram dashboard's Router Credentials for the device |
| `[TELEM] TCP connect failed` (Hologram branch) | LTE registration lost or PDP context broken; let firmware auto-recover or check `[GPRS]` / `[NET]` lines |
| Hologram Activity tab shows the message, no event in Events tab | Refresh after a few seconds — events are indexed asynchronously |
| Event exists but `matched_rules: []` | The Alert isn't subscribed to a matching tag. Use `_SOCKETAPI_` or your device's `_DEVICE_<id>_` |
| Alert log shows HTTP 401 from Worker | `BUOY_SECRET` header value on the Alert doesn't match the Cloudflare env var. Header *name* must be exactly `BUOY_SECRET` (this is a header check, distinct from the env var of the same name) |
| Alert log shows HTTP 400 from Worker | Body template missing or wrong; should be `<<decdata>>` so the Worker receives just the buoy JSON |
| Webhook returns 200, no workflow run | GitHub PAT missing `repo` scope, or `GITHUB_OWNER`/`GITHUB_REPO` env vars wrong on the Worker |
| Workflow ran, `data.json` unchanged | Run logs show what fields the workflow received; confirm payload includes `id`/`device_id`, `fix`, `rtk`, `lat`, `lon` |
| Raw JSON stale in browser | Hard-refresh; URL-encode `+` in branch name (`Base%2BPowerLog`); match `docs/config.js` branch |

### ngrok local portal path (`[TELEM] HTTP POST...`)

| Symptom | Things to try |
|---------|----------------|
| `[TELEM] TCP connect failed` | Stale ngrok address. Restart ngrok, paste new host:port into `secrets.h`, re-flash |
| `[TELEM] TCP send failed` | Modem dropped the socket; usually auto-recovers next cycle |
| `[TELEM] POST failed` (no other line) | Check `telemetryUrl` starts with `tcp://`; ensure `ngrok tcp 8080` is running |
| Server logs 401 | `BUOY_SECRET` env var on PC must match `telemetrySecret` in `secrets.h` exactly |
| GET on ingest returns 405 | Normal — buoy must **POST** JSON |
| No local map update | Run `test_ingest.ps1` first; check `local_portal/buoy.db` updates |

### Why not direct HTTPS? (B03 firmware findings)

Direct HTTPS from the modem to the Cloudflare Worker would eliminate the Hologram hop, but doesn't work on this SIM7000A reporting `Revision:1351B03SIM7000A`:

| Stack attempted | Result |
|-----------------|--------|
| `AT+SH*` (Botletics `postData` HTTPS) | `+CME ERROR: operation not allowed` on `SHCONN`, even after `CIPCLOSE`, clock sync, DNS config, double `SHDISC` |
| `AT+HTTP*` + `AT+HTTPSSL=1` (CNACT bearer) | `+HTTPACTION: 1,603,0` (DNS / TLS handshake fail) |
| `AT+CAOPEN` raw SSL socket | `+CME ERROR: operation not allowed` |
| `AT+SAPBR` + `AT+HTTPSSL=1` (legacy bearer) | `+HTTPACTION: 1,603,0` even with explicit DNS |
| **`AT+CIPSTART` plain TCP** | **Works** (same path as NTRIP and Hologram Cloud Socket) |

Conclusion: B03 has no working HTTPS post-NTRIP. `AT+SH*`, `AT+CAOPEN`, and CNACT-bound HTTPS were added in B05+. The Hologram relay path side-steps this entirely by doing the TLS hop in Hologram's cloud.

### Re-enabling direct HTTPS later (B05+ upgrade)

After a modem upgrade the buoy could POST straight to the Cloudflare Worker, skipping Hologram:

1. Flash B05+ firmware on the SIM7000A (see [firmware-esp32.md](firmware-esp32.md)).
2. In `secrets.h`: uncomment `telemetryUrl = "https://rtk-buoy-proxy.haberkiran.workers.dev/buoy"` and set `telemetrySecret` to the Cloudflare `BUOY_SECRET` value. The `/buoy` path is required — the Worker drops POSTs to other paths with 404.
3. Confirm `[TELEM] POST OK` and check Pages updates.

The Worker accepts both entry points — header check and downstream chain are identical, so the Hologram alert can stay configured as a fallback.

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
