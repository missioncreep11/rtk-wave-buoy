# ESP32 buoy firmware (`buoy_combo`)

Primary sketch: [`esp32/buoy_combo/`](../esp32/buoy_combo/).

## What it does

- Registers on Hologram LTE (CAT-M, configurable band)
- Connects to an NTRIP caster and forwards RTCM to the ZED-F9P
- Polls GNSS PVT over UART; prints fix / RTK / satellite count
- Logs INA228 power when on battery
- Posts JSON telemetry on an interval (HTTP or Hologram cloud)
- Bluetooth SPP mirror of key serial lines
- Graceful shutdown on GPIO 0 button

## Arduino setup

1. Board: **SparkFun ESP32 Thing Plus** (or compatible)
2. Install libraries: BotleticsSIM7000, Adafruit INA228, SparkFun u-blox GNSS
3. Copy [`secrets.h.example`](../esp32/buoy_combo/secrets.h.example) to `secrets.h` (gitignored)
4. Upload `buoy_combo.ino`

`BOTLETICS_SSL` is forced to `0` in the sketch so NTRIP uses plain TCP on port 2101.

## Configuration (`secrets.h`)

| Setting | Purpose |
|---------|---------|
| `casterHost`, `casterPort`, `mountPoint` | NTRIP caster |
| `casterUser`, `casterUserPW` | NTRIP credentials |
| `hologramDeviceKey` | 8-char Hologram Cloud Socket key (Router Credentials in Hologram dashboard) — used for the Hologram relay path |
| `telemetryUrl` | Direct ingest URL — `tcp://host:port/path` for ngrok local portal, `https://...` for direct Cloudflare Worker (B05+ modem only) |
| `telemetrySecret` | Shared secret used as `BUOY_SECRET` header on direct HTTP(S) POSTs (the `telemetryUrl` path only). Not used on the Hologram relay path |
| `TELEMETRY_INTERVAL_MS` | POST interval (default 60000 ms) |

**Pick one telemetry path:**

- **GitHub Pages via Hologram relay** (recommended, works on B03): set `hologramDeviceKey`, leave `telemetryUrl` empty — see [github-pages.md](github-pages.md)
- **Local dashboard**: set `telemetryUrl` to `tcp://<ngrok-host>:<port>/api/ingest`, leave `hologramDeviceKey` empty — see [local-portal.md](local-portal.md)
- **GitHub Pages via direct HTTPS** (needs B05+ modem): set `telemetryUrl` to `https://<worker>.workers.dev/buoy` and `telemetrySecret` to match `BUOY_SECRET` on Cloudflare (the Worker rejects POSTs to any other path with 404)

`post_telemetry_f()` checks both: if `hologramDeviceKey` is set and `telemetryUrl` is empty, it takes the Hologram branch (`sendHologramCloudMessage`); otherwise it dispatches through `httpPostJson()`.

## Key implementation details

### `BuoyModem`

Wraps SIM7000 for:

- `configureNetwork()` — LTE CAT-M, band, CGATT, operator search
- `tcpConnectPlain` / `tcpSendPlain` — NTRIP without SSL (plain TCP `AT+CIPSTART`/`AT+CIPSEND`)
- `sendHologramCloudMessage` — Hologram Cloud Socket relay (TCP to `cloudsocket.hologram.io:9999`); used when `hologramDeviceKey` is set and `telemetryUrl` is empty. Sends `{"k":"<key>","d":"<json>"}\n\n`; expects `[0,0]` back.
- `httpPostJson(url, body)` — direct POST path; dispatches by URL scheme:
  - `tcp://...` → `tcpHttpPost()` — raw HTTP/1.1 over `AT+CIPSTART` TCP socket (used for ngrok local portal)
  - `https://...` → `sapbrHttpsPost()` — HTTPS via legacy SAPBR bearer + `AT+HTTPSSL=1` (B05+ modem only)
  - `http://...` → Botletics `postData()` — unencrypted HTTP
  Sends `BUOY_SECRET` header when `telemetrySecret` is set

### NTRIP

`beginNTRIPClient()` opens TCP, sends NTRIP GET with credentials, validates HTTP 200/ICY, then `handleNTRIPData()` pumps RTCM to GPS.

### Telemetry data sources

The ESP32 polls three hardware sources each telemetry cycle (`post_telemetry_f()`):

| Source | Interface | Fields |
|--------|-----------|--------|
| ZED-F9P | UART (`myGNSS.getPVT()`) | `fix`, `rtk`, `sats`, `lat`, `lon`, `alt_m` |
| INA228 | I2C (Qwiic) | `bus_v`, `power_mw` |
| SIM7000 modem | AT commands | `id` (IMEI), `rssi`, `ntrip` |

### Telemetry JSON fields

`id` (IMEI), `fix`, `rtk`, `sats`, `lat`, `lon`, `alt_m`, `bus_v`, `power_mw`, `rssi`, `ntrip`

## Serial log legend

| Tag | Meaning |
|-----|---------|
| `[MODEM]` / `[DIAG]` | LTE configuration and registration |
| `[NET]` | CSQ, CGREG |
| `[GPRS]` | Packet data attached |
| `[NTRIP]` | Caster connect / fail |
| `[RTCM]` | Throughput to GPS |
| `[GPS]` | fix type, RTK state, satellites |
| `[PWR]` | INA228 readings or bench note |
| `[TELEM]` | HTTP or Hologram upload result |

USB serial: **115200**. Modem UART runs at **9600** after init.

## Deployment checks

Allow 1–3 minutes for first LTE registration outdoors or near a window.

Success pattern (Hologram relay path):

```
[NET] connected
[GPRS] enabled
[NTRIP] connected
[GPS] fix=3 rtk=FIXED sats=...
[TELEM] Hologram cloud...
[TELEM] Hologram OK
```

For the local-portal path the last two lines instead read `[TELEM] HTTP POST...` / `[TELEM] POST OK`.

Brief NTRIP reconnect after each telemetry send is normal.
