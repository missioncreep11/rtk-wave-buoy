# System architecture

## Overview

The buoy combines centimeter-level RTK positioning with onboard logging and optional live telemetry over LTE.

| Subsystem | Role |
|-----------|------|
| **ZED-F9P** | RTK GNSS receiver; receives RTCM corrections on UART from ESP32 |
| **ESP32 + SIM7000** | Cellular link, NTRIP client, RTCM forwarder, power monitor, telemetry |
| **OpenLog Artemis (OLA)** | Logs high-rate GNSS UBX and IMU data to microSD |
| **INA228** | Battery/modem bus voltage, current, power (Qwiic on ESP32) |
| **local_portal** | Default live dashboard on your PC (Flask + ngrok) |
| **cloudflare_worker** | Optional HTTPS proxy (buoy → GitHub Actions) |
| **docs/** (GitHub Pages) | Optional static public dashboard fed by `docs/data.json` |

## Data flows

### RTK corrections (always on when deployed)

```
NTRIP caster (TCP :2101) <--LTE--> SIM7000 <--UART--> ZED-F9P
```

The ESP32 opens a plain TCP socket to the caster (`BuoyModem::tcpConnectPlain`), sends an NTRIP request, and forwards RTCM bytes to the GPS UART. SSL is disabled for NTRIP (`BOTLETICS_SSL=0`).

### High-rate logging (SD card)

```
ZED-F9P --Qwiic/I2C--> OpenLog Artemis --> microSD (.ubx)
ICM-20948 --SPI--> OpenLog Artemis --> microSD (imuLogNNNNN.csv)
```

RTCM injection uses UART from ESP32 to ZED-F9P; OLA logs PVT (UBX binary) and IMU (CSV) independently. The OLA writes raw `.ubx` frames to `dataLogNNNNN.ubx` and formatted IMU CSV rows to `imuLogNNNNN.csv`. Post-processing on a PC converts `.ubx` to CSV using parsers in `ubx_parsers/`.

### Telemetry — default (local portal)

```
buoy_combo --plain TCP--> ngrok --> local_portal/server.py --> browser
```

Configure `telemetryUrl` with a `tcp://` scheme in `secrets.h` (see [local-portal.md](local-portal.md)). The firmware parses the URL and opens a raw `AT+CIPSTART` TCP socket, sending a hand-built `HTTP/1.1 POST` over it (`BuoyModem::tcpHttpPost`). This is the **only** telemetry path proven reliable on SIM7000A firmware B03 — the HTTPS AT stacks (`AT+SH*`, `AT+HTTP*+SSL`, `AT+CAOPEN`) all fail once an NTRIP socket is open. NTRIP is paused briefly during each POST then reconnects.

### Telemetry — public GitHub Pages (deployed path)

```
buoy_combo --plain TCP--> cloudsocket.hologram.io:9999
      --Hologram Alert webhook--> Cloudflare Worker
      --GitHub repository_dispatch--> workflow commits docs/data.json --> Pages
```

The buoy sends a small `{"k":"<deviceKey>","d":"<json>"}` message over a plain `AT+CIPSTART` TCP socket to Hologram (`BuoyModem::sendHologramCloudMessage`). Hologram emits an event tagged `_SOCKETAPI_`; a Hologram **Alert** subscribed to that tag fans out via webhook to the Cloudflare Worker (`rtk-buoy-proxy`). The Worker validates a `BUOY_SECRET` header and triggers the `Hologram telemetry → data.json` GitHub Actions workflow, which commits a refreshed `docs/data.json`.

This avoids HTTPS from the modem entirely — important because SIM7000A B03 firmware cannot establish HTTPS once an NTRIP TCP socket is open (see [troubleshooting.md](troubleshooting.md)). Set `hologramDeviceKey` in `secrets.h` and leave `telemetryUrl` empty.

Full setup including the Hologram Alert recipe: [github-pages.md](github-pages.md).

### Telemetry — direct HTTPS (future, B05+ modem)

```
buoy_combo --HTTPS POST--> Cloudflare Worker --> GitHub repository_dispatch --> docs/data.json
```

If/when the modem supports HTTPS, the buoy can POST directly to the same Cloudflare Worker via `AT+HTTPSSL=1` over the legacy SAPBR bearer (`BuoyModem::sapbrHttpsPost`), skipping the Hologram relay. Set `telemetryUrl` to the Worker URL and `telemetrySecret` to match `BUOY_SECRET`. The Worker accepts either entry point — the header check and downstream chain are identical.

## Firmware loop (`buoy_combo`)

Each `loop()` iteration roughly:

1. Network registration and GPRS (`network_status_check_f`, `enable_gprs_f`)
2. NTRIP connect/retry and RTCM relay (`beginNTRIPClient`, `handleNTRIPData`)
3. Connection health / hangup (`monitor_connection_health`)
4. Telemetry on interval (`post_telemetry_f`, default 60 s):
   - Polls ZED-F9P PVT over UART (`myGNSS.getPVT()`) for fix type, RTK status, satellites, lat/lon/alt
   - Reads INA228 over I2C (`getBusVoltage_V()`, `getPower_mW()`) for bus power
   - Reads modem RSSI (`modem.getRSSI()`)
   - Builds JSON payload and sends via either:
     - `modem.sendHologramCloudMessage()` — Hologram relay path (`hologramDeviceKey` set, `telemetryUrl` empty)
     - `modem.httpPostJson()` — direct path (`telemetryUrl` set); dispatches to `tcpHttpPost()` for `tcp://` URLs or `sapbrHttpsPost()` for `https://` URLs
   - NTRIP socket is briefly closed during POST to avoid AT-command conflicts, then reconnects
5. Power and GPS status every 5 s (`print_power_status_f`, PVT print over UART and Bluetooth SPP)
6. Graceful shutdown on button (`gracefulShutdown`)

## Repository map (active paths)

```
esp32/buoy_combo/              Main buoy sketch
OpenLog_Artemis_GNSS_Logging_Modified/   OLA firmware
local_portal/                  Flask ingest + dashboard
cloudflare_worker/             Optional buoy → GitHub proxy
docs/                          GitHub Pages static site + data.json
.github/workflows/             Pages deploy + telemetry update
ubx_parsers/                   UBX → CSV tools
python_visualizer/             GNSS / IMU plotting scripts
accelerometer/                 Magnetometer calibration notebook
documentation/                 This documentation set
```
