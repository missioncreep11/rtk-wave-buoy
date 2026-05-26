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
| **docs/** (GitHub Pages) | Optional static public dashboard fed by `docs/data.json` |

## Data flows

### RTK corrections (always on when deployed)

```
NTRIP caster (TCP :2101) <--LTE--> SIM7000 <--UART--> ZED-F9P
```

The ESP32 opens a plain TCP socket to the caster (`BuoyModem::tcpConnectPlain`), sends an NTRIP request, and forwards RTCM bytes to the GPS UART. SSL is disabled for NTRIP (`BOTLETICS_SSL=0`).

### High-rate logging (SD card)

```
ZED-F9P --Qwiic/I2C--> OpenLog Artemis --> microSD (.ubx, IMU CSV)
```

RTCM injection uses UART from ESP32 to ZED-F9P; OLA logs PVT and IMU independently.

### Telemetry — default (local portal)

```
buoy_combo --HTTPS POST--> ngrok --> local_portal/server.py --> browser
```

Configure `telemetryUrl` in `secrets.h` (see [local-portal.md](local-portal.md)). Uses the modem HTTP stack (`AT+SH*`), so NTRIP may pause briefly during each POST then reconnect.

### Telemetry — optional (Hologram cloud only)

```
buoy_combo --TCP--> cloudsocket.hologram.io:9999 --> Hologram dashboard
```

Set `hologramDeviceKey` and leave `telemetryUrl` empty. NTRIP pauses during each cloud message.

### Telemetry — optional (public GitHub Pages)

```
buoy --> Hologram Cloud --> Hologram Route --> GitHub repository_dispatch
      --> workflow updates docs/data.json --> GitHub Pages reads JSON
```

GitHub Pages cannot accept POSTs from the buoy. See [github-pages.md](github-pages.md).

## Firmware loop (`buoy_combo`)

Each `loop()` iteration roughly:

1. Network registration and GPRS (`network_status_check_f`, `enable_gprs_f`)
2. NTRIP connect/retry and RTCM relay (`beginNTRIPClient`, `handleNTRIPData`)
3. Connection health / hangup (`monitor_connection_health`)
4. Telemetry on interval (`post_telemetry_f`, default 60 s)
5. Power and GPS status every 5 s (`print_power_status_f`, PVT print)
6. Graceful shutdown on button (`gracefulShutdown`)

## Repository map (active paths)

```
esp32/buoy_combo/              Main buoy sketch
OpenLog_Artemis_GNSS_Logging_Modified/   OLA firmware
local_portal/                  Flask ingest + dashboard
docs/                          GitHub Pages static site + data.json
.github/workflows/             Pages deploy + Hologram telemetry update
ubx_parsers/                   UBX → CSV tools
python_visualizer/             GNSS / IMU plotting scripts
accelerometer/                 Magnetometer calibration notebook
documentation/                 This documentation set
```
