# RTK Wave Buoy

Centimeter-level RTK positioning for ocean wave and tide work, with onboard SD logging and optional live telemetry over LTE. Developed at UCSD Scripps Institution of Oceanography and the UC San Diego Department of Mechanical Engineering.

## What this repo contains

- **ESP32 + SIM7000** — NTRIP corrections, RTCM to ZED-F9P, power monitoring, JSON telemetry
- **OpenLog Artemis** — High-rate GNSS UBX and IMU logs to microSD
- **local_portal** — Default live dashboard on your laptop (Flask + ngrok)
- **docs/** — Optional static dashboard on GitHub Pages (Cloudflare Worker → workflow → `data.json`)
- **cloudflare_worker/** — HTTPS proxy for buoy telemetry → GitHub Actions

Full documentation: **[documentation/README.md](documentation/README.md)**

## Quick start (local dashboard)

1. **Firmware** — Copy `esp32/buoy_combo/secrets.h.example` to `secrets.h`, set NTRIP caster credentials, flash `esp32/buoy_combo/buoy_combo.ino`. See [documentation/firmware-esp32.md](documentation/firmware-esp32.md).

2. **Portal** — On your PC:
   ```powershell
   cd local_portal
   pip install -r requirements.txt
   python server.py
   ```
   Open http://127.0.0.1:8080/

3. **LTE ingest** — Run `ngrok http 8080`, set `telemetryUrl` in `secrets.h` to `https://<id>.ngrok-free.app/api/ingest`, re-flash. Serial should show `[TELEM] POST OK`.

Details: [documentation/local-portal.md](documentation/local-portal.md)

## Optional: public GitHub dashboard

No PC or ngrok required after setup. Buoy POSTs to a Cloudflare Worker; a GitHub workflow updates `docs/data.json`; Pages serves the map.

**[documentation/github-pages.md](documentation/github-pages.md)**

## Repository structure

```
esp32/buoy_combo/                          Main buoy firmware (cellular NTRIP + telemetry)
esp32/legacy/                              Older WiFi / Polaris sketches
OpenLog_Artemis_GNSS_Logging_Modified/     OLA GNSS + IMU logging
local_portal/                              Flask ingest + dashboard
cloudflare_worker/                         Optional buoy → GitHub proxy
docs/                                      GitHub Pages site (index.html, data.json)
.github/workflows/                         Pages deploy + telemetry workflow
documentation/                             System documentation (start here)
ubx_parsers/                               UBX → CSV conversion
python_visualizer/                         GNSS / IMU plotting
accelerometer/                             Magnetometer calibration notebook + datasheets
```

## Hardware (summary)

ZED-F9P, SIM7000 LTE shield, ESP32 Thing Plus, OpenLog Artemis, INA228 (Qwiic), Hologram SIM, dual LiPo packs, microSD (FAT32).

Wiring and power: [documentation/hardware.md](documentation/hardware.md)

## Arduino libraries (buoy)

Install from Library Manager: BotleticsSIM7000, Adafruit INA228, SparkFun u-blox GNSS. OLA needs Apollo3 boards **2.2.1** and additional libraries listed in [documentation/firmware-ola.md](documentation/firmware-ola.md).

## Credentials

Create `esp32/buoy_combo/secrets.h` from `secrets.h.example`. **Do not commit** `secrets.h` (gitignored).

## Data after deployment

Remove microSD from the OpenLog, parse `.ubx` with `ubx_parsers/`, plot with `python_visualizer/`. See [documentation/data-processing.md](documentation/data-processing.md).

## Troubleshooting

[documentation/troubleshooting.md](documentation/troubleshooting.md)
