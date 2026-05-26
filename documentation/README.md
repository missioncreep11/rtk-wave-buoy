# RTK Wave Buoy — Documentation

Documentation for the current field system: ESP32 + SIM7000 buoy firmware, OpenLog Artemis logging, local telemetry portal, and optional GitHub Pages dashboard.

## Contents

| Document | Description |
|----------|-------------|
| [architecture.md](architecture.md) | System overview, data flows, telemetry paths |
| [hardware.md](hardware.md) | Components, wiring, power |
| [firmware-esp32.md](firmware-esp32.md) | `esp32/buoy_combo` — NTRIP, LTE, INA228, telemetry |
| [firmware-ola.md](firmware-ola.md) | OpenLog Artemis GNSS + IMU logging |
| [local-portal.md](local-portal.md) | Local dashboard via Flask + ngrok (default telemetry) |
| [github-pages.md](github-pages.md) | Optional public dashboard (Hologram → GitHub) |
| [data-processing.md](data-processing.md) | UBX parsers, Python visualizers, magnetometer calibration |
| [legacy-firmware.md](legacy-firmware.md) | Older ESP32 sketches in `esp32/legacy/` |
| [rtk-primer.md](rtk-primer.md) | Short RTK background |
| [troubleshooting.md](troubleshooting.md) | Common field and software issues |

## Quick links

- **Flash buoy firmware:** [firmware-esp32.md](firmware-esp32.md)
- **Live map on your laptop:** [local-portal.md](local-portal.md)
- **Public map (no PC):** [github-pages.md](github-pages.md)
