# Documentation index

**New here?** Start with [learning-path.md](learning-path.md).

## Core guides (`documentation/`)

| Document | Audience | Contents |
|----------|----------|----------|
| [learning-path.md](learning-path.md) | Everyone | Suggested reading order by goal |
| [hardware-spec.md](hardware-spec.md) | Build / review | **150 Wh** battery spec, KiCad wiring diagram |
| [student-guide.md](student-guide.md) | Students | Power, RTK, fusion, pipeline theory, glossary |
| [system-architecture.md](system-architecture.md) | Students / integrators | Block diagram, dual data paths (LTE vs SD) |
| [wiring-and-pins.md](wiring-and-pins.md) | Build | ESP32 GPIO, UART, I2C, power bus |
| [firmware-walkthrough.md](firmware-walkthrough.md) | Firmware readers | `setup`/`loop`, function map |
| [data-formats.md](data-formats.md) | Analysis | Telemetry JSON, UBX/CSV, IMU CSV, `data.json` |
| [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md) | Deploy | `secrets.h`, caster, SSL off |
| [at-command-primer.md](at-command-primer.md) | Debug | CGREG, CSQ, CPIN, CIP in logs |
| [failure-paths.md](failure-paths.md) | Field / debug | RST → PWRKEY recovery, serial tags |
| [deployment-checklist.md](deployment-checklist.md) | Deploy | Pre-float verification |
| [legacy-vs-buoy-combo.md](legacy-vs-buoy-combo.md) | Reference | Do not flash WiFi legacy for LTE buoy |

## Elsewhere in the repo

| Location | Contents |
|----------|----------|
| [../README.md](../README.md) | Quick start, hardware, flash OLA/ESP32, troubleshooting |
| [../cloudflare_worker/README.md](../cloudflare_worker/README.md) | Telemetry Worker deploy |
| [../Visualizer/README.md](../Visualizer/README.md) | GNSS / IMU / altitude plotting |
| [../ubx_parsers/](../ubx_parsers/) | `v3_ubx_parser.py` and sample logs |
| [../docs/](../docs/) | GitHub Pages dashboard (`index.html`, `data.json`) |

## By task

| Task | Read |
|------|------|
| Understand the system | [learning-path.md](learning-path.md) → [system-architecture.md](system-architecture.md) |
| Wire the board | [hardware-spec.md](hardware-spec.md) → [wiring-and-pins.md](wiring-and-pins.md) |
| Debug LTE / NTRIP | [failure-paths.md](failure-paths.md) → [at-command-primer.md](at-command-primer.md) |
| Process SD logs | [data-formats.md](data-formats.md) → [../Visualizer/README.md](../Visualizer/README.md) |
