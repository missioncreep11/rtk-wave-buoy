# Documentation learning path

Suggested reading order for someone new to the RTK Wave Buoy. Each step links to existing docs — no separate lab workbook.

## Before you touch hardware

1. [student-guide.md](student-guide.md) — §1 Power, §2 RTK theory (skim if you already know GNSS)
2. [system-architecture.md](system-architecture.md) — how ESP32, OLA, modem, and ZED fit together

## Build and deploy

3. [../README.md](../README.md) — Quick start (ESP32, OLA, Cloudflare, Hologram, GitHub Pages)
4. [wiring-and-pins.md](wiring-and-pins.md) — pin table before wiring or debugging UART/I2C
5. [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md) — `secrets.h`, caster, `BOTLETICS_SSL=0`
6. [deployment-checklist.md](deployment-checklist.md) — pre-deployment checks

## Firmware and logs

7. [firmware-walkthrough.md](firmware-walkthrough.md) — `setup()` / `loop()` and main functions
8. [at-command-primer.md](at-command-primer.md) — what `CGREG`, `CSQ`, etc. mean in serial output
9. [failure-paths.md](failure-paths.md) — recovery modes when LTE or NTRIP fails

## Data and analysis

10. [data-formats.md](data-formats.md) — telemetry JSON, UBX/CSV, IMU CSV, `docs/data.json`
11. [../README.md](../README.md) — Data processing (UBX parse, Visualizer)
12. [../Visualizer/README.md](../Visualizer/README.md) — post-processing scripts

## Reference only

- [legacy-vs-buoy-combo.md](legacy-vs-buoy-combo.md) — do **not** flash legacy sketches for the LTE buoy
- [../cloudflare_worker/README.md](../cloudflare_worker/README.md) — Worker deploy detail
- [student-guide.md](student-guide.md) — §4–6 waves, components, glossary

## By goal

| Goal | Start here |
|------|------------|
| Understand the system | [system-architecture.md](system-architecture.md) → [student-guide.md](student-guide.md) |
| Flash and get RTK on serial | [../README.md](../README.md) Quick start → [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md) |
| Debug modem/NTRIP | [failure-paths.md](failure-paths.md) → [at-command-primer.md](at-command-primer.md) |
| Process SD logs | [data-formats.md](data-formats.md) → [../Visualizer/README.md](../Visualizer/README.md) |
| Live dashboard | [data-formats.md](data-formats.md) (`docs/data.json`) → [../README.md](../README.md) §5–6 |
