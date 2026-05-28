# Legacy firmware vs `buoy_combo`

The repository keeps older ESP32 sketches under `esp32/legacy/` for reference. **The deployed ocean buoy uses `esp32/buoy_combo/` only.**

---

## What to flash

| Use case | Directory | Connectivity |
|----------|-----------|--------------|
| **RTK wave buoy (production)** | `esp32/buoy_combo/` | SIM7000 LTE, Hologram telemetry, escalated modem recovery |
| Reference / classroom WiFi | `esp32/legacy/` | WiFi NTRIP — not for sealed LTE deployment |

---

## Legacy sketches (summary)

| Sketch | Description |
|--------|-------------|
| `esp32_rtk.ino` | SparkFun-style WiFi NTRIP client → ZED over I2C |
| `esp32_rtk_wifi.ino` | WiFi NTRIP variants |
| `esp32_rtk_mae.ino` | UCSD MAE course WiFi RTK |
| `esp32_polaris.ino` | Point One Polaris caster (HTTP/NTRIP 2.0) |
| `esp32_polaris_wifi.ino` | Polaris + WiFi, chunked HTTP, periodic GGA |
| `esp32_botletic.ino` | Botletics modem experiments |
| `esp32_rtk_button.ino` | Button/UI experiments |
| `botletics_core.ino` | Shared modem helpers (reference) |

Common traits:

- **No** Hologram Cloud Socket telemetry pipeline
- **No** `monitor_connection_health` / RST→PWRKEY escalation
- **WiFi** range limits for field buoys
- Polaris sketches implement **Ntrip/2.0** and **GGA** refresh — different from plain TCP `buoy_combo` caster profile

---

## Hardware overlap

Legacy Polaris docs use the **same ZED UART2 pins** as production:

- ESP32 GPIO **27** (RX2) ← ZED TX1  
- ESP32 GPIO **12** (TX2) → ZED RX1  

See comments in `esp32_polaris_wifi.ino` and [wiring-and-pins.md](wiring-and-pins.md).

---

## When legacy is still useful

- Bench NTRIP over WiFi without burning cellular data
- Comparing Polaris HTTP/2.0 client behavior to LTE plain TCP
- Teaching SparkFun NTRIP examples before introducing modem complexity

---

## Migration notes

If moving from a legacy WiFi sketch to `buoy_combo`:

1. Add SIM7000 shield wiring ([wiring-and-pins.md](wiring-and-pins.md))
2. Create `secrets.h` — NTRIP + Hologram key ([ntrip-and-caster-setup.md](ntrip-and-caster-setup.md))
3. Deploy Cloudflare + Hologram Alert ([../README.md](../README.md))
4. Expect `[TELEM]` / modem health logs not present in WiFi sketches

---

## Related

- [firmware-walkthrough.md](firmware-walkthrough.md) — production code map
- [system-architecture.md](system-architecture.md)
