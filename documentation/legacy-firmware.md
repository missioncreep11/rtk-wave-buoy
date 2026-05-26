# Legacy ESP32 firmware

Older sketches live in [`esp32/legacy/`](../esp32/legacy/). They are **not** the primary field path; use [`esp32/buoy_combo/`](../esp32/buoy_combo/) for cellular NTRIP, INA228 power logging, and telemetry.

## Sketches (reference)

| File | Description |
|------|-------------|
| `buoy_combo.ino` / `buoy_combo.h` | Pre-refactor copy superseded by `esp32/buoy_combo/` |
| `esp32_rtk_wifi.ino` | WiFi NTRIP (needs partition **No OTA (Large APP)**) |
| `esp32_polaris_wifi.ino` | Point One Polaris over WiFi |
| `esp32_polaris.ino` | Polaris over cellular |
| `esp32_rtk.ino`, `esp32_rtk_button.ino` | Earlier RTK experiments |
| `esp32_botletic.ino`, `botletics_core.ino` | Botletics examples |

WiFi sketches expect `ssid` / `password` in `secrets.h`. Polaris sketches use `polaris.pointonenav.com` style casters.

## When to use legacy code

- Bench testing with WiFi instead of LTE
- Comparing behavior with course-era Polaris configuration
- Recovering an old upload recipe — prefer current docs: [firmware-esp32.md](firmware-esp32.md)
