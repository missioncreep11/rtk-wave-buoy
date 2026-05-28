# Deployment checklist

Pre-deployment verification for the LTE **buoy_combo** stack. Theory: [student-guide.md](student-guide.md). Recovery: [failure-paths.md](failure-paths.md).

---

## Firmware and config

- [ ] Flash `esp32/buoy_combo/buoy_combo.ino` (ESP32 Thing Plus)
- [ ] `secrets.h` filled: NTRIP host, port, mount, user/password, Hologram 8-char key
- [ ] `LTE_CATM_BAND` matches carrier (12 AT&T/T-Mobile, 13 Verizon) in `buoy_combo.h`
- [ ] `BOTLETICS_SSL` is `0` in sketch
- [ ] Flash OLA logging firmware; FAT32 microSD seated
- [ ] `docs/config.js` has correct `BUOY_REPO` and branch for GitHub Pages

---

## Cloud path (live dashboard)

- [ ] Hologram SIM active with data
- [ ] Hologram Alert: topic `_SOCKETAPI_`, webhook to `https://<worker>.workers.dev/buoy`, body `<<decdata>>`, header `BUOY_SECRET`
- [ ] Cloudflare Worker env: `GITHUB_OWNER`, `GITHUB_REPO`, `GITHUB_PAT`, `BUOY_SECRET`
- [ ] GitHub Pages enabled from `/docs`
- [ ] Smoke test Worker with sample JSON ([cloudflare_worker/README.md](../cloudflare_worker/README.md))

---

## Bench test (before sealing enclosure)

Run outdoors or strong indoor cell signal, serial **115200**:

- [ ] `[DIAG] CPIN: READY`
- [ ] `[MODEM] LTE CAT-M, band N (boot)` without `SIM failure`
- [ ] `[NET] connected` within ~1–3 min
- [ ] `[GPRS] enabled`
- [ ] `[NTRIP] connected` and `[RTCM]` activity
- [ ] `[GPS] fix=3 rtk=FIXED` (or float → fixed within several minutes)
- [ ] `[TELEM] Hologram OK` at least once
- [ ] `docs/data.json` updates after telemetry (allow workflow delay)
- [ ] INA228 `bus_v` reasonable on **battery** power (not USB-only bench)

---

## Hardware and mechanical

- [ ] Both **3S2P 75 Wh** packs charged and matched SOC; **parallel** wiring (+ to +, − to −)
- [ ] Pack bus **10.8–12.8 V** at rest; INA228 / OKI-78SR-3.3 installed per [hardware-spec.md](hardware-spec.md)
- [ ] GNSS antenna with clear sky view (not under deck/roof for test)
- [ ] ZED UART2 wiring: ESP32 GPIO 12/27 ↔ F9P UART1 ([wiring-and-pins.md](wiring-and-pins.md))
- [ ] Qwiic/SD connections secure on OLA
- [ ] Modem antenna connected; SIM seated
- [ ] Shutdown button (GPIO 0) does not short unless intended
- [ ] Enclosure waterproofing plan documented for your deployment (project-specific)

---

## Operational expectations

- [ ] Telemetry closes NTRIP briefly every `TELEMETRY_INTERVAL_MS` — brief RTK interruption is normal
- [ ] After modem recover, allow up to **10–15 min** cooldown before next PWRKEY cycle
- [ ] Carrier outages (e.g. Hologram US2) may prevent re-register — check [status.hologram.io](https://status.hologram.io)
- [ ] SD logs continue on OLA even if LTE telemetry fails

---

## Post-deployment

- [ ] Confirm latest point on GitHub Pages map
- [ ] Retrieve SD after recovery for UBX/IMU analysis ([data-formats.md](data-formats.md))
- [ ] Archive serial log if investigating dropout ([failure-paths.md](failure-paths.md) example cascade)

---

## Related

- [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md)
- [../README.md](../README.md) — full setup sections
- [learning-path.md](learning-path.md)
