# Data formats reference

Field definitions for live telemetry, GitHub Pages storage, SD logs, and parsed CSV. Processing steps: [../README.md](../README.md) Data processing, [../Visualizer/README.md](../Visualizer/README.md).

---

## Live telemetry (ESP32 → Hologram)

Plain JSON inside Hologram Cloud Socket wrapper:

```json
{"k":"<8-char device key>","d":"{\"id\":\"<IMEI>\", ... }"}
```

### Inner payload fields

| Field | Type | Source | Description |
|-------|------|--------|-------------|
| `id` | string | Modem IMEI | Device ID on dashboard |
| `fix` | uint | ZED `getFixType()` | 0=none, 2=2D, 3=3D |
| `rtk` | string | `getCarrierSolutionType()` | `none`, `float`, `FIXED` |
| `sats` | uint | `getSIV()` | Satellites in view |
| `lat` | float | ZED | Decimal degrees (if fix ≥ 2) |
| `lon` | float | ZED | Decimal degrees |
| `alt_m` | float | `getAltitudeMSL()` | MSL altitude (m) |
| `bus_v` | float | INA228 | Pack bus voltage (V, ~10.8–12.8); 0 on bench/USB note path |
| `power_mw` | float | INA228 | Bus power (mW) |
| `rssi` | uint | `AT+CSQ` | 0–31 (99 = unknown) |
| `ntrip` | 0/1 | Firmware | 1 if NTRIP TCP connected |

If fix &lt; 2, `lat`/`lon`/`alt_m` are omitted from the JSON.

Built in `post_telemetry_f()` (`buoy_combo.h`).

---

## GitHub Pages (`docs/data.json`)

Updated by GitHub Actions after Cloudflare `repository_dispatch`. Consumed by `docs/index.html` + `docs/config.js`.

Typical fields (workflow may add metadata):

| Field | Meaning |
|-------|---------|
| `id`, `fix`, `rtk`, `sats`, `lat`, `lon`, `alt_m` | Same as telemetry |
| `bus_v`, `power_mw`, `rssi`, `ntrip` | Power and link |
| `received` / `updated` | Server timestamp string |
| `age_sec` | Seconds since last update (dashboard stale warning &gt; 180 s) |

Raw URL pattern (set `docs/config.js`):

```text
https://raw.githubusercontent.com/<owner>/<repo>/<branch>/docs/data.json
```

---

## SD: UBX (`dataLogNNNNN.ubx`)

| Property | Value |
|----------|--------|
| Writer | OpenLog Artemis GNSS logging firmware |
| Format | u-blox UBX binary |
| Key messages | `NAV-PVT`, `NAV-HPPOSLLH` (high-precision lat/lon/height) |

### Parse to CSV

```bash
pip install pyubx2
python ubx_parsers/v3_ubx_parser.py path/to/dataLog00020.ubx
# writes path/to/dataLog00020_parsed.csv by default
```

Older scripts: `ubx_parser.py`, `v2_ubx_parser.py` — prefer **v3**.

### Parsed CSV columns (`v3_ubx_parser.py`)

| Column | Description |
|--------|-------------|
| `timestamp` | Combined time key |
| `year` … `second` | UTC from NAV-PVT |
| `latitude`, `longitude` | Degrees (HPPOSLLH + hp when available) |
| `altitude_msl`, `altitude_ellipsoid` | meters |
| `horizontal_accuracy`, `vertical_accuracy` | meters |
| `fix_type` | u-blox fix type |
| `carrier_solution` | 0=none, 1=float, 2=fixed |
| `num_satellites` | SV count |
| `pDOP` | dilution of precision |
| `speed_2d`, `heading` | motion |
| `vel_north`, `vel_east`, `vel_down` | m/s |
| `flags`, `source` | PVT vs HPPOSLLH source tag |

Visualizer: `Visualizer/gnss_visualizer.py`, `altitude_visualizer.py`.

---

## SD: IMU CSV (`imuLogNNNNN.csv`)

| Property | Value |
|----------|--------|
| Writer | OLA when IMU logging enabled in menu |
| Header | Built at file create — columns depend on enabled sensors |

Base columns always include:

| Column | Description |
|--------|-------------|
| `Timestamp` | Log time |
| `Sensor` | Sensor id string |

Optional blocks (if enabled in OLA settings):

| Columns | Unit / note |
|---------|-------------|
| `AccX`, `AccY`, `AccZ` | Accelerometer (raw units per OLA firmware) |
| `GyrX`, `GyrY`, `GyrZ` | Gyroscope |
| `MagX`, `MagY`, `MagZ` | Magnetometer |
| `Temp` | Temperature |

`Visualizer/imu_visualizer.py` converts accel from milli-g to m/s².

Magnetometer calibration: [../accelerometer/magcal_notebook.ipynb](../accelerometer/magcal_notebook.ipynb).

---

## Fix / RTK enums (u-blox)

| `fix` / `fix_type` | Meaning |
|--------------------|---------|
| 0 | No fix |
| 2 | 2D |
| 3 | 3D |

| `carrier_solution` / `rtk` | Meaning |
|----------------------------|---------|
| 0 / `none` | No RTK |
| 1 / `float` | RTK float |
| 2 / `FIXED` | RTK fixed |

---

## Related

- [system-architecture.md](system-architecture.md) — which path produces which file
- [student-guide.md](student-guide.md) §4 — wave analysis from position/IMU
- [../README.md](../README.md) — Telemetry pipeline, Data processing
