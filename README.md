# RTK Wave Buoy

Centimeter-level RTK positioning for ocean wave and tide work, with onboard SD logging and live telemetry over LTE. Developed at UCSD Scripps Institution of Oceanography and the UC San Diego Department of Mechanical Engineering.

```
NTRIP caster (TCP :2101) <--LTE--> SIM7000 <--UART--> ZED-F9P
                                |
                           ESP32 Thing Plus
                                |
                           OpenLog Artemis --> microSD (.ubx + IMU .csv)

 Telemetry path:
 Buoy --plain TCP--> Hologram Cloud Socket
      --Alert webhook--> Cloudflare Worker
      --repository_dispatch--> GitHub Actions
      --> docs/data.json --> GitHub Pages
```

## Repository structure

```
esp32/buoy_combo/                          Main buoy firmware (NTRIP + telemetry)
esp32/legacy/                              Older WiFi / Polaris sketches (reference only)
OpenLog_Artemis_GNSS_Logging_Modified/     OLA high-rate GNSS + IMU logging
cloudflare_worker/                         Telemetry proxy (Hologram → GitHub)
ubx_parsers/                               UBX → CSV conversion
visualizer/                                GNSS / IMU / altitude plotting
accelerometer/                             Magnetometer calibration notebook + datasheets
docs/                                      GitHub Pages dashboard (index.html, data.json)
.github/workflows/                         Pages deploy + telemetry update workflows
matlab/                                    Original MATLAB analysis scripts
```

## Hardware

| Part | Function |
|------|----------|
| SparkFun ZED-F9P-02B | RTK GNSS receiver |
| Botletics SIM7000 LTE shield | Cellular data |
| SparkFun ESP32 Thing Plus | Main buoy controller |
| SparkFun OpenLog Artemis | SD logging, ICM-20948 IMU |
| Adafruit INA228 (Qwiic) | Bus voltage, current, power |
| u-blox ANN-MB1 antenna + ground plane | GNSS antenna |
| Hologram SIM | LTE data plan |
| LiPo batteries | Separate packs for OLA/GPS and ESP32/modem |
| microSD (FAT32) | OLA log storage |

### Wiring

```
ZED-F9P ──Qwiic/I2C──> OpenLog Artemis (address 0x42)
ZED-F9P ──UART───────> ESP32 (RTCM injection)
INA228 ──Qwiic/I2C──> ESP32 (SDA 23, SCL 22)
```

### Power

- **6000 mAh** — typically OpenLog Artemis + GPS stack
- **850 mAh** — typically ESP32 + cellular modem

| Component | Approx. draw |
|-----------|-------------|
| ZED-F9P | ~85 mA |
| GNSS antenna | ~15 mA |
| SIM7000 | ~110–170 mA |
| ESP32 + OLA | varies with logging rate |

On USB bench power, INA228 may not sit on the active battery rail — serial will show a bench/USB message.

### ZED-F9P LED

| LED | Meaning |
|-----|---------|
| Solid on | 3D fix, no RTK |
| Blinking | RTK float |
| Off | RTK fixed (cm-level) |

### Cellular band

`LTE_CATM_BAND` in `buoy_combo.h` defaults to **12** (US AT&T/T-Mobile). Set to **13** for Verizon-dominated deployments.

## Quick start

### 1. Flash the buoy (ESP32)

**Board:** SparkFun ESP32 Thing Plus (or compatible)

**Libraries** (install from Arduino Library Manager):
- BotleticsSIM7000
- Adafruit INA228
- SparkFun u-blox GNSS

`BOTLETICS_SSL` is forced to `0` in the sketch so NTRIP uses plain TCP on port 2101.

```bash
cp esp32/buoy_combo/secrets.h.example esp32/buoy_combo/secrets.h
# edit secrets.h with your NTRIP caster credentials and Hologram device key
```

Upload `esp32/buoy_combo/buoy_combo.ino` via Arduino IDE.

**Configuration (`secrets.h`):**

| Setting | Purpose |
|---------|---------|
| `casterHost`, `casterPort`, `mountPoint` | NTRIP caster |
| `casterUser`, `casterUserPW` | NTRIP credentials |
| `hologramDeviceKey` | 8-char Hologram Cloud Socket key (Router Credentials in Hologram dashboard) |
| `TELEMETRY_INTERVAL_MS` | POST interval (default 60000 ms) |

### 2. Flash the logger (OpenLog Artemis)

**Board:** SparkFun RedBoard Artemis ATP (Apollo3 package **2.2.1**)

**Additional board URL:**
```
https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
```

**Libraries:** SparkFun u-blox GNSS, ICM-20948, SdFat, MicroNMEA

Upload `OpenLog_Artemis_GNSS_Logging_Modified/` via Arduino IDE. On Apple Silicon Macs, use `upload_ola_firmware.py`.

**SD card:** Format FAT32. Reseat if mount errors appear.

**Menu** (press any key in serial monitor at 115200 baud):

| Key | Action |
|-----|--------|
| 1–5 | Configure logging, GNSS, IMU, Qwiic, power |
| f | New log file |
| g | Reset GNSS |
| r | Reset settings |
| q | Quit — close log and power down |
| x | Return to logging |

At boot the serial monitor shows the `dataLogNNNNN.ubx` and `imuLogNNNNN.csv` file numbers.

### 3. Deploy the Cloudflare Worker

See [`cloudflare_worker/README.md`](cloudflare_worker/README.md) for full setup.

1. Create a Worker in the Cloudflare Dashboard (do NOT connect GitHub).
2. Paste the contents of [`cloudflare_worker/worker.js`](cloudflare_worker/worker.js).
3. Set environment variables:

| Variable | Purpose |
|----------|---------|
| `GITHUB_OWNER` | GitHub username or org |
| `GITHUB_REPO` | Repository name |
| `GITHUB_PAT` | GitHub PAT with `repo` scope (encrypted) |
| `BUOY_SECRET` | Shared secret for header auth (encrypted) |

The Worker only accepts POSTs at the `/buoy` path — requests to the bare URL return `404 Not Found`.

### 4. Configure the Hologram Alert

In the Hologram dashboard, create an **Alert**:

| Field | Value |
|-------|-------|
| Trigger / Subscribed topic | `_SOCKETAPI_` |
| Scope | The SIM group or device containing your buoy |
| Notification method | **Webhook** |
| Destination URL | `https://YOUR-WORKER.workers.dev/buoy` |
| Message payload for POST | `<<decdata>>` |
| Header — Key | `BUOY_SECRET` |
| Header — Value | Same value as the Cloudflare `BUOY_SECRET` env var |
| Click **Add JSON content-type header** | Adds `Content-Type: application/json` |

### 5. Enable GitHub Pages

1. Repo **Settings → Pages**
2. Source: **Deploy from a branch**, branch: default, folder **`/docs`**
3. Edit [`docs/config.js`](docs/config.js) to set your repo owner and branch:

```javascript
window.BUOY_REPO = "YOUR_GITHUB_USER/rtk-wave-buoy";
window.BUOY_BRANCH = "Base+PowerLog";
```

### 6. Verify

Flash the buoy and watch serial (115200 baud):

```
[NET] connected
[GPRS] enabled
[NTRIP] connected
[GPS] fix=3 rtk=FIXED sats=...
[TELEM] Hologram cloud...
[TELEM] Hologram OK
```

Allow 1–3 minutes for first LTE registration outdoors or near a window. Brief NTRIP reconnect after each telemetry send is normal.

## Telemetry pipeline details

### Data flow

```
Buoy --plain TCP (AT+CIPSTART)--> cloudsocket.hologram.io:9999
      --> Hologram Alert (_SOCKETAPI_ tag)
      --webhook--> Cloudflare Worker (/buoy path, BUOY_SECRET check)
      --GitHub repository_dispatch--> hologram-telemetry.yml workflow
      --> commits docs/data.json --> GitHub Pages
```

The buoy sends `{"k":"<deviceKey>","d":"<jsonTelemetry>"}\n\n` over a plain TCP socket via `BuoyModem::sendHologramCloudMessage`. Hologram emits a `_SOCKETAPI_` event; the Alert fans out to the Worker, which validates the secret and triggers a `repository_dispatch`. The workflow commits the parsed telemetry to `docs/data.json`, and GitHub Pages serves the dashboard.

### Telemetry JSON fields

```json
{
  "id": "353026070123456",
  "fix": 3,
  "rtk": "FIXED",
  "sats": 14,
  "lat": 32.8651,
  "lon": -117.2573,
  "alt_m": 12.45,
  "bus_v": 3.921,
  "power_mw": 450.2,
  "rssi": 20,
  "ntrip": 1
}
```

| Field | Source | Description |
|-------|--------|-------------|
| `id` | Modem IMEI | Device identifier |
| `fix` | ZED-F9P (`getFixType`) | 0=no fix, 2=2D, 3=3D |
| `rtk` | ZED-F9P (`getCarrierSolutionType`) | `none`, `float`, `FIXED` |
| `sats` | ZED-F9P (`getSIV`) | Satellites in view |
| `lat`, `lon` | ZED-F9P | Decimal degrees |
| `alt_m` | ZED-F9P (`getAltitudeMSL`) | Mean sea level altitude (m) |
| `bus_v` | INA228 | Bus voltage (V) |
| `power_mw` | INA228 | Power draw (mW) |
| `rssi` | SIM7000 (`AT+CSQ`) | Signal strength (0–31) |
| `ntrip` | Firmware flag | 1 if NTRIP connected |

### Firmware loop

Each `loop()` iteration:

1. Network registration and GPRS (`network_status_check_f`, `enable_gprs_f`)
2. NTRIP connect/retry and RTCM relay (`beginNTRIPClient`, `handleNTRIPData`)
3. Connection health monitoring (`monitor_connection_health`)
   - CGREG checked every 30 s — transient drops ignored while RTCM flows
   - GPRS refreshed if no data for 5 min (`DATA_PATH_STALE_MS`)
   - Modem hard-reset after 10 consecutive NTRIP failures
4. Telemetry on interval (`post_telemetry_f`, default 60 s):
   - Polls ZED-F9P PVT, INA228 power, modem RSSI
   - Sends Hologram Cloud Socket message
   - NTRIP socket closed briefly during POST to avoid AT conflicts
5. Power + GPS status print every 5 s
6. Graceful shutdown on GPIO 0 button

### Serial log legend

| Tag | Meaning |
|-----|---------|
| `[MODEM]` / `[DIAG]` | LTE configuration and registration |
| `[NET]` | CSQ, CGREG |
| `[GPRS]` | Packet data attached |
| `[NTRIP]` | Caster connect / fail |
| `[RTCM]` | Throughput to GPS |
| `[GPS]` | fix type, RTK state, satellites |
| `[PWR]` | INA228 readings or bench note |
| `[TELEM]` | Hologram upload result |
| `[HEALTH]` | Connection health checks |
| `[DATA]` | Data path invalidation |
| `[HOLO]` | Hologram TCP socket details |

## Data processing

After field logging, remove the microSD from the OpenLog and process on a PC.

### UBX to CSV

Binary `.ubx` logs convert with scripts in [`ubx_parsers/`](ubx_parsers/):

| Script | Notes |
|--------|-------|
| `ubx_parser.py` | Original parser |
| `v2_ubx_parser.py` | Common choice — edit filename at bottom |
| `v3_ubx_parser.py` | Newer message coverage |

```bash
pip install pyubx2
python ubx_parsers/v2_ubx_parser.py
```

**CSV columns** (from `v2_ubx_parser.py`):

`timestamp`, `year`, `month`, `day`, `hour`, `minute`, `second`, `latitude`, `longitude`, `altitude_msl`, `altitude_ellipsoid`, `horizontal_accuracy`, `vertical_accuracy`, `fix_type`, `carrier_solution`, `num_satellites`, `pDOP`, `speed_2d`, `heading`, `flags`

**carrier_solution:** 0=none, 1=float, 2=fixed

### Visualization

Scripts in [`visualizer/`](visualizer/):

| Script | Purpose |
|--------|---------|
| `gnss_visualizer.py` | High-precision GNSS/RTK visualization and analysis (2D map, 3D trajectory, precision histogram) |
| `imu_visualizer.py` | IMU sensor data (accelerometer, gyroscope, magnetometer, temperature) |
| `altitude_visualizer.py` | Altitude time-series analysis |

```bash
cd visualizer
pip install -r requirements.txt
python gnss_visualizer.py path/to/parsed_positions.csv
python imu_visualizer.py path/to/imuLog.csv
```

### Magnetometer calibration

Open [`accelerometer/magcal_notebook.ipynb`](accelerometer/magcal_notebook.ipynb) in VS Code or Jupyter. Covers hard-iron offset and ellipsoid soft-iron correction for ICM-20948 magnetometer data.

Datasheets in `accelerometer/`: `ICM-20948-Datasheet-v1.3.pdf`, `AK09916-Magnetometer-Datasheet.pdf`.

## RTK reference

RTK uses **carrier phase** measurements (~mm noise) instead of code pseudorange (~m noise). Corrections from a nearby NTRIP caster cancel ionospheric, tropospheric, orbital, and clock errors that are correlated over distance.

| State | Accuracy | Meaning |
|-------|----------|---------|
| No RTK | ~m | Standard GNSS |
| RTK float | ~10–50 cm | Ambiguities not fully resolved |
| RTK fixed | ~1–3 cm | Integer ambiguities resolved |

The ZED-F9P reports carrier solution in telemetry (`rtk`: `none`, `float`, `FIXED`). Convergence after RTCM starts can take several minutes.

## Troubleshooting

### LTE / modem

| Symptom | Fix |
|---------|-----|
| `[NET] CSQ=99 CGREG=0` | Wait 1–3 min; move outdoors; check SIM active on Hologram |
| `[NET] registration lost` | Modem dropped LTE; auto-recovers |
| CPIN not READY | Reseat SIM; power cycle 30 s |
| Stays on wrong band | Set `LTE_CATM_BAND` to `13` for Verizon in `buoy_combo.h` |

### NTRIP

| Symptom | Fix |
|---------|-----|
| TCP connect failed | Check `casterHost`, port **2101**, credentials in `secrets.h` |
| HTTP error from caster | Wrong mountpoint or user/password |
| No RTCM / no RTK | Confirm `[NTRIP] connected` and `[RTCM]` activity; clear sky view |
| Drops after telemetry | Expected brief reconnect |

### GPS / RTK

| Symptom | Fix |
|---------|-----|
| fix=0 or low sats | Antenna view; check UART wiring |
| fix=3 but rtk=none | RTCM not arriving; wait several minutes after NTRIP up |
| RTK float only | Normal before fix; multipath or long baseline |

### Telemetry (Hologram path)

| Symptom | Fix |
|---------|-----|
| `[TELEM] Hologram failed` | Cloud Socket didn't return `[0,0]`. Confirm `hologramDeviceKey` matches Hologram dashboard |
| `[TELEM] TCP connect failed` | LTE registration lost; let firmware auto-recover |
| Hologram Event exists but `matched_rules: []` | Alert not subscribed to `_SOCKETAPI_` |
| Alert log shows HTTP 401 from Worker | `BUOY_SECRET` header doesn't match Cloudflare env var |
| Alert log shows HTTP 400 from Worker | Body template should be `<<decdata>>` |
| Webhook returns 200, no workflow run | GitHub PAT missing `repo` scope, or `GITHUB_OWNER`/`GITHUB_REPO` wrong |
| Workflow ran, `data.json` unchanged | Payload missing `id`, `fix`, `rtk`, `lat`, `lon` |
| Raw JSON stale in browser | Hard-refresh; URL-encode `+` in branch name (`Base%2BPowerLog`) |
| Pages 404 | Enable Pages from `/docs` on default branch |

### OpenLog / SD

| Symptom | Fix |
|---------|-----|
| GPS not on OLA | Qwiic cable; I2C address 0x42 |
| SD errors | FAT32 format; reseat card; free space |
| No IMU log | Enable via OLA menu option 3 |

### Power

| Symptom | Fix |
|---------|-----|
| INA228 not found | Qwiic cable to ESP32 |
| Bench/USB message | Expected when not on battery rail |

### General

- Do not commit `secrets.h`
- Arduino: Apollo3 **2.2.1** for OLA
