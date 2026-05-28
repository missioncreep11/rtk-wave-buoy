# Student Guide: RTK Wave Buoy Engineering

This guide provides the theoretical background and engineering details for the RTK Wave Buoy. It is intended as a learning resource for students studying embedded systems, marine technology, and precision GNSS.

**Other docs:** [hardware-spec.md](hardware-spec.md) (wiring diagram), [learning-path.md](learning-path.md), [system-architecture.md](system-architecture.md), [wiring-and-pins.md](wiring-and-pins.md), [firmware-walkthrough.md](firmware-walkthrough.md).

---

## 🔌 1. Power Architecture: Dual 3S2P Li-ion (150 Wh)

The buoy uses two **Li-ion 3S2P** packs (**75 Wh** each) wired in **parallel** on a shared bus (~**10.8–12.8 V**, nominal **~11.1 V**), for **~150 Wh** total stored energy. Full schematic and BOM: [hardware-spec.md](hardware-spec.md).

### 1.1 Parallel packs

```
BT1 (3S2P, 75 Wh) ──┐
                      ├── Pack bus ── INA228 ── OKI-78SR-3.3 ── 3.3 V rail
BT2 (3S2P, 75 Wh) ──┘              │
                                ESP32 · SIM7000 · ZED-F9P · OpenLog Artemis
```

**How it works:**
- **Parallel wiring** ties both pack positives together and both negatives together, so bus voltage stays at one pack’s voltage while **amp-hour / Wh capacity adds**.
- **Redundancy:** If one pack is unplugged or open-circuits, the other can still feed the bus (use matched packs and appropriate pack protection).
- **3S2P per pack:** Three cells in series (voltage) × two parallel strings (current capacity) per physical pack.

Official wiring diagram (KiCad **rtk-wave-buoy**):

![Wiring diagram](assets/wiring-diagram.png)

### 1.2 Voltage regulation

Pack voltage is stepped down by a switching regulator (**OKI-78SR-3.3** in the schematic) to a **3.3 V** logic rail that supplies the ESP32, modem, GNSS, and logger (see [hardware-spec.md](hardware-spec.md)).

**Why switching regulation?**
- Dropping $\sim 11\text{ V}$ to $3.3\text{ V}$ with a linear regulator would waste most energy as heat at modem peak currents.
- A buck converter keeps efficiency high — important when only **150 Wh** is available offshore.

### 1.3 Power budgeting

$$ P_{\text{pack}} \approx \frac{\sum (V_{\text{rail}} \times I)}{\eta_{\text{DC-DC}}} $$

**Example budget** (3.3 V rails, illustrative):

| Component | Voltage | Current | Power |
|-----------|---------|---------|-------|
| ZED-F9P + antenna | 3.3V | ~100 mA | ~0.33 W |
| SIM7000 (avg) | 3.3V | ~150 mA | ~0.50 W |
| ESP32 | 3.3V | ~100 mA | ~0.33 W |
| OLA | 3.3V | ~50 mA | ~0.17 W |
| **Total @ 3.3 V** | | | **~1.3 W** |

With $\eta \approx 85\%$ from an $11\text{ V}$ pack: $P_{\text{pack}} \approx 1.5\text{ W}$. **150 Wh** $\div 1.5\text{ W} \approx$ **100 hours** (~4 days) of continuous operation before depletion (average load; LTE bursts reduce margin).

---

## 🎓 2. Theory of Operation: Precision GNSS

### 2.1 How RTK Works
Standard GNSS receivers use **code-phase** tracking, which has a precision of a few meters. The GPS L1 C/A code chip rate is $1.023$ MHz, giving a chip length of $\sim 300$ m — tracking accuracy is typically $<1\%$ of that, or a few meters.

**Real-Time Kinematic (RTK)** positioning achieves centimeter-level accuracy by tracking the **carrier-phase** of the satellite signal. The L1 carrier frequency is $1575.42$ MHz, giving a wavelength of $\lambda \approx 19$ cm. By measuring the phase of this carrier wave to $\sim 1\%$ of a cycle, the receiver can determine range to $\sim 2$ mm precision — but only after solving the **integer ambiguity** (the unknown number of whole wavelengths between satellite and receiver).

**The Process:**
1. **Reference Station:** A stationary base station with a known, surveyed location monitors the same satellites as the buoy.
2. **Correction Generation:** The base station calculates the difference between its known position and the GNSS-calculated position, generating **RTCM** (Radio Technical Commission for Maritime Services) correction messages.
3. **NTRIP Caster:** These corrections are sent over the internet via an **NTRIP (Networked Transport of RTCM via Internet Protocol)** caster.
4. **Correction Injection:** The buoy receives these corrections over LTE via the ESP32 and injects them into the ZED-F9P via UART.
5. **Integer Ambiguity Resolution:** The ZED-F9P resolves the "integer ambiguity", moving from an **RTK Float** state to an **RTK Fixed** state.

| State | Accuracy | Meaning |
|-------|----------|---------|
| No RTK | $\sim$ m | Standard code-phase GNSS |
| RTK float | $\sim 10\text{–}50$ cm | Ambiguities not fully resolved |
| RTK fixed | $\sim 1\text{–}3$ cm | Integer ambiguities resolved |

### 2.2 Coordinate Reference Systems (CRS)
The buoy operates across different spatial representations:
- **WGS84:** The global standard for GPS coordinates (Latitude, Longitude, Ellipsoidal Height). This is what the ZED-F9P reports natively.
- **Mean Sea Level (MSL):** To calculate true ocean wave height, the ellipsoidal height is corrected using a geoid model (e.g., EGM96). The geoid represents the shape the ocean surface would take under gravity and rotation alone.
- **ENU (East, North, Up):** For local wave analysis, geodetic coordinates are converted to a local Cartesian system centered at the buoy's mean position. This allows plotting the buoy's motion as a simple $(X, Y, Z)$ trajectory in meters. The conversion uses the pyproj library in the visualizer scripts.

### 2.3 Sensor Fusion: GNSS + IMU
Wave motion is analyzed by combining two complementary data sources:
- **GNSS (ZED-F9P):** Provides highly accurate absolute vertical position (cm-level with RTK fix) but at a lower sampling rate ($\sim 10$ Hz). It is used to track long-term tides and low-frequency wave components.
- **IMU (ICM-20948):** Provides high-rate (up to $1$ kHz) acceleration data on three axes. It captures rapid wave dynamics and "chop" that the GNSS cannot resolve temporally. However, accelerometer data suffers from **drift** when double-integrated to position — small DC offsets accumulate into large position errors over time.
- **Fusion Approach:** The accelerometer data is high-pass filtered (capturing wave-frequency motion) while the GNSS position is low-pass filtered (capturing tides and mean position). The two are complementary and recombined in the frequency domain.

---

## 💻 3. Software Architecture

### 3.1 ESP32 State Machine
The `buoy_combo` firmware is designed as a robust state machine to handle unstable cellular connectivity in marine environments:

1. **Network Registration:** Ensures the SIM is registered to the LTE tower (checks `CGREG` registration status).
2. **GPRS Activation:** Establishes a packet data connection (PDP context) with the cellular network.
3. **NTRIP Connection:** Opens a TCP socket to the RTK caster and streams RTCM correction data to the ZED-F9P GNSS receiver via a dedicated UART.
4. **Telemetry Cycle:** Periodically (every $60$ s by default) closes the NTRIP socket briefly to send a JSON telemetry payload over the Hologram Cloud Socket, then reconnects.
5. **Health Monitoring:** `network_status_check_f()` and `monitor_connection_health()` poll registration and the data path:

| Fault | Detection | Recovery |
|-------|-----------|----------|
| CGREG glitch | Bad `CGREG` while RTCM still flowing | **Ignored** for $2$ min (`CELLULAR_LINK_ALIVE_MS`) |
| CGREG loss (confirmed) | $2$ bad polls without recent RTCM | Invalidate NTRIP/GPRS — `[DATA] invalidate` |
| Stale data path | `CGREG` OK but no RTCM/telemetry for $5$ min | GPRS refresh — `[GPRS] refresh` ($2$ min cooldown) |
| Registration timeout | `CGREG` not $1$/$5$ for $5$ min ($10$ min if searching + good `CSQ`) | **Escalated recover** (see below) |
| NTRIP failure streak | $10$ consecutive NTRIP failures while GPRS up | **Escalated recover** (see below) |

**Escalated modem recovery** (`modemRecoverEscalated_f`) — used for both registration timeout and NTRIP failure streak:

1. **First trigger** → RST pin reset + `configureNetwork(true)` — log: `[MODEM] hard recover` (**$10$ min** cooldown)
2. **Next trigger** (if still stuck) → full PWRKEY power cycle + `modem.begin()` — log: `[MODEM] power cycle` (**$15$ min** cooldown)

When `CGREG` returns to $1$ or $5$, escalation resets to RST-first. **Boot** (`setup()`) uses a separate safe path: radio on first, band config at `CFUN=1` — log `(boot)`. Recover uses optional `CFUN=0` band cycle — log `(recover)`.

Cooldowns prevent rapid modem cycling that could drain the battery. Full timers, serial tags, and a field log example: [`failure-paths.md`](failure-paths.md). Operator quick fixes: [root `README.md` — Troubleshooting](../README.md#troubleshooting).

### 3.2 The Telemetry Pipeline
The data flow is designed for maximum availability and low maintenance:

```
Buoy ──TCP Socket──> Hologram Cloud
     ──Webhook──> Cloudflare Worker
     ──Dispatch API──> GitHub Actions
     ──Commit──> docs/data.json
     ──Serve──> GitHub Pages (Live Dashboard)
```

Each hop in the pipeline serves a specific purpose:
- **Hologram Cloud Socket:** Eliminates the need for a public IP or port forwarding on the buoy. The buoy initiates an outbound TCP connection.
- **Cloudflare Worker:** Acts as an authentication gate (validates `BUOY_SECRET` header), reformats the payload, and triggers a GitHub `repository_dispatch` event. This separates public-facing webhook handling from private CI/CD.
- **GitHub Actions:** The `hologram-telemetry.yml` workflow parses the incoming payload and commits it to `docs/data.json`. This leverages GitHub's existing infrastructure for hosting and version control.
- **GitHub Pages:** Serves the dashboard (Leaflet map + time-series charts) with zero server management.

### 3.3 Serial Protocols: Why Different Buses?
The buoy uses two different serial communication protocols:

| Protocol | Used For | Why |
|----------|----------|-----|
| **I2C** | ZED-F9P ↔ OLA (Qwiic), INA228 ↔ ESP32 | Multi-drop bus: multiple devices share two wires (SDA/SCL). Perfect for periodic sensor reads. Operates at $400$ kHz. |
| **UART** | ESP32 → ZED-F9P (RTCM injection) | Point-to-point, high-throughput. RTCM corrections are streaming binary data — UART's full-duplex nature allows simultaneous receipt of corrections and polling of PVT data. Operates at $115200$ baud. |

**I2C (Inter-Integrated Circuit):**
- Uses address-based device selection (ZED-F9P at $0x42$, INA228 at $0x40$).
- Requires pull-up resistors on SDA/SCL lines (built into Qwiic connectors).
- Prone to bus lockups if a device holds SDA low — the OLA firmware handles this with a bus timeout reset.

**UART (Universal Asynchronous Receiver-Transmitter):**
- Two-wire: TX (transmit) and RX (receive).
- No clock line — both ends must agree on baud rate in advance.
- The ESP32's UART2 is dedicated to the ZED-F9P's RTCM injection port, while UART1 handles modem communication.

---

## 🌊 4. Wave Analysis from Buoy Motion

### 4.1 What We Measure
The buoy's vertical motion ($Z(t)$) is a direct measurement of the ocean surface elevation at the buoy's location. By analyzing this time series, we can extract:

- **Significant Wave Height ($H_s$):** The average height of the highest one-third of waves. Approximately $H_s \approx 4\sigma_\eta$, where $\sigma_\eta$ is the standard deviation of the vertical displacement.
- **Peak Wave Period ($T_p$):** The period corresponding to the peak of the wave energy spectrum.
- **Wave Spectrum:** The distribution of wave energy across frequencies, obtained via the Fast Fourier Transform (FFT) of the vertical displacement time series.

### 4.2 RTK + IMU Complementarity
The ZED-F9P at its NAV-PVT rate ($\sim 10$ Hz) captures the bulk of wave energy for typical ocean swell ($0.05\text{–}0.2$ Hz or $5\text{–}20$ second periods). However, wind chop and higher-frequency waves require faster sampling. The ICM-20948 IMU's accelerometer samples at $>100$ Hz, capturing these short-period components.

| Sensor | Sample Rate | Best For |
|--------|------------|----------|
| ZED-F9P GNSS | $\sim 10$ Hz | Tides, swell, mean position |
| ICM-20948 IMU | $\sim 100$ Hz | Wind waves, chop, high-frequency motion |

### 4.3 Data Pipeline
The logged `.ubx` binary files are parsed with `ubx_parsers/v3_ubx_parser.py` which extracts high-precision position (using the `NAV-HPPOSLLH` message for $0.1$ mm resolution). The resulting CSV is fed into the `Visualizer/` scripts:

```
ubx_parsers/v3_ubx_parser.py        → *_parsed.csv
Visualizer/gnss_visualizer.py       → 2D map + 3D trajectory + precision histogram
Visualizer/imu_visualizer.py        → Accel/gyro/mag temperature plots
Visualizer/altitude_visualizer.py   → Altitude time-series analysis
```

File formats: [data-formats.md](data-formats.md). Doc index: [README.md](README.md).

---

## 🔧 5. Key Components Deep-Dive

### 5.1 ZED-F9P GNSS Receiver
- **Supported constellations:** GPS, GLONASS, Galileo, BeiDou
- **Max update rate:** $20$ Hz (PVT), $10$ Hz (RTK)
- **RTK convergence time:** Typically $< 10$ s after RTCM stream starts under good sky view
- **I2C address:** $0x42$ when configured via Qwiic

### 5.2 INA228 Power Monitor
- Measures bus voltage ($0\text{–}85$ V), shunt current, and power
- Connected on the **battery pack bus** (input to the 3.3 V regulator) to report total system draw in telemetry
- **Shunt resistor:** $15$ m$\Omega$ on the Adafruit breakout, rated to $10$ A
- **I2C address:** $0x40$ (default) or $0x45$ (ALTERNATE pin high)

### 5.3 Botletics SIM7000 LTE Module
- **Cellular bands:** LTE CAT-M1 (band 12 default for AT&T/T-Mobile, band 13 for Verizon)
- **Data protocol:** AT commands over UART ($115200$ baud)
- **Cloud socket protocol:** Plain TCP to `cloudsocket.hologram.io:9999`
- **Current draw:** Peaks at $>500$ mA during LTE transmission bursts

---

## 🛠️ 6. Practical Troubleshooting for Students

| Symptom | Likely Cause | Diagnostic Step |
| :--- | :--- | :--- |
| **No RTK Fixed** | Poor sky view or missing RTCM | Check `[NTRIP] connected` and `[RTCM]` activity in serial logs. Verify NTRIP credentials and caster host. |
| **Power Rail Fluctuation** | Unmatched parallel packs or weak cell | Measure each pack at rest and under load; parallel only matched SOC packs. |
| **I2C Errors** | Loose Qwiic cable, address conflict, or bus lockup | Run an I2C scanner sketch. Expect devices at $0x40$ (INA228) and $0x42$ (ZED-F9P). Reseat Qwiic cable. |
| **Modem Not Responding** | Power surge, SIM failure, or cooldown lockout | Check `[MODEM] hard recover` then `[MODEM] power cycle` / `* skipped (cooldown)`. See [`failure-paths.md`](failure-paths.md). Verify SIM on Hologram. |
| **`[DIAG] CPIN: SIM failure`** | Bad SIM contact or dead SIM | Reseat SIM; ESP32 reset. Expect `[MODEM] LTE CAT-M, band 12 (boot)` and `CPIN: READY`. |
| **Hours OK then `CGREG=0`** | Carrier / Hologram outage possible | Check [Hologram status](https://status.hologram.io); US2 ICCID **89418…** incidents reported |
| **SD Card Not Mounting** | FAT32 format issue or card damage | Re-format as FAT32. Try a different card. Check OLA menu option to verify mount. |
| **One pack draining faster** | Parallel imbalance or different cell health | Balance/charge packs together; swap leads to test; check `bus_v` in telemetry. |
| **Serial Monitor Garbage** | Wrong baud rate | Set terminal to $115200$ baud. Check that `BOTLETICS_SSL` is forced to $0$ in the sketch. |
| **Cloudflare Worker 401** | Mismatched `BUOY_SECRET` | Verify the secret matches between Hologram Alert header and Cloudflare environment variable. |

---

## 📖 7. Glossary

| Term | Definition |
|------|------------|
| **CGREG** | LTE network registration status (AT command). $0$ = not registered, $1$ = registered (home), $5$ = registered (roaming). |
| **CSQ** | Signal quality (AT command). $0\text{–}31$ scale; $99$ = not measurable. |
| **ENU** | East, North, Up — a local Cartesian coordinate system tangent to the Earth's surface. |
| **GPRS** | General Packet Radio Service — the data bearer over which TCP/IP traffic flows. |
| **NTRIP** | Networked Transport of RTCM via Internet Protocol — the standard protocol for streaming GNSS corrections. |
| **OLA** | OpenLog Artemis — the dedicated SD logging microcontroller by SparkFun. |
| **PDP** | Packet Data Protocol — the network context that enables IP connectivity over LTE. |
| **RTCM** | Radio Technical Commission for Maritime Services — the binary message format for GNSS correction data. |
| **RTK** | Real-Time Kinematic — a carrier-phase GNSS technique achieving centimeter-level positioning. |
| **UBX** | u-blox's proprietary binary GNSS message format (.ubx file extension for logged data). |
