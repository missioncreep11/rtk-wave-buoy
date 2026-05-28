# Student Guide: RTK Wave Buoy Engineering

This guide provides the theoretical background and engineering details for the RTK Wave Buoy. It is intended as a learning resource for students studying embedded systems, marine technology, and precision GNSS.

---

## 🔌 1. Power Architecture: 12V Dual-Battery Diode ORing

The buoy uses a redundant power system designed for longevity and reliability in harsh marine environments.

### 1.1 Diode ORing for Redundancy
Instead of a single battery, the buoy utilizes two 12V LiFePO4 batteries combined via **ORing diodes**.

**Schematic:**
```
12V Batt A ──[Schottky Diode]──┐
                                +── 12V Power Bus ── INA228 ── Buck (12V→5V)
12V Batt B ──[Schottky Diode]──┘                                       │
                                                                        ├── SIM7000 (5V)
                                                                        └── OLA (5V)
                                                                                │
                                                                          ESP32 LDO (5V→3.3V)
                                                                                │
                                                                           ┌────┴────┐
                                                                        ZED-F9P   I2C Sensors
```

**How it works:**
Each battery is connected to a common power bus through a blocking diode. This configuration ensures that:
- **Redundancy:** If one battery fails or is disconnected, the other continues to power the system without interruption.
- **Reverse Protection:** Diodes prevent current from flowing from one battery into another, protecting against back-charging and potential damage if one battery's internal resistance drops significantly (short circuit).

**Diode Types:**
- **Schottky Diodes:** Common for this application due to their low forward voltage drop ($\sim 0.3\text{V}$ to $0.4\text{V}$).
- **Ideal Diodes:** For higher efficiency, MOSFET-based "ideal diode" controllers can be used, reducing the voltage drop to millivolts and eliminating heat dissipation.

### 1.2 Voltage Regulation Chain
Since the buoy's electronics operate at low voltages, the $12\text{V}$ bus is stepped down through a regulation chain:

1. **$12\text{V}$ Bus $\rightarrow$ Buck Regulator $\rightarrow$ $5\text{V}$ Rail:** A high-efficiency DC-DC buck converter reduces $12\text{V}$ to $5\text{V}$. This rail powers the SIM7000 LTE modem and the OpenLog Artemis.
2. **$5\text{V}$ Rail $\rightarrow$ LDO $\rightarrow$ $3.3\text{V}$ Rail:** The ESP32's onboard Low-Dropout (LDO) regulator further reduces the voltage to $3.3\text{V}$ for the microprocessor, I2C sensors, and the ZED-F9P GNSS receiver.

**Why a buck converter instead of a linear regulator?**
- A linear regulator would dissipate $(12\text{V} - 5\text{V}) \times I$ as heat, which is extremely inefficient at the buoy's typical currents ($>150$ mA).
- A buck (switching) regulator operates at $>85\%$ efficiency, meaning most of the power reaches the load rather than being wasted as heat.
- This is critical in a battery-powered marine deployment where every watt-hour matters.

### 1.3 Power Budgeting
Calculating runtime involves summing the average current draw of all components and accounting for regulator efficiency ($\eta$):

$$ \text{Total Power (W)} = \frac{\sum (\text{Voltage} \times \text{Current})}{\eta} $$

**Example budget:**
| Component | Voltage | Current | Power |
|-----------|---------|---------|-------|
| ZED-F9P | 3.3V | 85 mA | 0.28 W |
| GNSS antenna | 3.3V | 15 mA | 0.05 W |
| SIM7000 (avg) | 5V | 150 mA | 0.75 W |
| ESP32 | 3.3V | 100 mA | 0.33 W |
| OLA | 5V | 50 mA | 0.25 W |
| **Total** | | | **1.66 W** |

With a 90% efficient buck regulator, the actual power drawn from the $12\text{V}$ bus is $1.66\text{W} / 0.90 \approx 1.84\text{W}$. A typical 12V 5Ah LiFePO4 battery provides $60\text{Wh}$, so two batteries provide $120\text{Wh}$ — over 65 hours of continuous runtime.

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
5. **Health Monitoring:** A dedicated function (`monitor_connection_health`) continuously checks for faults:

| Fault | Detection | Recovery |
|-------|-----------|----------|
| CGREG loss | Poll $30$ s; $2$ bad polls in a row | Data path invalidation + GPRS refresh |
| Stale data path | No RTCM or telemetry for $5$ min | GPRS refresh (PDP re-establish) |
| NTRIP failure streak | $10$ consecutive connection failures | RST hard recover (Level 1) |
| Persistent unregistered | No CGREG $1$/$5$ for $5\text{–}10$ min | PWRKEY power cycle (Level 2) |

The recovery system is designed with **cooldowns** ($10$ min for RST, $15$ min for PWRKEY) to prevent rapid cycling that could damage the modem or drain the battery. See [`failure-paths.md`](failure-paths.md) for the full reference.

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
The logged `.ubx` binary files are parsed with `ubx_parsers/v3_ubx_parser.py` which extracts high-precision position (using the `NAV-HPPOSLLH` message for $0.1$ mm resolution). The resulting CSV is fed into the `visualizer/` scripts:

```
ubx_parsers/v3_ubx_parser.py       → parsed_positions.csv
visualizer/gnss_visualizer.py       → 2D map + 3D trajectory + precision histogram
visualizer/imu_visualizer.py        → Accel/gyro/mag temperature plots
visualizer/altitude_visualizer.py   → Altitude time-series analysis
```

---

## 🔧 5. Key Components Deep-Dive

### 5.1 ZED-F9P GNSS Receiver
- **Supported constellations:** GPS, GLONASS, Galileo, BeiDou
- **Max update rate:** $20$ Hz (PVT), $10$ Hz (RTK)
- **RTK convergence time:** Typically $< 10$ s after RTCM stream starts under good sky view
- **I2C address:** $0x42$ when configured via Qwiic

### 5.2 INA228 Power Monitor
- Measures bus voltage ($0\text{–}85$ V), shunt current, and power
- Connected to the $12\text{V}$ bus before the buck regulator to measure total system draw
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
| **Power Rail Fluctuation** | Battery mismatch or diode drop | Measure voltage before and after the blocking diodes using a multimeter. A healthy diode should drop $0.3\text{–}0.4\text{V}$. |
| **I2C Errors** | Loose Qwiic cable, address conflict, or bus lockup | Run an I2C scanner sketch. Expect devices at $0x40$ (INA228) and $0x42$ (ZED-F9P). Reseat Qwiic cable. |
| **Modem Not Responding** | Power surge, SIM failure, or cooldown lockout | Check `[MODEM] hard recover` / `[MODEM] * skipped (cooldown)` in serial logs. Verify SIM is active in Hologram dashboard. |
| **SD Card Not Mounting** | FAT32 format issue or card damage | Re-format as FAT32. Try a different card. Check OLA menu option to verify mount. |
| **One Battery Draining Faster** | Asymmetric diode drop or regulator imbalance | Swap battery positions. Measure voltage under load at each battery terminal. |
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
