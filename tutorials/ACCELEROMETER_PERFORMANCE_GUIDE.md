# IMU Performance Assessment — OpenLog Artemis ICM-20948

Characterize all three sensors in the ICM-20948 9-DOF IMU, calibrate each one, and evaluate whether the system can resolve ocean wave motion. You will collect several datasets, build a Jupyter notebook to analyze them, and compare measurements to datasheet specifications and known physical references.

> **Note:** The analysis notebook is not provided — you will build it yourself. The PDFs in this folder are your primary reference for specs, register descriptions, and operating principles.
>
> - `ICM-20948-Datasheet-v1.3.pdf` — gyroscope, accelerometer, magnetometer specs; register map
> - `AK09916-Magnetometer-Datasheet.pdf` — magnetometer operating modes, self-test procedure, calibration register

---

## 1. Hardware Background

### 1.1 Package Architecture

The ICM-20948 is a **multi-chip module (MCM)** — two separate silicon dies bonded inside a single 3×3×1 mm QFN package.

| Die | Contents |
|-----|----------|
| Die 1 (InvenSense) | 3-axis MEMS gyroscope, 3-axis MEMS accelerometer, Digital Motion Processor (DMP) |
| Die 2 (Asahi Kasei) | AK09916 3-axis Hall-effect magnetometer |

The main die communicates with the magnetometer over an **internal I²C master bus** — the magnetometer is not directly accessible from the host microcontroller. The OLA firmware handles this transparently and delivers all nine axes together in the CSV.

The full device communicates with the Apollo3 microcontroller on the OLA via I²C (address `0x68` or `0x69` depending on the AD0 pin) at up to 400 kHz, or SPI at up to 7 MHz.

---

### 1.2 Accelerometer — MEMS Capacitive

The accelerometer uses a **MEMS proof mass** suspended by microscale flexures. When the chip accelerates, the mass deflects relative to the frame. That deflection is measured as a change in capacitance between the mass and fixed electrodes.

> **Important:** An accelerometer measures **specific force** — the net non-gravitational force per unit mass. At rest on a table, it reads **+1g upward** (the table's normal force), not zero. In free fall, it reads zero. This distinction matters when interpreting wave buoy data.

**Accelerometer specifications (ICM-20948 Datasheet, Table 2):**

| Parameter | Value |
|-----------|-------|
| Full-scale range (default, ACCEL\_FS=0) | ±2 g |
| Programmable ranges | ±2 g, ±4 g, ±8 g, ±16 g |
| ADC resolution | 16-bit |
| Sensitivity (±2 g) | 16,384 LSB/g |
| Initial zero-g offset (board-level) | ±50 mg |
| Noise spectral density | 230 µg/√Hz |
| Nonlinearity | ±0.5% |
| Cross-axis sensitivity | ±2% |
| Output units in OLA CSV | **milli-g** |

**Converting noise density to expected RMS noise:**

At a given low-pass filter bandwidth *B* (Hz), expected RMS noise ≈ 230 µg/√Hz × √B. At the OLA's default 10 Hz sample rate (effective bandwidth ≈ 5 Hz): expected noise ≈ 230 × √5 ≈ 515 µg ≈ **0.5 milli-g RMS**. You will verify this experimentally.

---

### 1.3 Gyroscope — MEMS Coriolis

The gyroscope uses the **Coriolis effect**. A proof mass is driven to vibrate at a fixed frequency along one axis. When the device rotates, the Coriolis force deflects the mass in the perpendicular direction. That secondary deflection is proportional to the angular rate.

> Gyroscopes measure **angular rate** (degrees per second), not angle. To get heading or orientation you must integrate over time — which accumulates drift. The zero-rate output (ZRO) bias is the main source of that drift.

**Gyroscope specifications (ICM-20948 Datasheet, Table 1):**

| Parameter | Value |
|-----------|-------|
| Full-scale range (default, GYRO\_FS\_SEL=0) | ±250 dps |
| Programmable ranges | ±250, ±500, ±1000, ±2000 dps |
| ADC resolution | 16-bit |
| Sensitivity (±250 dps) | 131 LSB/dps |
| Initial ZRO (zero-rate output) bias | ±5 dps |
| ZRO temperature dependence | ±0.05 dps/°C |
| Noise spectral density | 0.015 dps/√Hz |
| Cross-axis sensitivity | ±2% |
| Output units in OLA CSV | **deg/s** |

**Accumulated heading error from bias:**

If the gyro has a 1 dps bias and you integrate for 60 seconds, you accumulate 60° of heading error. For a buoy that rocks on waves, gyro bias directly limits how accurately you can estimate tilt angle over time.

---

### 1.4 Magnetometer — Hall-Effect (AK09916)

The AK09916 uses the **Hall effect**: a current-carrying conductor in a magnetic field develops a transverse voltage proportional to the component of the field perpendicular to the current. Three orthogonal Hall sensors give the full 3-axis field vector.

The magnetometer measures the **total ambient magnetic field** — Earth's geomagnetic field plus any local distortions from nearby ferromagnetic objects or currents. Separating these contributions is the core challenge of magnetometer calibration.

Earth's field at San Diego is approximately 47 µT total, with a horizontal component of ~22 µT pointing roughly north and a downward dip of ~60°.

**Magnetometer specifications (ICM-20948 Datasheet, Table 3 / AK09916 Datasheet, §8.3):**

| Parameter | Value |
|-----------|-------|
| Full-scale range | ±4900 µT |
| ADC resolution | 16-bit |
| Sensitivity scale factor | 0.15 µT/LSB |
| Initial zero-field offset | ±2000 LSB (±300 µT) |
| Maximum output data rate | 100 Hz |
| Output units in OLA CSV | **µT** |

The large initial offset (±300 µT) is why raw magnetometer data rarely points at magnetic north without calibration.

---

### 1.5 Digital Motion Processor (DMP)

The ICM-20948 includes an onboard **Digital Motion Processor** — a small microcontroller that can run sensor fusion algorithms internally, fusing accelerometer, gyroscope, and magnetometer data into orientation quaternions. The OLA firmware does **not** use the DMP in its current configuration; it logs raw sensor values. You will implement your own calibration and fusion in your notebook.

---

## 2. Setup

The OLA should be running the modified GNSS Logger firmware with IMU logging enabled. See `IMU_STUDENT_GUIDE.md` for firmware setup. Default logging settings:

- Sample rate: 100 ms (10 Hz)
- All sensors: accelerometer, gyroscope, magnetometer, temperature

Use the `f` command in the serial menu to start a new log file between experiments without rebooting. Each module below requires a separate dataset.

---

## 3. Module A — Accelerometer

### A1. Static Noise Test

Place the OLA flat and undisturbed on a stable surface. Log for at least 60 seconds.

This dataset gives you the baseline noise floor. Consider:

- What value do you expect on each axis? What does the datasheet predict for zero-g output?
- How do you quantify noise? What statistic is appropriate, and why?
- Using the noise spectral density from Table 2 and your measured sample rate, what RMS noise does the datasheet predict? How does your measurement compare?
- At what axis orientation is the OLA when flat? How does this relate to the sign and magnitude of the readings?

### A2. Gravity Vector Check (Six-Face Calibration)

Rotate the OLA through all six face-down orientations (±X, ±Y, ±Z pointing down), holding each for roughly 10 seconds. Log the full sequence as one file.

This is a **6-position calibration** — the standard method for determining accelerometer bias and scale factor on each axis without specialized equipment.

For each axis, the calibration model is:

$$a_\text{corrected} = \frac{a_\text{raw} - b}{s}$$

where *b* is the bias (zero-g offset) and *s* is the scale factor.

Consider:

- With two opposing orientations (e.g., +Z up and −Z up), how can you solve for both bias *b* and scale factor *s* on the Z-axis?
- What should the total acceleration magnitude ‖**a**‖ equal in every orientation? Does your data satisfy this?
- What does cross-axis coupling look like in the data? Can you estimate it from your six-face results?

---

## 4. Module B — Gyroscope

### B1. Zero-Rate Bias Test

Place the OLA completely still on a stable surface. Log for at least 120 seconds.

The mean output on each axis when stationary is the **zero-rate output (ZRO) bias**. The datasheet specifies initial ZRO of ±5 dps and temperature drift of ±0.05 dps/°C.

Consider:

- What is the measured ZRO bias on each axis? Is it within the ±5 dps spec?
- If you integrate the raw (uncorrected) gyro signal over 120 seconds, how much heading error accumulates? What about after bias subtraction?
- The temperature sensor is also logged. Does the gyro bias change over the 120-second window as the board warms up? Can you see the correlation?
- How does the gyro noise spectral density (0.015 dps/√Hz) compare to the bias? Which limits your heading accuracy more over short timescales vs. long timescales?

### B2. Known Rotation Test

Place the OLA flat. Rotate it exactly 90° about the Z-axis, hold for a few seconds, then rotate back. Log the full sequence.

Consider:

- After integrating GyrZ with respect to time, does your computed heading change equal 90°?
- Apply your ZRO bias correction from B1. Does the integrated angle improve?
- What limits the accuracy of gyro-only orientation estimation for a wave buoy that may be rocking continuously for hours?

---

## 5. Module C — Magnetometer

### C1. Raw Field Measurement

With the OLA held flat and oriented consistently (USB connector pointing north, if possible), log 30 seconds of static data.

Consider:

- What is the magnitude of the measured magnetic field vector ‖**B**‖ = √(MagX² + MagY² + MagZ²)?
- San Diego's total field magnitude is approximately 47 µT. How close is your reading to this value?
- What heading does the raw magnetometer give? (heading = atan2(MagY, MagX) when the board is level). How does it compare to known north?

The offset between your raw heading and geographic north has two components: **magnetic declination** (the angle between magnetic north and true north — about +11° east in San Diego) and **sensor bias**. You will separate these in the calibration step.

### C2. Hard-Iron and Soft-Iron Calibration

Rotate the OLA slowly through all orientations — pitch, roll, yaw, and combinations — for at least 60 seconds. Try to sample the full sphere of possible orientations. Log the entire rotation as one file.

When you plot MagX vs. MagY (and vs. MagZ), the data should trace a sphere centered at the origin. In practice you will see an **ellipsoid offset from the origin**:

- **Hard-iron distortion**: permanent magnetic fields from nearby ferromagnetic material (e.g., PCB traces, LiPo) shift the center of the sphere away from the origin. Corrected by subtracting the offset vector.
- **Soft-iron distortion**: the shape of the sphere becomes distorted into an ellipsoid due to soft-magnetic materials that locally concentrate or distort the field. Corrected by a 3×3 scaling matrix.

A simple first-order calibration (hard-iron only):

$$\text{offset}_i = \frac{\max(B_i) + \min(B_i)}{2}$$

$$B_{i,\text{cal}} = B_i - \text{offset}_i$$

Consider:

- Plot MagX vs. MagY before and after applying your hard-iron correction. Does the ellipse shift to be centered on the origin?
- After correction, what is the radius of the distribution? Does it match the expected horizontal field component?
- What heading does the calibrated magnetometer give when the board is pointing north? How close is it to the known magnetic declination-corrected value?
- Where on the OLA board do you think the hard-iron sources are located?

---

## 6. Module D — Pendulum Test

Build a simple pendulum using the OLA as the bob. Tape the board securely to the end of a rigid rod or hang it from a string, with the board face parallel to the plane of swing. Measure the pendulum length **L** from the pivot point to the center of the board.

The period of a simple pendulum (small-angle approximation) is:

$$T = 2\pi \sqrt{\frac{L}{g}}$$

Start the pendulum from a small angle (< 15°) and log at least 10 full oscillations.

Consider:

- Which axis or axes show the oscillating signal? Why those and not the others?
- How do you extract the measured period **T** from the data? There is more than one valid approach — consider both time-domain and frequency-domain methods.
- How does your measured **T** compare to the value predicted from your measured **L**?
- The small-angle approximation breaks down for large release angles. Does your data show any sign of this (e.g., period lengthening at larger amplitudes)?
- What is the frequency of the pendulum? How does it compare to typical ocean wave frequencies? What does this imply about using the accelerometer to sense waves?

---

## 7. Building Your Notebook

Your notebook should load each module's CSV, produce relevant plots, and compute quantitative results. At a minimum:

**Module A (Accelerometer):**
- Time-series plot of all three axes
- Noise histogram with expected vs. measured RMS
- Six-face bar chart showing per-axis bias and scale factor
- Gravity magnitude plot before and after calibration correction

**Module B (Gyroscope):**
- Time-series of all three axes during static test
- ZRO bias estimate per axis with units of dps and equivalent deg/min drift
- Integrated heading from rotation test before and after bias correction

**Module C (Magnetometer):**
- 2D scatter plots: MagX vs. MagY and MagX vs. MagZ, before and after hard-iron correction
- Field magnitude histogram (target: ~47 µT total at San Diego)
- Heading estimate before and after calibration

**Module D (Pendulum):**
- Time-series of oscillating axis
- Period extraction and comparison to T = 2π√(L/g)
- PSD with labeled peak frequency

The CSV format is documented in `IMU_STUDENT_GUIDE.md`. The `imu_analysis.ipynb` at the repo root shows the data loading pattern.

> **Reminder:** Notebooks in this folder are excluded from version control (`.gitignore`). Commit only your written report.

---

## 8. Troubleshooting

| Issue | Fix |
|-------|-----|
| All accelerometer axes near zero | IMU not initialized. Check serial output on boot for "IMU online". |
| AccZ ≈ +1000 when flat | Board is upside down — component side facing down. |
| Noise much larger than datasheet predicts | Table is vibrating. Try a more isolated surface; check for HVAC vibration. |
| Gyro not returning to zero after rotation | Expected — ZRO bias. Subtract your measured bias from B1 and re-check. |
| Magnetometer magnitude far from 47 µT | You may be near a large ferromagnetic object (steel desk, power supply). Move away and re-test. |
| Magnetometer not in CSV | Check OLA menu 3 → option 4 (Log Magnetometer) is enabled. |
| Pendulum signal hard to see | Make sure the board is rigidly attached — loose tape adds its own vibration modes. |
| CSV empty or very short | Quit logging through the menu (q → y) before removing the SD card. |
| Timestamps stuck at 2000/01/01 | RTC coin cell is dead — expected on this unit. Relative timing is still correct for all analysis. |
