# Python Data Visualizers

Python-based data visualizers for post-processed GNSS and IMU data from the RTK Wave Buoy.

## Scripts

### `gnss_visualizer.py` — GNSS / RTK Analysis
- Interactive RTK filtering (Fix vs All Data)
- Stationary vs Buoy/Walking test modes
- Geographic (lat/lon/alt) to local Cartesian (ENU) conversion
- Outlier filtering: velocity, IQR, jump detection, MAD
- 2D position map with 1-sigma precision circle, east/north/vertical time-series, 3D trajectory, precision histogram
- Color-coded by RTK status (Fix, Float, 3D, Bad)

```bash
python gnss_visualizer.py path/to/parsed_positions.csv
```

### `imu_visualizer.py` — IMU Data
- Converts accelerometer from milli-g to m/s²
- Cleans timestamps, calculates relative time
- 2x2 subplot: accelerometer, gyroscope, magnetometer, temperature

```bash
python imu_visualizer.py path/to/imuLog.csv
```

### `altitude_visualizer.py` — Altitude Time-Series
- Altitude-over-time analysis from parsed GNSS data

```bash
python altitude_visualizer.py path/to/parsed_positions.csv
```

## Installation

```bash
pip install -r requirements.txt
```

## Dependencies

`pandas`, `matplotlib`, `numpy`, `scipy`, `contextily`, `pyproj`, `plotly`
