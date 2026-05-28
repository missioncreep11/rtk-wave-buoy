# Python Data Visualizers

This folder contains Python-based data visualizers that replicate the functionality of the original MATLAB scripts.

## Scripts

### 1. `imu_visualizer.py`
Replicates `simple_IMU_plotting.m`.
- **Purpose**: Visualizes raw IMU data (Accelerometer, Gyroscope, Magnetometer, and Temperature).
- **Features**:
  - Converts accelerometer data from milli-g to m/s².
  - Cleans timestamps and calculates relative time in seconds.
  - Generates a 2x2 subplot of all sensor data.
- **Usage**:
  ```bash
  python imu_visualizer.py path/to/imuLog.csv
  ```

### 2. `gnss_visualizer.py`
Replicates `v3_fall_parsed_positions_graph.m`.
- **Purpose**: High-precision GNSS/RTK data visualization and analysis.
- **Features**:
  - Interactive selection for RTK filtering (RTK Fix vs. All Data).
  - Test mode selection (Stationary vs. Buoy/Walking) to adjust outlier thresholds.
  - Conversion from Geographic (Lat/Lon/Alt) to Local Cartesian (ENU meters).
  - Comprehensive outlier filtering: Velocity, IQR, Jump Detection, and MAD.
  - 2x3 visualization grid:
    - 2D Position map with 1-sigma precision circle.
    - East/North/Vertical time-series with standard deviation lines.
    - 3D Trajectory (East, North, Time).
    - Precision distribution histogram.
  - Color-coded points based on RTK status (Fix, Float, 3D, Bad).
- **Usage**:
  ```bash
  python gnss_visualizer.py path/to/parsed_positions.csv
  ```

## Installation

These scripts require Python 3 and several libraries. You can install the dependencies using:

```bash
pip install -r requirements.txt
```

## Dependencies
- `pandas`: Data loading and manipulation.
- `matplotlib`: Data visualization.
- `numpy`: Numerical calculations.
- `scipy`: Statistical calculations for outlier filtering.
