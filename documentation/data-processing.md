# Data processing

After field logging, process SD card and UBX files on a PC.

## UBX to CSV (`ubx_parsers/`)

Binary `.ubx` logs from the OpenLog can be converted with:

| Script | Notes |
|--------|-------|
| `ubx_parser.py` | Original parser |
| `v2_ubx_parser.py` | Common choice — edit filenames at bottom |
| `v3_ubx_parser.py` | Newer message coverage |

Example (`v2_ubx_parser.py`):

```python
if __name__ == "__main__":
    ubx_filename = "dataLog00008.ubx"
    csv_filename = "dataLog00008_parsed.csv"
```

```bash
pip install pyubx2
python ubx_parsers/v2_ubx_parser.py
```

Sample `.ubx` and parsed CSV files in `ubx_parsers/` are for reference.

## Python visualizers (`python_visualizer/`)

| Script | Purpose |
|--------|---------|
| `gnss_visualizer.py` | Plot GNSS / RTK time series from parsed CSV |
| `imu_visualizer.py` | Plot IMU logs |

```bash
cd python_visualizer
pip install -r requirements.txt
```

Adjust input paths inside each script for your log files.

## Magnetometer calibration (`accelerometer/`)

Open `accelerometer/magcal_notebook.ipynb` in VS Code or Jupyter. The notebook covers hard-iron offset and ellipsoid soft-iron correction for ICM-20948 magnetometer data.

Datasheets: `ICM-20948-Datasheet-v1.3.pdf`, `AK09916-Magnetometer-Datasheet.pdf`.

## Output column reference

**GPS CSV:** timestamp, latitude, longitude, altitude_ellipsoid, fix_type, carrier_solution (0=none, 1=float, 2=fix), h_acc, v_acc

**IMU CSV:** Timestamp, Sensor, AccX/Y/Z (milli-g), GyrX/Y/Z (deg/s), MagX/Y/Z (µT), Temp (°C)
