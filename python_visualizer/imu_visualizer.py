import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import sys

def plot_imu_data(filename):
    try:
        # Load data
        print(f"Loading {filename}...")
        data = pd.read_csv(filename)
        
        # Check for required columns
        required_cols = ['AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ', 'MagX', 'MagY', 'MagZ', 'Temp', 'Timestamp']
        for col in required_cols:
            if col not in data.columns:
                print(f"Error: Missing column {col}")
                return

        # extract and convert accelerometer (milli-g to m/s^2)
        accX = data['AccX'] / 1000 * 9.81
        accY = data['AccY'] / 1000 * 9.81
        accZ = data['AccZ'] / 1000 * 9.81

        # gyroscope (already in deg/s)
        gyrX = data['GyrX']
        gyrY = data['GyrY']
        gyrZ = data['GyrZ']

        # magnetometer (already in uT)
        magX = data['MagX']
        magY = data['MagY']
        magZ = data['MagZ']

        # temperature
        temp = data['Temp']

        # fix timestamps
        # MATLAB: clean_timestamps = cellfun(@(x) strrep(x, ' IMU', ''), timestamps, 'UniformOutput', false);
        # time_data = datetime(clean_timestamps, 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SS');
        def parse_timestamp(ts):
            ts_clean = ts.replace(' IMU', '')
            try:
                return datetime.strptime(ts_clean, '%Y/%m/%d %H:%M:%S.%f')
            except ValueError:
                # Handle cases where milliseconds might be missing or different format
                return datetime.strptime(ts_clean, '%Y/%m/%d %H:%M:%S')

        time_data = data['Timestamp'].apply(parse_timestamp)
        time_sec = (time_data - time_data.iloc[0]).dt.total_seconds()

        print(f"Samples: {len(accX)}")
        print(f"Duration: {time_sec.max():.1f} sec")

        # Plot everything
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.canvas.manager.set_window_title(f'IMU Data: {filename}')

        # Accelerometer
        axs[0, 0].plot(time_sec, accX, 'r', label='X')
        axs[0, 0].plot(time_sec, accY, 'g', label='Y')
        axs[0, 0].plot(time_sec, accZ, 'b', label='Z')
        axs[0, 0].grid(True)
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('m/s^2')
        axs[0, 0].set_title('Accelerometer')
        axs[0, 0].legend()

        # Gyroscope
        axs[0, 1].plot(time_sec, gyrX, 'r', label='X')
        axs[0, 1].plot(time_sec, gyrY, 'g', label='Y')
        axs[0, 1].plot(time_sec, gyrZ, 'b', label='Z')
        axs[0, 1].grid(True)
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('deg/s')
        axs[0, 1].set_title('Gyroscope')
        axs[0, 1].legend()

        # Magnetometer
        axs[1, 0].plot(time_sec, magX, 'r', label='X')
        axs[1, 0].plot(time_sec, magY, 'g', label='Y')
        axs[1, 0].plot(time_sec, magZ, 'b', label='Z')
        axs[1, 0].grid(True)
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('uT')
        axs[1, 0].set_title('Magnetometer')
        axs[1, 0].legend()

        # Temperature
        axs[1, 1].plot(time_sec, temp, 'm')
        axs[1, 1].grid(True)
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('C')
        axs[1, 1].set_title('Temperature')

        plt.tight_layout()
        print("Done! Displaying plot...")
        plt.show()

    except Exception as e:
        print(f"Error processing file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot IMU data from CSV')
    parser.add_argument('filename', help='Path to the IMU CSV file')
    args = parser.parse_args()
    
    plot_imu_data(args.filename)
