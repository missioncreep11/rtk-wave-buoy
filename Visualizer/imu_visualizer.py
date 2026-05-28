import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import sys
from matplotlib.widgets import Button, Slider


def add_playback_controls(fig, n_frames, frame_callback, step=1, interval_ms=50):
    """Add Play/Pause button and time slider; call frame_callback(1..n_frames)."""
    state = {'frame': 1, 'playing': False, '_updating': False}

    ax_play = fig.add_axes([0.42, 0.005, 0.08, 0.025])
    ax_slider = fig.add_axes([0.12, 0.005, 0.28, 0.025])
    btn = Button(ax_play, 'Play')
    slider = Slider(ax_slider, 'Frame', 1, n_frames, valinit=1, valstep=1)
    timer = fig.canvas.new_timer(interval=interval_ms)

    def apply_frame(f):
        f = int(np.clip(f, 1, n_frames))
        state['frame'] = f
        state['_updating'] = True
        slider.set_val(f)
        state['_updating'] = False
        frame_callback(f)
        fig.canvas.draw_idle()

    def on_timer():
        if not state['playing']:
            return
        nxt = state['frame'] + step
        if nxt > n_frames:
            state['playing'] = False
            btn.label.set_text('Play')
            timer.stop()
            return
        apply_frame(nxt)

    def toggle(_event):
        state['playing'] = not state['playing']
        btn.label.set_text('Pause' if state['playing'] else 'Play')
        if state['playing']:
            timer.start()
        else:
            timer.stop()

    def on_slider(val):
        if state['_updating']:
            return
        if state['playing']:
            state['playing'] = False
            btn.label.set_text('Play')
            timer.stop()
        apply_frame(val)

    btn.on_clicked(toggle)
    slider.on_changed(on_slider)
    timer.add_callback(on_timer)
    apply_frame(1)
    return state

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

        n_pts = len(accX)
        time_sec_arr = time_sec.values

        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.canvas.manager.set_window_title(f'IMU Data: {filename}')

        series = [
            (axs[0, 0], time_sec_arr, [accX, accY, accZ], ['r', 'g', 'b'], 'm/s^2', 'Accelerometer'),
            (axs[0, 1], time_sec_arr, [gyrX, gyrY, gyrZ], ['r', 'g', 'b'], 'deg/s', 'Gyroscope'),
            (axs[1, 0], time_sec_arr, [magX, magY, magZ], ['r', 'g', 'b'], 'uT', 'Magnetometer'),
        ]
        plot_groups = []
        vlines = []
        for ax, t, channels, colors, ylabel, plot_title in series:
            lines = [ax.plot([], [], c, label=l)[0] for c, l in zip(colors, ['X', 'Y', 'Z'])]
            vline = ax.axvline(0, color='k', linestyle=':', alpha=0.7)
            ax.grid(True)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.set_title(plot_title)
            ax.set_xlim(0, time_sec_arr[-1])
            ax.legend(loc='upper right')
            plot_groups.append((lines, [ch.values for ch in channels]))
            vlines.append(vline)

        ax_temp = axs[1, 1]
        line_temp, = ax_temp.plot([], [], 'm')
        vline_temp = ax_temp.axvline(0, color='k', linestyle=':', alpha=0.7)
        ax_temp.grid(True)
        ax_temp.set_xlabel('Time (s)')
        ax_temp.set_ylabel('C')
        ax_temp.set_title('Temperature')
        ax_temp.set_xlim(0, time_sec_arr[-1])

        title = fig.suptitle('')

        def update_frame(i):
            for (lines, channels), vline in zip(plot_groups, vlines):
                for line, ch in zip(lines, channels):
                    line.set_data(time_sec_arr[:i], ch[:i])
                vline.set_xdata([time_sec_arr[i - 1], time_sec_arr[i - 1]])
                ax = lines[0].axes
                ymax = max(np.max(ch[:i]) for ch in channels)
                ymin = min(np.min(ch[:i]) for ch in channels)
                pad = (ymax - ymin) * 0.05 or 1.0
                ax.set_ylim(ymin - pad, ymax + pad)

            line_temp.set_data(time_sec_arr[:i], temp.values[:i])
            vline_temp.set_xdata([time_sec_arr[i - 1], time_sec_arr[i - 1]])
            t_pad = (temp.values[:i].max() - temp.values[:i].min()) * 0.05 or 0.5
            ax_temp.set_ylim(temp.values[:i].min() - t_pad, temp.values[:i].max() + t_pad)

            title.set_text(
                f"{filename}  |  Time: {time_sec_arr[i - 1]:.2f} s / {time_sec_arr[-1]:.1f} s  |  "
                f"Sample {i}/{n_pts}"
            )

        plt.tight_layout(rect=[0, 0.06, 1, 0.96])
        play_step = max(1, n_pts // 2000)
        add_playback_controls(fig, n_pts, update_frame, step=play_step, interval_ms=30)
        print(f"\nPlayback: use Play/Pause and the Frame slider ({play_step} samples per step)")
        print("Done! Displaying plot...")
        plt.show()

    except Exception as e:
        print(f"Error processing file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot IMU data from CSV')
    parser.add_argument('filename', help='Path to the IMU CSV file')
    args = parser.parse_args()
    
    plot_imu_data(args.filename)
