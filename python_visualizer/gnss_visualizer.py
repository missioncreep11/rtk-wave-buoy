import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import sys
from scipy import stats

def gnss_visualizer(filename):
    try:
        # Load data
        print(f"Loading {filename}...")
        data = pd.read_csv(filename)
        
        # Extract columns
        lat = data['latitude'].values
        lon = data['longitude'].values
        alt = data['altitude_ellipsoid'].values
        fix_type = data['fix_type'].values
        
        # Check for carrier solution
        if 'carrier_solution' in data.columns:
            carr_soln = data['carrier_solution'].values
            print("\nCarrier Solution Stats:")
            print(f"  No RTK: {np.sum(carr_soln == 0)}")
            print(f"  RTK Float: {np.sum(carr_soln == 1)}")
            print(f"  RTK Fix: {np.sum(carr_soln == 2)}")
        else:
            carr_soln = np.zeros_like(fix_type)
            print("\nNo carrier solution found")

        # Interactive inputs
        print("\nWhat data do you want to analyze?")
        print("  1 - RTK Fix only (best accuracy)")
        print("  2 - All data (with outlier filtering)")
        try:
            choice = int(input("Enter choice (1 or 2): "))
        except EOFError:
            choice = 1 # Default for non-interactive
            print(f"Using default: {choice}")
        except ValueError:
            choice = 1
            print(f"Invalid input, using default: {choice}")

        if choice == 1:
            rtk_mask = (carr_soln == 2)
            print(f"\nFiltering to RTK Fix only: {np.sum(rtk_mask)} of {len(carr_soln)} points")
            
            lat = lat[rtk_mask]
            lon = lon[rtk_mask]
            alt = alt[rtk_mask]
            fix_type = fix_type[rtk_mask]
            carr_soln = carr_soln[rtk_mask]
            
            time_data = pd.to_datetime(data.loc[rtk_mask, ['year', 'month', 'day', 'hour', 'minute', 'second']])
            use_rtk_only = True
        else:
            print("\nUsing all data with outlier filtering")
            time_data = pd.to_datetime(data[['year', 'month', 'day', 'hour', 'minute', 'second']])
            use_rtk_only = False

        print("\nWhat sort of test are you doing?")
        print("1 - Stationary")
        print("2 - Buoy/Walking")
        try:
            choice_2 = int(input("Enter choice (1 or 2): "))
        except EOFError:
            choice_2 = 1
            print(f"Using default: {choice_2}")
        except ValueError:
            choice_2 = 1
            print(f"Invalid input, using default: {choice_2}")

        if choice_2 == 1:
            jump_thresh = 0.5
            max_speed = 2.0
            test_str = "Stationary"
        else:
            jump_thresh = 1.0
            max_speed = 10.0
            test_str = "Buoy/Walking"
        
        print(f"Using {test_str} settings: max_speed={max_speed} m/s, jump_thresh={jump_thresh} m")

        print(f"\nTotal points: {len(lat)}")
        print(f"Start: {time_data.iloc[0]}")
        print(f"End: {time_data.iloc[-1]}")

        # Convert to local coordinates (meters)
        lat0 = np.median(lat)
        lon0 = np.median(lon)
        alt0 = np.median(alt)

        lat0_rad = np.radians(lat0)
        m_per_deg_lat = 111132.92 - 559.82*np.cos(2*lat0_rad) + 1.175*np.cos(4*lat0_rad)
        m_per_deg_lon = 111412.84*np.cos(lat0_rad) - 93.5*np.cos(3*lat0_rad)

        x = (lon - lon0) * m_per_deg_lon  # east
        y = (lat - lat0) * m_per_deg_lat  # north
        z = alt - alt0                     # up

        print("\nBefore filtering:")
        print(f"  E-W spread: {(np.max(x)-np.min(x))*100:.1f} cm")
        print(f"  N-S spread: {(np.max(y)-np.min(y))*100:.1f} cm")
        print(f"  Vert spread: {(np.max(z)-np.min(z))*100:.1f} cm")

        # Filtering
        if use_rtk_only:
            valid = np.ones(len(x), dtype=bool)
            print("\nSkipping outlier filtering (RTK Fix data is clean)")
        else:
            valid = np.ones(len(x), dtype=bool)
            
            # 1. Velocity filter
            dt = np.diff(time_data.values.astype('datetime64[ns]')).astype(float) / 1e9
            dt[dt == 0] = 0.001
            
            vx = np.diff(x) / dt
            vy = np.diff(y) / dt
            speed = np.sqrt(vx**2 + vy**2)
            
            bad_speed = speed > max_speed
            bad_vel = np.concatenate(([False], bad_speed)) | np.concatenate((bad_speed, [False]))
            valid &= ~bad_vel
            print(f"Velocity outliers: {np.sum(bad_vel)}")

            # 2. IQR filter
            for d, name in zip([x, y, z], ['East', 'North', 'Up']):
                good_data = d[valid]
                q1, q3 = np.percentile(good_data, [25, 75])
                iqr_val = q3 - q1
                lower = q1 - 3*iqr_val
                upper = q3 + 3*iqr_val
                bad = (d < lower) | (d > upper)
                num_bad = np.sum(bad & valid)
                valid &= ~bad
                if num_bad > 0:
                    print(f"IQR outliers ({name}): {num_bad}")

            # 3. Jump detection
            dx = np.abs(np.diff(x))
            dy = np.abs(np.diff(y))
            jumps = np.sqrt(dx**2 + dy**2) > jump_thresh
            bad_jumps = np.concatenate(([False], jumps)) | np.concatenate((jumps, [False]))
            num_jumps = np.sum(bad_jumps & valid)
            valid &= ~bad_jumps
            if num_jumps > 0:
                print(f"Jump outliers: {num_jumps}")

            # 4. MAD filter
            for d, name in zip([x, y, z], ['East', 'North', 'Up']):
                good_data = d[valid]
                med = np.median(good_data)
                mad_val = np.median(np.abs(good_data - med))
                thresh = 5 * mad_val * 1.4826
                bad = np.abs(d - med) > thresh
                num_bad = np.sum(bad & valid)
                valid &= ~bad
                if num_bad > 0:
                    print(f"MAD outliers ({name}): {num_bad}")

            print(f"\nTotal removed: {np.sum(~valid)} of {len(valid)}")
            print(f"Kept: {np.sum(valid)} points ({100*np.sum(valid)/len(valid):.1f}%)")

        # Apply filter
        x_filt = x[valid]
        y_filt = y[valid]
        z_filt = z[valid]
        time_filt = time_data[valid]
        carr_filt = carr_soln[valid]
        fix_filt = fix_type[valid]

        print("\nAfter filtering:")
        print(f"  E-W spread: {(np.max(x_filt)-np.min(x_filt))*100:.2f} cm")
        print(f"  N-S spread: {(np.max(y_filt)-np.min(y_filt))*100:.2f} cm")
        print(f"  Vert spread: {(np.max(z_filt)-np.min(z_filt))*100:.2f} cm")

        # Stats
        std_x = np.std(x_filt)
        std_y = np.std(y_filt)
        std_z = np.std(z_filt)
        std_horiz = np.sqrt(std_x**2 + std_y**2)

        print("\nPrecision (1-sigma):")
        print(f"  East: {std_x*100:.2f} cm")
        print(f"  North: {std_y*100:.2f} cm")
        print(f"  Horizontal: {std_horiz*100:.2f} cm")
        print(f"  Vertical: {std_z*100:.2f} cm")

        # Colors
        colors = []
        for i in range(len(carr_filt)):
            if carr_filt[i] == 2:        # RTK Fix = bright green
                colors.append([0, 0.9, 0])
            elif carr_filt[i] == 1:    # RTK Float = darker green
                colors.append([0, 0.5, 0])
            elif fix_filt[i] >= 3:     # 3D fix = yellow
                colors.append([1, 0.8, 0])
            else:                        # bad = red
                colors.append([1, 0, 0])
        colors = np.array(colors)

        t_sec = (time_filt - time_filt.iloc[0]).dt.total_seconds().values

        # Plots
        fig = plt.figure(figsize=(16, 10))
        fig.canvas.manager.set_window_title(f'GNSS Data: {filename}')

        # 1. 2D Position
        ax1 = fig.add_subplot(2, 3, 1)
        ax1.scatter(x_filt*100, y_filt*100, 20, c=colors)
        ax1.plot(x_filt[0]*100, y_filt[0]*100, 'ko', markersize=10, markerfacecolor='g', label='Start')
        ax1.plot(x_filt[-1]*100, y_filt[-1]*100, 'ks', markersize=10, markerfacecolor='r', label='End')
        # 1-sigma circle
        th = np.linspace(0, 2*np.pi, 100)
        ax1.plot(np.mean(x_filt)*100 + std_horiz*100*np.cos(th), 
                 np.mean(y_filt)*100 + std_horiz*100*np.sin(th), 'b--', label='1-sigma')
        ax1.grid(True)
        ax1.set_xlabel('East (cm)')
        ax1.set_ylabel('North (cm)')
        ax1.set_title(f'Position - Horiz Std: {std_horiz*100:.2f} cm')
        ax1.axis('equal')
        ax1.legend()

        # 2. East vs Time
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.scatter(time_filt, x_filt*100, 12, c=colors)
        ax2.axhline(np.mean(x_filt)*100, color='b', linestyle='-')
        ax2.axhline(np.mean(x_filt)*100 + std_x*100, color='b', linestyle='--')
        ax2.axhline(np.mean(x_filt)*100 - std_x*100, color='b', linestyle='--')
        ax2.grid(True)
        ax2.set_ylabel('East (cm)')
        ax2.set_title(f'East-West vs Time - Std: {std_x*100:.2f} cm')
        plt.setp(ax2.get_xticklabels(), rotation=45)

        # 3. North vs Time
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.scatter(time_filt, y_filt*100, 12, c=colors)
        ax3.axhline(np.mean(y_filt)*100, color='b', linestyle='-')
        ax3.axhline(np.mean(y_filt)*100 + std_y*100, color='b', linestyle='--')
        ax3.axhline(np.mean(y_filt)*100 - std_y*100, color='b', linestyle='--')
        ax3.grid(True)
        ax3.set_ylabel('North (cm)')
        ax3.set_title(f'North-South vs Time - Std: {std_y*100:.2f} cm')
        plt.setp(ax3.get_xticklabels(), rotation=45)

        # 4. Altitude vs Time
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.scatter(time_filt, z_filt*100, 12, c=colors)
        ax4.axhline(np.mean(z_filt)*100, color='b', linestyle='-')
        ax4.grid(True)
        ax4.set_ylabel('Altitude (cm)')
        ax4.set_title(f'Vertical vs Time - Std: {std_z*100:.2f} cm')
        plt.setp(ax4.get_xticklabels(), rotation=45)

        # 5. 3D Trajectory
        ax5 = fig.add_subplot(2, 3, 5, projection='3d')
        ax5.scatter(x_filt*100, y_filt*100, t_sec, c=colors)
        ax5.plot([x_filt[0]*100], [y_filt[0]*100], [t_sec[0]], 'ko', markersize=8, markerfacecolor='g')
        ax5.plot([x_filt[-1]*100], [y_filt[-1]*100], [t_sec[-1]], 'ks', markersize=8, markerfacecolor='r')
        ax5.set_xlabel('East (cm)')
        ax5.set_ylabel('North (cm)')
        ax5.set_zlabel('Time (s)')
        ax5.set_title('3D Trajectory')
        ax5.view_init(elev=30, azim=45)

        # 6. Histogram
        ax6 = fig.add_subplot(2, 3, 6)
        dist = np.sqrt((x_filt - np.mean(x_filt))**2 + (y_filt - np.mean(y_filt))**2) * 100
        ax6.hist(dist, bins=30, color=[0.3, 0.7, 0.3])
        ax6.axvline(std_horiz*100, color='r', linestyle='--', label='1-sigma')
        ax6.axvline(2*std_horiz*100, color='r', linestyle=':', label='2-sigma')
        ax6.grid(True)
        ax6.set_xlabel('Distance from Mean (cm)')
        ax6.set_ylabel('Count')
        ax6.set_title('Position Distribution')
        ax6.legend()

        mode_str = 'RTK Fix Only' if use_rtk_only else 'All Data (filtered)'
        duration = (time_filt.iloc[-1] - time_filt.iloc[0]).total_seconds() / 60
        fig.suptitle(f"{filename} - {mode_str} - {test_str}\n"
                     f"Duration: {duration:.1f} min, {len(x_filt)} points, Horiz Std: {std_horiz*100:.2f} cm",
                     fontsize=14, fontweight='bold')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # Summary
        print("\n=== RTK Summary ===")
        fix_total = np.sum(carr_filt == 2)
        float_total = np.sum(carr_filt == 1)
        none_total = np.sum(carr_filt == 0)
        total = len(carr_filt)
        print(f"RTK Fix: {fix_total} ({100*fix_total/total:.1f}%)")
        print(f"RTK Float: {float_total} ({100*float_total/total:.1f}%)")
        print(f"No RTK: {none_total} ({100*none_total/total:.1f}%)")

        print("\n" + "*"*30)
        if std_horiz*100 < 5:
            print("** Excellent RTK performance! **")
        elif std_horiz*100 < 20:
            print("Good RTK Float performance")
        else:
            print("Standard GPS performance")
        print("*"*30)

        plt.show()

    except Exception as e:
        print(f"Error processing file: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot GNSS RTK data from CSV')
    parser.add_argument('filename', help='Path to the parsed GNSS CSV file')
    args = parser.parse_args()
    
    gnss_visualizer(args.filename)
