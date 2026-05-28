import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
from matplotlib.widgets import Button, Slider
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


def add_playback_controls(fig, n_frames, frame_callback, play_stride=1, interval_ms=60):
    """
    Play/Pause + frame slider (valstep=1).
    Keeps FuncAnimation on fig._gnss_anim so it is not garbage-collected before plt.show().
    """
    state = {
        'frame': 1,
        'playing': False,
        '_updating': False,
        '_pending': None,
    }

    ax_play = fig.add_axes([0.42, 0.04, 0.09, 0.04])
    ax_slider = fig.add_axes([0.08, 0.04, 0.30, 0.04])
    for ax in (ax_play, ax_slider):
        ax.set_zorder(100)
        ax.patch.set_alpha(0.9)

    btn = Button(ax_play, 'Play')
    slider = Slider(ax_slider, 'Frame', 1, n_frames, valinit=1, valstep=1)

    frame_list = list(range(1, n_frames + 1, max(1, play_stride)))
    if frame_list[-1] != n_frames:
        frame_list.append(n_frames)
    n_anim_frames = len(frame_list)

    def render_frame(f, *, from_anim=False, force_full=False):
        f = int(np.clip(f, 1, n_frames))
        fast = from_anim and not force_full
        if not fast and not force_full and f == state.get('_last_rendered'):
            return
        state['_last_rendered'] = f
        state['frame'] = f
        state['_updating'] = True
        slider.set_val(f)
        state['_updating'] = False
        frame_callback(f, fast=fast)
        fig.canvas.draw_idle()

    def flush_pending():
        state['_throttle_timer'].stop()
        state['_throttle_armed'] = False
        pending = state.pop('_pending', None)
        if pending is not None:
            render_frame(pending)

    state['_throttle_timer'] = fig.canvas.new_timer(interval=30)
    state['_throttle_timer'].add_callback(flush_pending)

    def request_frame(f):
        state['_pending'] = int(np.clip(f, 1, n_frames))
        if not state.get('_throttle_armed'):
            state['_throttle_armed'] = True
            state['_throttle_timer'].start()

    def anim_step(frame_idx):
        render_frame(frame_list[frame_idx % n_anim_frames], from_anim=True)

    anim = FuncAnimation(
        fig,
        anim_step,
        frames=n_anim_frames,
        interval=interval_ms,
        repeat=True,
        blit=False,
        cache_frame_data=False,
    )
    # Must survive until plt.show() ends — otherwise Play does nothing (GC warning).
    fig._gnss_anim = anim

    def sync_anim_index(f):
        idx = int(np.searchsorted(frame_list, f))
        anim._frame_counter = min(idx, n_anim_frames - 1)

    def stop_play():
        anim.pause()
        was_playing = state['playing']
        state['playing'] = False
        btn.label.set_text('Play')
        if was_playing:
            render_frame(state['frame'], force_full=True)

    def start_play():
        sync_anim_index(state['frame'])
        anim.resume()
        state['playing'] = True
        btn.label.set_text('Pause')

    def toggle(_event):
        if state['playing']:
            stop_play()
        else:
            start_play()

    def on_slider(val):
        if state['_updating']:
            return
        if state['playing']:
            stop_play()
        f = int(round(val))
        sync_anim_index(f)
        request_frame(f)

    btn.on_clicked(toggle)
    slider.on_changed(on_slider)
    anim.pause()
    render_frame(1)
    return anim

def gps_timestamp_values(df):
    """u-blox PVT 'timestamp' column: GPS time-of-week in seconds."""
    if 'timestamp' not in df.columns:
        return None
    return df['timestamp'].values.astype(np.float64)


def is_gps_time_of_week(ts):
    """
    Detect GPS TOW (0–604800 s) vs Unix epoch.
    Parsed logs use TOW; calendar 'second' is often 0 for all 1 Hz samples in a minute.
    """
    if ts is None or len(ts) < 2:
        return False
    span = float(np.nanmax(ts) - np.nanmin(ts))
    if span < 0.5:
        return False
    med_dt = float(np.nanmedian(np.diff(ts)))
    return (
        np.nanmax(ts) < 7e5
        and (span > 2.0 or np.nanmax(ts) > 1e5)
        and 0.02 <= med_dt <= 120.0
    )


def elapsed_seconds(ts):
    """Elapsed seconds from first sample; handles GPS week rollover."""
    ts = np.asarray(ts, dtype=np.float64)
    t = ts - ts[0]
    jumps = np.diff(t)
    if np.any(jumps < -1000):
        t = np.unwrap(ts - ts[0], period=604800)
    return t


def elapsed_seconds_from_df(df):
    """Monotonic elapsed seconds for time-series plots and playback."""
    ts = gps_timestamp_values(df)
    if is_gps_time_of_week(ts):
        return elapsed_seconds(ts)
    cal = pd.to_datetime(df[['year', 'month', 'day', 'hour', 'minute', 'second']])
    return (cal - cal.iloc[0]).dt.total_seconds().values


def build_time_series(df):
    """Wall-clock datetimes for console start/end (GPS TOW aligned to calendar anchor)."""
    cal = pd.to_datetime(df[['year', 'month', 'day', 'hour', 'minute', 'second']])
    ts = gps_timestamp_values(df)
    if is_gps_time_of_week(ts):
        return pd.Series(
            cal.iloc[0] + pd.to_timedelta(elapsed_seconds(ts), unit='s'),
            index=df.index,
        )
    return cal


def fix_quality_colors(carr_soln, fix_type):
    """RGB rows for scatter plots by RTK / fix quality."""
    colors = np.zeros((len(carr_soln), 3))
    colors[carr_soln == 2] = (0, 0.9, 0)
    colors[carr_soln == 1] = (0, 0.5, 0)
    mask_3d = (carr_soln == 0) & (fix_type >= 3)
    colors[mask_3d] = (1, 0.8, 0)
    colors[(carr_soln == 0) & (fix_type < 3)] = (1, 0, 0)
    return colors


def configure_trajectory_3d(ax, x_cm, y_cm, z_vals, std_horiz_cm=None, pad_frac=0.12):
    """
    Set 3D limits and box aspect from data spans so East, North, and the third axis
    are visually comparable (matplotlib 3D autoscale otherwise squashes position).
    """
    min_horiz = 8.0
    if std_horiz_cm is not None and std_horiz_cm > 0:
        min_horiz = max(min_horiz, std_horiz_cm * 4)

    def padded_limits(vals, min_span):
        lo, hi = float(np.min(vals)), float(np.max(vals))
        span = hi - lo
        if span < min_span:
            mid = 0.5 * (lo + hi)
            lo, hi = mid - min_span / 2, mid + min_span / 2
            span = min_span
        pad = span * pad_frac
        return lo - pad, hi + pad, span + 2 * pad

    xlo, xhi, xspan = padded_limits(np.asarray(x_cm), min_horiz)
    ylo, yhi, yspan = padded_limits(np.asarray(y_cm), min_horiz)
    zlo, zhi, zspan = padded_limits(np.asarray(z_vals), float(np.ptp(z_vals)) or 1.0)

    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_zlim(zlo, zhi)
    ax.set_box_aspect((xspan, yspan, zspan))
    ax.view_init(elev=28, azim=-55)
    ax.grid(True, alpha=0.35)


def wgs84_to_web_mercator(lon, lat):
    """Lon/lat (degrees) -> Web Mercator meters for map tiles."""
    from pyproj import Transformer
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    return transformer.transform(np.asarray(lon), np.asarray(lat))


def raise_map_artists(ax_map, *artists):
    """Keep trajectory artists above satellite tiles (contextily draws an image layer)."""
    for img in ax_map.images:
        img.set_zorder(0)
    for artist in artists:
        if artist is not None:
            artist.set_zorder(15)


def _basemap_zoom_for_extent(xmin, ymin, xmax, ymax, max_zoom=19):
    """Pick tile zoom from Web Mercator bounds; cap to avoid Esri 'not available' tiles."""
    from contextily.tile import _calculate_zoom, _sm2ll
    w, s = _sm2ll(xmin, ymin)
    e, n = _sm2ll(xmax, ymax)
    return min(int(_calculate_zoom(w, s, e, n)), max_zoom)


def add_satellite_basemap(ax, lon, lat, pad_fraction=0.25, min_pad_m=40.0, use_tiles=True):
    """
    Esri World Imagery satellite tiles (Google Earth–style; no API key).
    Returns (mx, my, ok, span). On failure or use_tiles=False, lat/lon fallback.
    """
    if not use_tiles:
        mx, my = lon, lat
        ax.set_xlim(lon.min() - 1e-4, lon.max() + 1e-4)
        ax.set_ylim(lat.min() - 1e-4, lat.max() + 1e-4)
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.grid(True, alpha=0.4)
        span = max(float(np.ptp(mx)), float(np.ptp(my)), 1e-3)
        print("  Map: lat/lon axes only (--lite, no tile download)")
        return mx, my, False, span
    # Esri has no imagery above ~zoom 19; smaller pads caused zoom 22+ placeholder tiles
    MAX_TILE_ZOOM = 19
    MIN_VIEW_M = 200.0

    mx, my = wgs84_to_web_mercator(lon, lat)
    span = max(float(np.ptp(mx)), float(np.ptp(my)), 1e-3)
    view_m = max(MIN_VIEW_M, span * 1.5, min_pad_m)
    pad = max((view_m - span) / 2.0, 3.0)
    if span >= 20.0:
        pad = max(min_pad_m, pad_fraction * span)

    xmin, xmax = mx.min() - pad, mx.max() + pad
    ymin, ymax = my.min() - pad, my.max() + pad
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    zoom = _basemap_zoom_for_extent(xmin, ymin, xmax, ymax, MAX_TILE_ZOOM)

    try:
        import contextily as cx
        tile_sources = [
            ('Esri satellite', cx.providers.Esri.WorldImagery),
            ('OpenStreetMap', cx.providers.OpenStreetMap.Mapnik),
        ]
        for name, source in tile_sources:
            try:
                cx.add_basemap(
                    ax,
                    source=source,
                    attribution_size=7,
                    zoom=zoom,
                )
                for img in ax.images:
                    img.set_zorder(0)
                print(f"  Map tiles: {name} (zoom {zoom}, view ~{xmax - xmin:.0f} m)")
                return mx, my, True, span
            except Exception as exc:
                print(f"  {name} tiles failed ({exc}); trying next source...")
    except ImportError as exc:
        print(f"  contextily not installed ({exc}); using lat/lon axes.")

    print("  Map tiles unavailable; using lat/lon axes.")
    ax.set_xlim(lon.min() - 1e-4, lon.max() + 1e-4)
    ax.set_ylim(lat.min() - 1e-4, lat.max() + 1e-4)
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.grid(True, alpha=0.4)
    return lon, lat, False, span


def add_map_path_inset(ax_map, map_x, map_y, x_cm, y_cm, path_span_m):
    """
    Zoomed inset for paths smaller than the satellite view can show.
    Uses local East/North (cm) so RTK motion is visible.
    """
    inset_pad_m = max(2.0, path_span_m * 0.35)
    rect = Rectangle(
        (map_x.min() - inset_pad_m, map_y.min() - inset_pad_m),
        float(np.ptp(map_x)) + 2 * inset_pad_m,
        float(np.ptp(map_y)) + 2 * inset_pad_m,
        linewidth=2, edgecolor='#ffff00', facecolor='none', linestyle='--', zorder=18,
    )
    ax_map.add_patch(rect)

    ax_inset = ax_map.inset_axes([0.03, 0.03, 0.46, 0.46])
    ax_inset.set_facecolor('#1a1a1a')
    ax_inset.patch.set_alpha(0.88)
    pad_cm = max(8.0, float(np.ptp(x_cm)) * 0.35, float(np.ptp(y_cm)) * 0.35)
    cx_cm, cy_cm = float(np.mean(x_cm)), float(np.mean(y_cm))
    ax_inset.set_xlim(cx_cm - pad_cm, cx_cm + pad_cm)
    ax_inset.set_ylim(cy_cm - pad_cm, cy_cm + pad_cm)
    ax_inset.set_aspect('equal')
    ax_inset.grid(True, color='white', alpha=0.25)
    ax_inset.tick_params(colors='white', labelsize=7)
    ax_inset.set_xlabel('East (cm)', color='white', fontsize=8)
    ax_inset.set_ylabel('North (cm)', color='white', fontsize=8)
    ax_inset.set_title(
        f'Path detail ({path_span_m * 100:.0f} cm span)',
        color='white', fontsize=9,
    )

    trail_inset, = ax_inset.plot([], [], '-', color='#00ffaa', linewidth=2.5, zorder=10)
    cur_inset, = ax_inset.plot([], [], 'o', markersize=10, markerfacecolor='orange',
                               markeredgecolor='white', markeredgewidth=1.2, zorder=14)
    ax_inset.plot(x_cm[0], y_cm[0], 'o', markersize=9, markerfacecolor='lime',
                  markeredgecolor='white', zorder=14)

    return {
        'ax': ax_inset,
        'trail': trail_inset,
        'current': cur_inset,
        'rect': rect,
    }


def time_deltas_seconds(df):
    """Seconds between consecutive samples (GPS TOW preferred)."""
    ts = gps_timestamp_values(df)
    if is_gps_time_of_week(ts):
        dt = np.diff(ts)
    else:
        cal = pd.to_datetime(df[['year', 'month', 'day', 'hour', 'minute', 'second']])
        dt = np.diff(cal.values.astype('datetime64[ns]')).astype(float) / 1e9
    dt[dt <= 0] = np.nan
    dt = np.nan_to_num(dt, nan=1.0)
    dt[dt < 1e-6] = 1e-6
    return dt


def prompt_choice(prompt, options, default):
    """Interactive menu with non-interactive (EOF) default."""
    print(prompt)
    for key, label in options.items():
        print(f"  {key} - {label}")
    try:
        choice = int(input(f"Enter choice ({'/'.join(map(str, options))}): "))
    except (EOFError, ValueError):
        choice = default
        print(f"Using default: {choice}")
    return choice if choice in options else default


def gnss_visualizer(filename, data_choice=None, test_choice=None, lite=False):
    try:
        # Load data
        print(f"Loading {filename}...")
        data = pd.read_csv(filename)
        
        # Check for carrier solution (full file, before RTK mask)
        if 'carrier_solution' in data.columns:
            carr_soln = data['carrier_solution'].values
            print("\nCarrier Solution Stats:")
            print(f"  No RTK: {np.sum(carr_soln == 0)}")
            print(f"  RTK Float: {np.sum(carr_soln == 1)}")
            print(f"  RTK Fix: {np.sum(carr_soln == 2)}")
        else:
            carr_soln = np.zeros_like(fix_type)
            print("\nNo carrier solution found")

        choice = data_choice
        if choice is None:
            choice = prompt_choice(
                "\nWhat data do you want to analyze?",
                {1: "RTK Fix only (best accuracy)", 2: "All data (with outlier filtering)"},
                default=1,
            )

        if choice == 1:
            work_df = data.loc[carr_soln == 2].reset_index(drop=True)
            print(f"\nFiltering to RTK Fix only: {len(work_df)} of {len(data)} points")
            use_rtk_only = True
        else:
            print("\nUsing all data with outlier filtering")
            work_df = data.reset_index(drop=True)
            use_rtk_only = False

        lat = work_df['latitude'].values
        lon = work_df['longitude'].values
        alt = work_df['altitude_ellipsoid'].values
        fix_type = work_df['fix_type'].values
        carr_soln = work_df['carrier_solution'].values if 'carrier_solution' in work_df.columns else np.zeros(len(work_df))
        time_data = build_time_series(work_df)

        choice_2 = test_choice
        if choice_2 is None:
            choice_2 = prompt_choice(
                "\nWhat sort of test are you doing?",
                {1: "Stationary", 2: "Buoy/Walking"},
                default=1,
            )

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
        t_all = elapsed_seconds_from_df(work_df)
        print(f"Duration (GPS): {t_all[-1]:.1f} s  |  median dt: {np.median(np.diff(t_all)):.3f} s")

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
            
            dt = time_deltas_seconds(work_df)

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
        lat_filt = lat[valid]
        lon_filt = lon[valid]
        x_filt = x[valid]
        y_filt = y[valid]
        z_filt = z[valid]
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

        colors = fix_quality_colors(carr_filt, fix_filt)

        filt_df = work_df.iloc[valid].reset_index(drop=True)
        t_sec = elapsed_seconds_from_df(filt_df)
        dup_frac = 1.0 - len(np.unique(np.round(t_sec, 3))) / max(len(t_sec), 1)
        if dup_frac > 0.05:
            print(f"  Note: {dup_frac * 100:.1f}% of points share rounded timestamps (sub-ms resolution)")

        dist = np.sqrt((x_filt - np.mean(x_filt))**2 + (y_filt - np.mean(y_filt))**2) * 100
        n_pts = len(x_filt)
        mode_str = 'RTK Fix Only' if use_rtk_only else 'All Data (filtered)'
        x_cm = x_filt * 100
        y_cm = y_filt * 100
        x_plot_cm = x_cm
        y_plot_cm = y_cm
        z_plot_cm = z_filt * 100

        # Plots (2x4: satellite map spans center columns)
        fig = plt.figure(figsize=(18, 9), dpi=90 if lite else 96)
        fig.canvas.manager.set_window_title(f'GNSS Data: {filename}')
        enable_3d = not lite
        gs = fig.add_gridspec(2, 4, hspace=0.35, wspace=0.35)

        # 1. 2D Position (local cm) — line trail + markers (faster than scatter playback)
        ax1 = fig.add_subplot(gs[0, 0])
        bg_step = max(1, n_pts // 1500)
        ax1.plot(
            x_cm[::bg_step], y_cm[::bg_step], ',', color='gray', alpha=0.35,
            markersize=2, label='All points',
        )
        trail1, = ax1.plot([], [], '-', color='#00bb66', linewidth=1.5, alpha=0.9, label='Played')
        cur1, = ax1.plot([], [], 'o', color='orange', markersize=9, markeredgecolor='white',
                         markeredgewidth=0.8, label='Current')
        ax1.plot(x_cm[0], y_cm[0], 'o', color='lime', markersize=9, markeredgecolor='white',
                 markeredgewidth=0.8, label='Start')
        th = np.linspace(0, 2*np.pi, 100)
        ax1.plot(np.mean(x_filt)*100 + std_horiz*100*np.cos(th),
                 np.mean(y_filt)*100 + std_horiz*100*np.sin(th), 'b--', label='1-sigma')
        ax1.grid(True)
        ax1.set_xlabel('East (cm)')
        ax1.set_ylabel('North (cm)')
        ax1.set_title(f'Position - Horiz Std: {std_horiz*100:.2f} cm')
        ax1.axis('equal')
        ax1.legend()

        # 2. Satellite map overlay
        ax_map = fig.add_subplot(gs[0, 1:3])
        if lite:
            print("\nMap: lat/lon mode (--lite, skipping satellite tiles)")
        else:
            print("\nLoading satellite map tiles (requires internet)...")
        map_x, map_y, map_mercator, path_span_m = add_satellite_basemap(
            ax_map, lon_filt, lat_filt, use_tiles=not lite,
        )

        trail_map, = ax_map.plot([], [], '-', color='#00ffaa', linewidth=2.5, alpha=0.9,
                                 solid_capstyle='round', zorder=10, label='Path')
        cur_map, = ax_map.plot([], [], 'o', markersize=12, markerfacecolor='orange',
                               markeredgecolor='white', markeredgewidth=1.5, zorder=14)
        start_map, = ax_map.plot([map_x[0]], [map_y[0]], 'o', markersize=11,
                                 markerfacecolor='lime', markeredgecolor='white',
                                 markeredgewidth=1.5, zorder=14, label='Start')
        raise_map_artists(ax_map, trail_map, cur_map, start_map)

        # Satellite view is ~200 m wide; sub-meter paths need a zoom inset
        map_inset = None
        if not lite and path_span_m < 150.0:
            map_inset = add_map_path_inset(ax_map, map_x, map_y, x_cm, y_cm, path_span_m)
            print(f"  Path inset added (track span {path_span_m * 100:.1f} cm on satellite view)")

        if map_mercator:
            ax_map.set_xlabel('Eastings (m, Web Mercator)')
            ax_map.set_ylabel('Northings (m, Web Mercator)')
        ax_map.set_title('Satellite map + path detail (yellow box = zoom area)')
        ax_map.legend(loc='upper right', fontsize=8)

        # 3. East vs Time
        ax2 = fig.add_subplot(gs[0, 3])
        trail2, = ax2.plot([], [], '-', color='#00bb66', linewidth=1.2, alpha=0.85)
        cur2, = ax2.plot([], [], 'o', color='orange', markersize=5)
        vline2 = ax2.axvline(t_sec[0], color='k', linestyle=':', alpha=0.7)
        ax2.axhline(np.mean(x_filt)*100, color='b', linestyle='-')
        ax2.axhline(np.mean(x_filt)*100 + std_x*100, color='b', linestyle='--')
        ax2.axhline(np.mean(x_filt)*100 - std_x*100, color='b', linestyle='--')
        ax2.set_xlim(0, max(t_sec[-1], 1.0))
        ax2.grid(True)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('East (cm)')
        ax2.set_title(f'East-West vs Time - Std: {std_x*100:.2f} cm')

        # 4. North vs Time
        ax3 = fig.add_subplot(gs[1, 0])
        trail3, = ax3.plot([], [], '-', color='#00bb66', linewidth=1.2, alpha=0.85)
        cur3, = ax3.plot([], [], 'o', color='orange', markersize=5)
        vline3 = ax3.axvline(t_sec[0], color='k', linestyle=':', alpha=0.7)
        ax3.axhline(np.mean(y_filt)*100, color='b', linestyle='-')
        ax3.axhline(np.mean(y_filt)*100 + std_y*100, color='b', linestyle='--')
        ax3.axhline(np.mean(y_filt)*100 - std_y*100, color='b', linestyle='--')
        ax3.set_xlim(0, max(t_sec[-1], 1.0))
        ax3.grid(True)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('North (cm)')
        ax3.set_title(f'North-South vs Time - Std: {std_y*100:.2f} cm')

        # 5. Altitude vs Time
        ax4 = fig.add_subplot(gs[1, 1])
        trail4, = ax4.plot([], [], '-', color='#00bb66', linewidth=1.2, alpha=0.85)
        cur4, = ax4.plot([], [], 'o', color='orange', markersize=5)
        vline4 = ax4.axvline(t_sec[0], color='k', linestyle=':', alpha=0.7)
        ax4.axhline(np.mean(z_filt)*100, color='b', linestyle='-')
        ax4.set_xlim(0, max(t_sec[-1], 1.0))
        ax4.grid(True)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Altitude (cm)')
        ax4.set_title(f'Vertical vs Time - Std: {std_z*100:.2f} cm')

        # 6. 3D trajectory (ENU) — optional; hidden during Play for performance
        ax5 = fig.add_subplot(gs[1, 2], projection='3d' if enable_3d else None)
        trail5 = cur5 = None
        if enable_3d:
            trail5, = ax5.plot([], [], [], '-', color='#00aa66', linewidth=1.2, alpha=0.7)
            cur5, = ax5.plot([], [], [], 'o', markersize=9, markerfacecolor='orange',
                             markeredgecolor='white', markeredgewidth=0.8)
            ax5.plot([x_plot_cm[0]], [y_plot_cm[0]], [z_plot_cm[0]], 'o', markersize=9,
                     markerfacecolor='lime', markeredgecolor='white', markeredgewidth=0.8)
            ax5.set_xlabel('East (cm)')
            ax5.set_ylabel('North (cm)')
            ax5.set_zlabel('Altitude (cm)')
            ax5.set_title('3D Trajectory (ENU)')
            configure_trajectory_3d(
                ax5, x_plot_cm, y_plot_cm, z_plot_cm,
                std_horiz_cm=std_horiz * 100,
            )
        else:
            ax5.text(
                0.5, 0.5, '3D view disabled\n(use default mode)',
                ha='center', va='center', transform=ax5.transAxes, fontsize=11,
            )
            ax5.set_xticks([])
            ax5.set_yticks([])

        # 7. Histogram (static — stats do not change during playback)
        ax6 = fig.add_subplot(gs[1, 3])
        ax6.hist(dist, bins=min(30, max(10, n_pts // 15)), color=[0.3, 0.7, 0.3])
        ax6.axvline(std_horiz*100, color='r', linestyle='--', label='1-sigma')
        ax6.axvline(2*std_horiz*100, color='r', linestyle=':', label='2-sigma')
        ax6.grid(True)
        ax6.set_xlabel('Distance from Mean (cm)')
        ax6.set_ylabel('Count')
        ax6.set_title('Position Distribution')
        ax6.legend()

        title = fig.suptitle('', fontsize=14, fontweight='bold')

        cur_color = colors[0]
        title_tpl = (
            f"{filename} - {mode_str} - {test_str}\n"
            "Time: {elapsed:.1f} s / {total:.1f} s  |  "
            "Point {i}/{n}  |  Horiz Std: {std:.2f} cm"
        )

        def update_frame(i, fast=False):
            nonlocal cur_color
            sl = slice(i)
            idx = i - 1
            cur_color = colors[idx]
            t_now = t_sec[idx]

            trail1.set_data(x_cm[sl], y_cm[sl])
            cur1.set_data([x_cm[idx]], [y_cm[idx]])
            cur1.set_color(cur_color)

            trail2.set_data(t_sec[sl], x_cm[sl])
            cur2.set_data([t_sec[idx]], [x_cm[idx]])
            trail3.set_data(t_sec[sl], y_cm[sl])
            cur3.set_data([t_sec[idx]], [y_cm[idx]])
            trail4.set_data(t_sec[sl], z_plot_cm[sl])
            cur4.set_data([t_sec[idx]], [z_plot_cm[idx]])

            vline2.set_xdata([t_now, t_now])
            vline3.set_xdata([t_now, t_now])
            vline4.set_xdata([t_now, t_now])

            # Heavy panels: hide satellite/3D during Play (tile + 3D redraws dominate CPU)
            if fast:
                if enable_3d and ax5.get_visible():
                    ax5.set_visible(False)
                if map_mercator and ax_map.get_visible():
                    ax_map.set_visible(False)
                elif not map_mercator:
                    trail_map.set_data(map_x[sl], map_y[sl])
                    cur_map.set_data([map_x[idx]], [map_y[idx]])
            else:
                if map_mercator and not ax_map.get_visible():
                    ax_map.set_visible(True)
                trail_map.set_data(map_x[sl], map_y[sl])
                cur_map.set_data([map_x[idx]], [map_y[idx]])

                if map_inset is not None:
                    map_inset['trail'].set_data(x_cm[sl], y_cm[sl])
                    map_inset['current'].set_data([x_cm[idx]], [y_cm[idx]])

                if enable_3d:
                    if not ax5.get_visible():
                        ax5.set_visible(True)
                    trail5.set_data(x_plot_cm[sl], y_plot_cm[sl])
                    trail5.set_3d_properties(z_plot_cm[sl])
                    cur5.set_data([x_plot_cm[idx]], [y_plot_cm[idx]])
                    cur5.set_3d_properties([z_plot_cm[idx]])

            if not fast or i % 8 == 0 or i == n_pts:
                title.set_text(title_tpl.format(
                    elapsed=t_now, total=t_sec[-1], i=i, n=n_pts, std=std_horiz * 100,
                ))

        fig.subplots_adjust(left=0.05, right=0.98, top=0.90, bottom=0.12, hspace=0.4, wspace=0.35)
        play_stride = max(1, n_pts // 80)
        gnss_anim = add_playback_controls(
            fig, n_pts, update_frame, play_stride=play_stride, interval_ms=90,
        )
        _keep_anim_alive = gnss_anim  # hold ref until window closes
        print(
            f"\nPlayback: slider=1 point/step | Play={play_stride} pts @ 90ms "
            f"(pauses map/3D during Play; use --lite for more speed)"
        )
        if lite:
            print("  --lite: no satellite tiles, no 3D panel")
        
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
    parser.add_argument(
        '--data', type=int, choices=[1, 2], default=None,
        help='1=RTK Fix only, 2=all data with outlier filtering (skip prompt)',
    )
    parser.add_argument(
        '--test', type=int, choices=[1, 2], default=None,
        help='1=stationary thresholds, 2=buoy/walking (skip prompt)',
    )
    parser.add_argument(
        '--lite', action='store_true',
        help='Faster mode: no satellite tiles, no 3D panel, lower DPI',
    )
    args = parser.parse_args()

    gnss_visualizer(
        args.filename,
        data_choice=args.data,
        test_choice=args.test,
        lite=args.lite,
    )
