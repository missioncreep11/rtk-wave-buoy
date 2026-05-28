"""
Interactive altitude vs time plot for parsed GNSS CSV logs.
GUI for options; CLI still supported. Optional wave period / spectrum analysis.
"""

import argparse
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.interpolate import interp1d
from scipy.signal import butter, detrend, sosfiltfilt, welch

from gnss_visualizer import elapsed_seconds_from_df

PLOT_CONFIG = {
    'scrollZoom': True,
    'displayModeBar': True,
    'displaylogo': False,
}


@dataclass
class RunConfig:
    filename: str
    rtk_only: bool = False
    absolute: bool = False
    t_start: float | None = None
    t_end: float | None = None
    period_min: float = 3.0
    period_max: float = 25.0
    fs: float | None = None
    bandpass: bool = True
    linear_detrend: bool = True
    welch_segment: int | None = None


def load_altitude_series(filename, rtk_only=False, relative=True):
    """Return elapsed seconds and altitude (m) from a parsed dataLog CSV."""
    print(f"Loading {filename}...")
    df = pd.read_csv(filename)

    if 'altitude_ellipsoid' not in df.columns:
        raise ValueError("CSV must include an 'altitude_ellipsoid' column")

    if rtk_only:
        if 'carrier_solution' not in df.columns:
            raise ValueError("RTK filter requested but 'carrier_solution' column is missing")
        df = df.loc[df['carrier_solution'] == 2].reset_index(drop=True)
        print(f"RTK Fix only: {len(df)} points")

    t_sec = elapsed_seconds_from_df(df)
    alt_m = df['altitude_ellipsoid'].astype(np.float64).values

    med = np.nanmedian(alt_m)
    keep = np.abs(alt_m - med) < 100.0
    n_drop = int(np.sum(~keep))
    if n_drop:
        print(f"Dropped {n_drop} altitude outlier(s) (>100 m from median)")
        t_sec = t_sec[keep]
        alt_m = alt_m[keep]

    if relative:
        alt_m = alt_m - np.nanmedian(alt_m)

    return t_sec, alt_m, df


def slice_time_window(t_sec, alt_m, t_start=None, t_end=None):
    """Keep samples with elapsed time in [t_start, t_end] (seconds from first sample)."""
    t_sec = np.asarray(t_sec, dtype=np.float64)
    alt_m = np.asarray(alt_m, dtype=np.float64)

    if t_start is None and t_end is None:
        return t_sec, alt_m, None

    if t_start is not None and t_end is not None and t_start >= t_end:
        raise ValueError(f't-start ({t_start}) must be less than t-end ({t_end})')

    mask = np.ones(len(t_sec), dtype=bool)
    if t_start is not None:
        mask &= t_sec >= t_start
    if t_end is not None:
        mask &= t_sec <= t_end

    n_keep = int(np.sum(mask))
    if n_keep < 2:
        raise ValueError(
            f'Time window has {n_keep} point(s); need at least 2. '
            f'Record spans {t_sec[0]:.1f} - {t_sec[-1]:.1f} s.',
        )

    label = _format_time_window(t_start, t_end)
    print(f"Time window: {label}  ->  {n_keep} points, {t_sec[mask][-1] - t_sec[mask][0]:.1f} s span")
    return t_sec[mask], alt_m[mask], label


def _format_time_window(t_start, t_end):
    if t_start is not None and t_end is not None:
        return f'{t_start:.1f} - {t_end:.1f} s (elapsed)'
    if t_start is not None:
        return f'{t_start:.1f} s to end'
    return f'start to {t_end:.1f} s'


def infer_sample_rate(t_sec):
    dt = np.diff(np.asarray(t_sec, dtype=np.float64))
    dt = dt[dt > 1e-6]
    if len(dt) == 0:
        return 1.0
    return float(1.0 / np.median(dt))


def resample_uniform(t_sec, alt_m, fs):
    t_sec = np.asarray(t_sec, dtype=np.float64)
    alt_m = np.asarray(alt_m, dtype=np.float64)
    t0, t1 = t_sec[0], t_sec[-1]
    n = int(np.floor((t1 - t0) * fs)) + 1
    if n < 4:
        raise ValueError('Record too short for spectral analysis')
    t_uniform = t0 + np.arange(n) / fs
    interp = interp1d(t_sec, alt_m, kind='linear', fill_value='extrapolate')
    return t_uniform, interp(t_uniform)


def bandpass_waves(data, fs, period_min_s, period_max_s, order=4):
    f_low = 1.0 / period_max_s
    f_high = 1.0 / period_min_s
    nyq = 0.5 * fs
    if f_high >= nyq * 0.98:
        f_high = nyq * 0.98
    if f_low <= 1e-4:
        f_low = 1e-4
    if f_low >= f_high:
        raise ValueError(
            f'Invalid period band: {period_min_s}-{period_max_s} s at fs={fs} Hz '
            f'(Nyquist period = {2/fs:.2f} s)',
        )
    sos = butter(order, [f_low, f_high], btype='band', fs=fs, output='sos')
    return sosfiltfilt(sos, data)


def prepare_heave_signal(t_sec, alt_m, fs, linear_detrend=True, bandpass=True,
                         period_min_s=3.0, period_max_s=25.0):
    t_u, eta = resample_uniform(t_sec, alt_m, fs)
    if linear_detrend:
        eta = detrend(eta, type='linear')
    eta = eta - np.mean(eta)
    if bandpass:
        eta = bandpass_waves(eta, fs, period_min_s, period_max_s)
    return t_u, eta


def compute_wave_spectrum(eta, fs, period_min_s, period_max_s, nperseg=None):
    n = len(eta)
    if nperseg is None:
        nperseg = min(256, max(64, n // 8))
    nperseg = min(nperseg, n)

    freqs, psd = welch(eta, fs=fs, nperseg=nperseg, noverlap=nperseg // 2,
                       window='hann', scaling='density')

    f_min = 1.0 / period_max_s
    f_max = 1.0 / period_min_s
    band = (freqs >= f_min) & (freqs <= f_max) & (freqs > 0)
    if not np.any(band):
        raise ValueError('No spectral bins in the requested period band')

    psd_band = psd[band]
    freqs_band = freqs[band]
    i_peak = int(np.argmax(psd_band))
    f_peak = float(freqs_band[i_peak])
    T_peak = 1.0 / f_peak

    df = np.diff(freqs)
    df = np.append(df, df[-1] if len(df) else 1.0)
    m0 = float(np.sum(psd_band * df[band]))
    m1 = float(np.sum(freqs_band * psd_band * df[band]))
    T_mean = m0 / m1 if m1 > 0 else float('nan')

    eta_std = float(np.std(eta))
    return freqs, psd, {
        'f_peak_hz': f_peak,
        'T_peak_s': T_peak,
        'T_mean_s': T_mean,
        'm0': m0,
        'Hs_4std_m': 4.0 * eta_std,
        'Hs_spec_m': 4.0 * np.sqrt(m0) if m0 > 0 else float('nan'),
        'eta_std_m': eta_std,
        'freq_resolution_hz': freqs[1] - freqs[0] if len(freqs) > 1 else 0.0,
    }


def format_wave_report(metrics, fs, period_min_s, period_max_s, duration_s, n_samples,
                       time_window=None):
    lines = ['=== Wave analysis ===']
    if time_window:
        lines.append(f'  Segment: {time_window}')
    lines.extend([
        f'  Samples (uniform @ {fs:.3f} Hz): {n_samples}',
        f'  Duration: {duration_s:.1f} s',
        f'  Search band: {period_min_s:.1f} - {period_max_s:.1f} s',
        f'  Nyquist period: {2.0 / fs:.2f} s  |  freq resolution ~ {metrics["freq_resolution_hz"]:.4f} Hz',
        f'  Peak period Tp: {metrics["T_peak_s"]:.2f} s  (f = {metrics["f_peak_hz"]:.4f} Hz)',
        f'  Mean period (spectral): {metrics["T_mean_s"]:.2f} s',
        f'  Heave std: {metrics["eta_std_m"] * 100:.2f} cm',
        f'  Hs (4*std): {metrics["Hs_4std_m"]:.3f} m',
        f'  Hs (4*sqrt(m0), narrow-band): {metrics["Hs_spec_m"]:.3f} m',
    ])
    return '\n'.join(lines)


def print_wave_report(metrics, fs, period_min_s, period_max_s, duration_s, n_samples,
                      time_window=None):
    print('\n' + format_wave_report(
        metrics, fs, period_min_s, period_max_s, duration_s, n_samples, time_window,
    ))


def plot_altitude_interactive(t_sec, alt_m, title="Altitude vs time", y_title="Altitude (m)"):
    fig = go.Figure(
        data=go.Scatter(
            x=t_sec, y=alt_m, mode='lines', name='Altitude',
            line=dict(color='#2d70b3', width=2),
            hovertemplate='Time: %{x:.3f} s<br>Altitude: %{y:.4f} m<extra></extra>',
        ),
    )
    _style_time_axis(fig, title, y_title)
    fig.show(config=PLOT_CONFIG)


def plot_time_and_spectrum(t_sec, alt_m, t_heave, eta, freqs, psd, metrics,
                           title, y_title, period_min_s, period_max_s):
    periods_all = 1.0 / np.maximum(freqs, 1e-9)

    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=False, vertical_spacing=0.12,
        subplot_titles=(
            'Heave (processed for waves)',
            f'Power spectrum (band {period_min_s:.0f}-{period_max_s:.0f} s)',
        ),
    )

    fig.add_trace(
        go.Scatter(x=t_sec, y=alt_m, mode='lines', name='Raw delta altitude',
                   line=dict(color='#9aa5b1', width=1), opacity=0.5),
        row=1, col=1,
    )
    fig.add_trace(
        go.Scatter(x=t_heave, y=eta, mode='lines', name='Filtered heave',
                   line=dict(color='#2d70b3', width=2),
                   hovertemplate='Time: %{x:.1f} s<br>Heave: %{y:.4f} m<extra></extra>'),
        row=1, col=1,
    )
    fig.add_trace(
        go.Scatter(x=periods_all, y=psd, mode='lines', name='PSD',
                   line=dict(color='#2d70b3', width=2),
                   hovertemplate='Period: %{x:.2f} s<br>PSD: %{y:.2e}<extra></extra>'),
        row=2, col=1,
    )

    Tp = metrics['T_peak_s']
    fig.add_vline(
        x=Tp, line_dash='dash', line_color='#e65c00', line_width=2,
        annotation_text=f'Tp = {Tp:.1f} s', annotation_position='top right',
        row=2, col=1,
    )

    fig.update_xaxes(title_text='Elapsed time (s)', row=1, col=1)
    fig.update_yaxes(title_text=y_title, row=1, col=1)
    fig.update_xaxes(title_text='Period (s)', row=2, col=1, range=[period_max_s, period_min_s])
    fig.update_yaxes(title_text='PSD (m^2/Hz)', row=2, col=1, type='log')
    fig.update_layout(
        title=title, template='plotly_white', height=720,
        dragmode='pan', hovermode='x unified',
        margin=dict(l=60, r=30, t=80, b=50),
    )
    fig.show(config=PLOT_CONFIG)


def _style_time_axis(fig, title, y_title):
    fig.update_layout(
        title=title, template='plotly_white', dragmode='pan', hovermode='x unified',
        xaxis=dict(title='Elapsed time (s)', showgrid=True, gridcolor='#e8e8e8',
                   spikesnap='cursor', showspikes=True, spikemode='across'),
        yaxis=dict(title=y_title, showgrid=True, gridcolor='#e8e8e8', zeroline=True,
                   zerolinecolor='#ccc', spikesnap='cursor', showspikes=True, spikemode='across'),
        margin=dict(l=60, r=30, t=50, b=50),
    )


def _parse_optional_float(text):
    text = text.strip()
    if not text:
        return None
    return float(text)


def _parse_optional_int(text):
    text = text.strip()
    if not text:
        return None
    return int(text)


def effective_period_min(period_min, fs):
    nyquist_period = 2.0 / fs
    if period_min < nyquist_period:
        print(
            f'  Note: period-min ({period_min} s) below Nyquist ({nyquist_period:.2f} s); '
            'using Nyquist as minimum.',
        )
        return max(period_min, nyquist_period * 1.02)
    return period_min


def run_normal_plot(cfg: RunConfig):
    """Original full-record interactive altitude graph."""
    t_sec, alt_m, _ = load_altitude_series(
        cfg.filename, rtk_only=cfg.rtk_only, relative=not cfg.absolute,
    )
    if len(t_sec) < 2:
        raise ValueError('Need at least 2 points to plot.')

    duration = float(t_sec[-1] - t_sec[0])
    print(f"Full record: {len(t_sec)} points  |  {t_sec[0]:.1f} - {t_sec[-1]:.1f} s ({duration:.1f} s)")

    y_title = 'Ellipsoid height (m)' if cfg.absolute else 'Delta altitude vs median (m)'
    plot_altitude_interactive(t_sec, alt_m, title=Path(cfg.filename).name, y_title=y_title)
    return f'Plotted full record ({len(t_sec)} points, {duration:.1f} s).'


def run_wave_analysis(cfg: RunConfig):
    """Wave analysis on optional time segment."""
    if cfg.period_min <= 0 or cfg.period_max <= 0 or cfg.period_min >= cfg.period_max:
        raise ValueError('Period min must be positive and less than period max.')

    t_sec, alt_m, _ = load_altitude_series(
        cfg.filename, rtk_only=cfg.rtk_only, relative=not cfg.absolute,
    )
    if len(t_sec) < 2:
        raise ValueError('Need at least 2 points.')

    duration_full = float(t_sec[-1] - t_sec[0])
    print(f"Full record: {len(t_sec)} points  |  {t_sec[0]:.1f} - {t_sec[-1]:.1f} s ({duration_full:.1f} s)")

    time_window = None
    if cfg.t_start is not None or cfg.t_end is not None:
        t_sec, alt_m, time_window = slice_time_window(t_sec, alt_m, cfg.t_start, cfg.t_end)

    duration = float(t_sec[-1] - t_sec[0])
    print(f"Using: {len(t_sec)} points  |  Duration: {duration:.1f} s")

    fs = cfg.fs if cfg.fs is not None else infer_sample_rate(t_sec)
    period_min = effective_period_min(cfg.period_min, fs)

    min_span = max(3.0 * cfg.period_max, 30.0)
    if duration < min_span:
        print(f'  Warning: segment is {duration:.1f} s; {min_span:.0f} s+ recommended.')

    t_heave, eta = prepare_heave_signal(
        t_sec, alt_m, fs,
        linear_detrend=cfg.linear_detrend,
        bandpass=cfg.bandpass,
        period_min_s=period_min,
        period_max_s=cfg.period_max,
    )

    freqs, psd, metrics = compute_wave_spectrum(
        eta, fs, period_min, cfg.period_max, nperseg=cfg.welch_segment,
    )

    report = format_wave_report(
        metrics, fs, period_min, cfg.period_max,
        duration_s=float(t_heave[-1] - t_heave[0]),
        n_samples=len(eta), time_window=time_window,
    )
    print('\n' + report)

    y_title = 'Ellipsoid height (m)' if cfg.absolute else 'Delta altitude vs median (m)'
    title = Path(cfg.filename).name
    if time_window:
        title = f'{title}  [{time_window}]'

    plot_time_and_spectrum(
        t_sec, alt_m, t_heave, eta, freqs, psd, metrics,
        title=title, y_title=y_title,
        period_min_s=period_min, period_max_s=cfg.period_max,
    )
    return report


class AltitudeGui:
    def __init__(self, root: tk.Tk, initial_file: str | None = None):
        self.root = root
        root.title('RTK Altitude Visualizer')
        root.minsize(520, 620)

        pad = {'padx': 8, 'pady': 4}

        file_row = ttk.Frame(root)
        file_row.pack(fill='x', **pad)
        ttk.Label(file_row, text='CSV file:').pack(side='left')
        self.file_var = tk.StringVar(value=initial_file or '')
        ttk.Entry(file_row, textvariable=self.file_var, width=48).pack(side='left', fill='x', expand=True, padx=4)
        ttk.Button(file_row, text='Browse...', command=self._browse).pack(side='left')

        opts = ttk.LabelFrame(root, text='Data options')
        opts.pack(fill='x', **pad)
        self.rtk_var = tk.BooleanVar(value=True)
        self.abs_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text='RTK Fix only', variable=self.rtk_var).grid(row=0, column=0, sticky='w', padx=8, pady=2)
        ttk.Checkbutton(opts, text='Absolute ellipsoid height (no median subtract)', variable=self.abs_var).grid(
            row=0, column=1, sticky='w', padx=8, pady=2,
        )

        seg = ttk.LabelFrame(root, text='Time segment (wave analyze only)')
        seg.pack(fill='x', **pad)
        self.use_window_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(seg, text='Limit analysis to time window (s, elapsed from start)',
                        variable=self.use_window_var, command=self._toggle_window).grid(
            row=0, column=0, columnspan=4, sticky='w', padx=8, pady=2,
        )
        ttk.Label(seg, text='Start:').grid(row=1, column=0, padx=4, pady=2)
        self.t_start_var = tk.StringVar(value='')
        self.t_start_entry = ttk.Entry(seg, textvariable=self.t_start_var, width=10, state='disabled')
        self.t_start_entry.grid(row=1, column=1, sticky='w')
        ttk.Label(seg, text='End:').grid(row=1, column=2, padx=4, pady=2)
        self.t_end_var = tk.StringVar(value='')
        self.t_end_entry = ttk.Entry(seg, textvariable=self.t_end_var, width=10, state='disabled')
        self.t_end_entry.grid(row=1, column=3, sticky='w')

        wave = ttk.LabelFrame(root, text='Wave analysis')
        wave.pack(fill='x', **pad)
        ttk.Label(wave, text='Period min (s):').grid(row=0, column=0, sticky='e', padx=4, pady=2)
        self.pmin_var = tk.StringVar(value='3')
        ttk.Entry(wave, textvariable=self.pmin_var, width=8).grid(row=0, column=1, sticky='w')
        ttk.Label(wave, text='Period max (s):').grid(row=0, column=2, sticky='e', padx=4, pady=2)
        self.pmax_var = tk.StringVar(value='25')
        ttk.Entry(wave, textvariable=self.pmax_var, width=8).grid(row=0, column=3, sticky='w')

        ttk.Label(wave, text='Sample rate (Hz):').grid(row=1, column=0, sticky='e', padx=4, pady=2)
        self.fs_var = tk.StringVar(value='')
        ttk.Entry(wave, textvariable=self.fs_var, width=8).grid(row=1, column=1, sticky='w')
        ttk.Label(wave, text='blank = auto').grid(row=1, column=2, columnspan=2, sticky='w')

        self.bandpass_var = tk.BooleanVar(value=True)
        self.detrend_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(wave, text='Bandpass filter', variable=self.bandpass_var).grid(
            row=2, column=0, columnspan=2, sticky='w', padx=8,
        )
        ttk.Checkbutton(wave, text='Linear detrend', variable=self.detrend_var).grid(
            row=2, column=2, columnspan=2, sticky='w', padx=8,
        )

        ttk.Label(wave, text='Welch segment (samples):').grid(row=3, column=0, sticky='e', padx=4, pady=2)
        self.welch_var = tk.StringVar(value='')
        ttk.Entry(wave, textvariable=self.welch_var, width=8).grid(row=3, column=1, sticky='w')
        ttk.Label(wave, text='blank = auto').grid(row=3, column=2, columnspan=2, sticky='w')

        btn_row = ttk.Frame(root)
        btn_row.pack(fill='x', **pad)
        ttk.Button(
            btn_row, text='Plot altitude (full record)',
            command=self._on_plot_normal,
        ).pack(side='left', padx=4)
        ttk.Button(
            btn_row, text='Analyze waves',
            command=self._on_analyze,
        ).pack(side='left', padx=4)

        out = ttk.LabelFrame(root, text='Log / results')
        out.pack(fill='both', expand=True, **pad)
        self.log = scrolledtext.ScrolledText(out, height=12, state='disabled', wrap='word')
        self.log.pack(fill='both', expand=True, padx=4, pady=4)

        self._log('Plot altitude: original interactive graph (entire file).\n'
                  'Analyze waves: spectrum + period using options above.\n')

    def _toggle_window(self):
        state = 'normal' if self.use_window_var.get() else 'disabled'
        self.t_start_entry.configure(state=state)
        self.t_end_entry.configure(state=state)

    def _browse(self):
        path = filedialog.askopenfilename(
            title='Select parsed GNSS CSV',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
        )
        if path:
            self.file_var.set(path)

    def _log(self, msg: str, clear=False):
        self.log.configure(state='normal')
        if clear:
            self.log.delete('1.0', 'end')
        self.log.insert('end', msg + ('\n' if not msg.endswith('\n') else ''))
        self.log.see('end')
        self.log.configure(state='disabled')

    def _build_config(self) -> RunConfig:
        path = self.file_var.get().strip()
        if not path:
            raise ValueError('Choose a CSV file first.')
        if not Path(path).is_file():
            raise ValueError(f'File not found: {path}')

        t_start = t_end = None
        if self.use_window_var.get():
            t_start = _parse_optional_float(self.t_start_var.get())
            t_end = _parse_optional_float(self.t_end_var.get())
            if t_start is None and t_end is None:
                raise ValueError('Enter at least start or end time for the window.')

        return RunConfig(
            filename=path,
            rtk_only=self.rtk_var.get(),
            absolute=self.abs_var.get(),
            t_start=t_start,
            t_end=t_end,
            period_min=float(self.pmin_var.get()),
            period_max=float(self.pmax_var.get()),
            fs=_parse_optional_float(self.fs_var.get()),
            bandpass=self.bandpass_var.get(),
            linear_detrend=self.detrend_var.get(),
            welch_segment=_parse_optional_int(self.welch_var.get()),
        )

    def _on_plot_normal(self):
        try:
            cfg = self._build_config()
            msg = run_normal_plot(cfg)
            self._log(msg)
        except Exception as exc:
            messagebox.showerror('Plot error', str(exc))
            self._log(f'Error: {exc}')

    def _on_analyze(self):
        try:
            cfg = self._build_config()
            report = run_wave_analysis(cfg)
            self._log(report, clear=True)
        except Exception as exc:
            messagebox.showerror('Analysis error', str(exc))
            self._log(f'Error: {exc}')


def launch_gui(initial_file: str | None = None):
    root = tk.Tk()
    AltitudeGui(root, initial_file=initial_file)
    root.mainloop()


def run_cli(args):
    cfg = RunConfig(
        filename=args.filename,
        rtk_only=args.rtk,
        absolute=args.absolute,
        t_start=args.t_start,
        t_end=args.t_end,
        period_min=args.period_min,
        period_max=args.period_max,
        fs=args.fs,
        bandpass=not args.no_bandpass,
        linear_detrend=not args.no_detrend,
        welch_segment=args.welch_segment,
    )

    if args.analyze:
        run_wave_analysis(cfg)
    else:
        run_normal_plot(cfg)


def main():
    parser = argparse.ArgumentParser(
        description='Interactive altitude vs time from parsed GNSS CSV',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('filename', nargs='?', default=None, help='Parsed GNSS CSV (optional with GUI)')
    parser.add_argument('--gui', action='store_true', help='Open settings window (default if no CLI flags)')
    parser.add_argument('--rtk', action='store_true', help='RTK Fix only')
    parser.add_argument('--absolute', action='store_true', help='Absolute ellipsoid height')
    parser.add_argument('--analyze', action='store_true', help='Wave analysis (CLI)')
    parser.add_argument('--fs', type=float, default=None, help='Resample rate (Hz)')
    parser.add_argument('--period-min', type=float, default=3.0, dest='period_min')
    parser.add_argument('--period-max', type=float, default=25.0, dest='period_max')
    parser.add_argument('--no-bandpass', action='store_true')
    parser.add_argument('--no-detrend', action='store_true')
    parser.add_argument('--welch-segment', type=int, default=None, dest='welch_segment')
    parser.add_argument('--t-start', type=float, default=None, dest='t_start')
    parser.add_argument('--t-end', type=float, default=None, dest='t_end')
    args = parser.parse_args()

    cli_mode = (
        args.analyze
        or args.rtk
        or args.absolute
        or args.fs is not None
        or args.no_bandpass
        or args.no_detrend
        or args.welch_segment is not None
        or args.t_start is not None
        or args.t_end is not None
        or (args.period_min != 3.0)
        or (args.period_max != 25.0)
    )

    if args.gui or (args.filename is None) or (args.filename and not cli_mode):
        launch_gui(initial_file=args.filename)
        return

    if not args.filename:
        raise SystemExit('filename required for CLI mode')
    run_cli(args)


if __name__ == '__main__':
    main()
