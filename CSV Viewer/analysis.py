"""
Signal-analysis routines for the flight viewer (numpy + pandas only, no scipy).

Everything takes a time vector `t` in seconds and values `y`; samples may be unevenly spaced and
may contain NaNs. Spectral tools first resample onto a uniform grid at the median sample interval.
"""
from __future__ import annotations

import math

import numpy as np
import pandas as pd

WINDOWS = {
    "Hann": np.hanning,
    "Hamming": np.hamming,
    "Blackman": np.blackman,
    "Rectangular": np.ones,
}
DETREND_MODES = ("None", "Mean", "Linear")
ROLLING_STATS = ("mean", "std", "rms", "min", "max", "median")


# ----------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------
def finite_xy(t, y):
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    ok = np.isfinite(t) & np.isfinite(y)
    return t[ok], y[ok]


def uniform(t, y, max_points=4_000_000):
    """Resample onto an even grid at the median sample interval -> (tt, yy, dt) or None."""
    t, y = finite_xy(t, y)
    if len(t) < 4:
        return None
    d = np.diff(t)
    d = d[d > 0]
    if not len(d):
        return None
    dt = float(np.median(d))
    n = int(math.floor((t[-1] - t[0]) / dt)) + 1
    if n > max_points:
        n = max_points
        dt = (t[-1] - t[0]) / (n - 1)
    tt = t[0] + np.arange(n) * dt
    return tt, np.interp(tt, t, y), dt


def detrend(y, mode="Mean"):
    y = np.asarray(y, float)
    if mode == "Mean":
        return y - y.mean()
    if mode == "Linear" and len(y) > 1:
        x = np.arange(len(y), dtype=float)
        k, b = np.polyfit(x, y, 1)
        return y - (k * x + b)
    return y


def _detrend_rows(frames, mode):
    if mode == "Mean":
        return frames - frames.mean(axis=1, keepdims=True)
    if mode == "Linear":
        n = frames.shape[1]
        x = np.arange(n, dtype=float)
        A = np.vstack([x, np.ones(n)]).T
        coef, *_ = np.linalg.lstsq(A, frames.T, rcond=None)
        return frames - (A @ coef).T
    return frames


def _frames(y, nperseg, step, max_frames=None):
    view = np.lib.stride_tricks.sliding_window_view(y, nperseg)[::step]
    if max_frames and len(view) > max_frames:
        view = view[:: int(math.ceil(len(view) / max_frames))]
    return view


# ----------------------------------------------------------------------------------------------
# Statistics
# ----------------------------------------------------------------------------------------------
def times_s(u):
    """Unit of ∫x dt: deg/s -> deg, m/s² -> m/s, m -> m·s."""
    if u.endswith("/s"):
        return u[:-2]
    if u.endswith("/s²"):
        return u[:-1]
    return f"{u}·s" if u else "s"


def per_s(u):
    """Unit of dx/dt: m -> m/s, m/s -> m/s², deg/s -> deg/s²."""
    if u.endswith("/s"):
        return u + "²"
    return f"{u}/s" if u else "/s"


def describe(t, y, unit="", threshold=None):
    """Ordered list of (name, value, unit) describing one signal over its time span."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    total = len(y)
    t, y = finite_xy(t, y)
    n = len(y)
    u = unit
    rows = [("Samples", n, ""), ("Missing (NaN)", total - n, "")]
    if n == 0:
        return rows

    def add(name, value, unit_=""):
        rows.append((name, float(value) if value is not None else math.nan, unit_))

    dur = t[-1] - t[0]
    add("Start time", t[0], "s")
    add("Duration", dur, "s")
    dt = np.diff(t)
    dtp = dt[dt > 0]
    if len(dtp):
        add("Sample rate (median)", 1 / np.median(dtp), "Hz")
        add("Sample interval jitter (std)", np.std(dtp) * 1e3, "ms")
        add("Largest gap", dt.max(), "s")

    mean = y.mean()
    std = y.std(ddof=1) if n > 1 else 0.0
    rms = math.sqrt(np.mean(y ** 2))
    imin, imax = int(np.argmin(y)), int(np.argmax(y))
    add("Mean", mean, u)
    add("Median", np.median(y), u)
    add("Std deviation", std, u)
    add("Variance", std ** 2, f"{u}²" if u else "")
    add("Min", y[imin], u)
    add("Time of min", t[imin], "s")
    add("Max", y[imax], u)
    add("Time of max", t[imax], "s")
    add("Range (peak-to-peak)", y[imax] - y[imin], u)
    add("RMS", rms, u)
    add("Mean |x|", np.mean(np.abs(y)), u)
    for p in (1, 5, 25, 75, 95, 99):
        add(f"Percentile {p}", np.percentile(y, p), u)
    q1, q3 = np.percentile(y, [25, 75])
    add("Interquartile range", q3 - q1, u)
    add("Median abs deviation", np.median(np.abs(y - np.median(y))), u)
    if std > 0 and n > 2:
        z = (y - mean) / y.std()
        add("Skewness", np.mean(z ** 3))
        add("Excess kurtosis", np.mean(z ** 4) - 3)
    if rms > 0:
        add("Crest factor (max|x| / RMS)", np.max(np.abs(y)) / rms)
    if mean != 0:
        add("Coefficient of variation", std / abs(mean))

    if dur > 0:
        integral = np.trapezoid(y, t)
        add("Time-weighted mean", integral / dur, u)
        add("Integral (∫x dt)", integral, times_s(u))
        add("First value", y[0], u)
        add("Last value", y[-1], u)
        add("Net change", y[-1] - y[0], u)
        if n > 2:
            k, b = np.polyfit(t - t[0], y, 1)
            fit = k * (t - t[0]) + b
            ss_res = np.sum((y - fit) ** 2)
            ss_tot = np.sum((y - mean) ** 2)
            add("Linear trend (slope)", k, per_s(u))
            add("Linear trend R²", 1 - ss_res / ss_tot if ss_tot > 0 else math.nan)
            with np.errstate(divide="ignore", invalid="ignore"):
                rate = np.gradient(y, t)
            rate[~np.isfinite(rate)] = np.nan
            if np.isfinite(rate).any():
                ir = int(np.nanargmax(np.abs(rate)))
                add("Max rate of change", rate[ir], per_s(u))
                add("Time of max rate", t[ir], "s")
    s = np.sign(y)
    add("Zero crossings", int(np.count_nonzero(s[1:] * s[:-1] < 0)))
    sm = np.sign(y - mean)
    add("Mean crossings", int(np.count_nonzero(sm[1:] * sm[:-1] < 0)))

    u3 = uniform(t, y)
    if u3 is not None and len(u3[1]) >= 16:
        f, A = amplitude_spectrum(u3[1], 1 / u3[2], "Hann", "Linear")
        peaks = find_peaks(f, A, 1)
        if peaks:
            add("Dominant frequency (FFT)", peaks[0][0], "Hz")
            add("Dominant amplitude", peaks[0][1], u)

    if threshold is not None and math.isfinite(threshold):
        above = y >= threshold
        w = np.r_[np.diff(t), 0.0]
        t_above = float(np.sum(w[above]))
        add(f"Time ≥ {threshold:g}", t_above, "s")
        if dur > 0:
            add(f"Fraction ≥ {threshold:g}", 100 * t_above / dur, "%")
        add(f"Up-crossings of {threshold:g}", int(np.count_nonzero(~above[:-1] & above[1:])))
        if above.any():
            add(f"First time ≥ {threshold:g}", t[np.argmax(above)], "s")
            add(f"Last time ≥ {threshold:g}", t[len(above) - 1 - np.argmax(above[::-1])], "s")
    return rows


# ----------------------------------------------------------------------------------------------
# Spectra
# ----------------------------------------------------------------------------------------------
def amplitude_spectrum(y, fs, window="Hann", detrend_mode="Mean"):
    """Single-sided amplitude spectrum: a sine of amplitude A shows a peak of height ~A."""
    y = detrend(y, detrend_mode)
    w = WINDOWS[window](len(y))
    Y = np.fft.rfft(y * w)
    A = 2 * np.abs(Y) / w.sum()
    A[0] /= 2
    if len(y) % 2 == 0:
        A[-1] /= 2
    return np.fft.rfftfreq(len(y), 1 / fs), A


def welch_psd(y, fs, nperseg=1024, window="Hann", detrend_mode="Mean", overlap=0.5):
    """Welch power spectral density (unit²/Hz) -> (f, P, number of averaged segments)."""
    y = np.asarray(y, float)
    nperseg = int(min(nperseg, len(y)))
    step = max(1, int(nperseg * (1 - overlap)))
    frames = _detrend_rows(_frames(y, nperseg, step, max_frames=4000).astype(float), detrend_mode)
    w = WINDOWS[window](nperseg)
    P = np.mean(np.abs(np.fft.rfft(frames * w, axis=1)) ** 2, axis=0) / (fs * np.sum(w ** 2))
    if nperseg % 2 == 0:
        P[1:-1] *= 2
    else:
        P[1:] *= 2
    return np.fft.rfftfreq(nperseg, 1 / fs), P, len(frames)


def spectrogram(y, fs, nperseg=256, overlap=0.75, window="Hann", max_frames=1500):
    """Short-time power spectrum in dB -> (frame center offsets in s, f, S[frame, freq])."""
    y = detrend(np.asarray(y, float), "Mean")
    nperseg = int(min(nperseg, len(y)))
    step = max(1, int(nperseg * (1 - overlap)))
    raw = np.lib.stride_tricks.sliding_window_view(y, nperseg)
    idx = np.arange(0, len(raw), step)
    if len(idx) > max_frames:
        idx = idx[:: int(math.ceil(len(idx) / max_frames))]
    frames = _detrend_rows(raw[idx].astype(float), "Mean")
    w = WINDOWS[window](nperseg)
    P = np.abs(np.fft.rfft(frames * w, axis=1)) ** 2 / (fs * np.sum(w ** 2))
    S = 10 * np.log10(P + 1e-20)
    centers = (idx + nperseg / 2) / fs
    return centers, np.fft.rfftfreq(nperseg, 1 / fs), S


def find_peaks(f, A, n=8, min_freq=0.0):
    """Largest local maxima (excluding DC), at least 3 bins apart -> [(freq, value)]."""
    f = np.asarray(f)
    A = np.asarray(A)
    if len(A) < 3:
        return []
    is_peak = np.zeros(len(A), bool)
    is_peak[1:-1] = (A[1:-1] > A[:-2]) & (A[1:-1] >= A[2:])
    is_peak &= (f > max(min_freq, 0.0)) & np.isfinite(A)
    cand = np.flatnonzero(is_peak)
    cand = cand[np.argsort(A[cand])[::-1]]
    chosen = []
    for i in cand:
        if all(abs(i - j) > 3 for j in chosen):
            chosen.append(i)
        if len(chosen) >= n:
            break
    return [(float(f[i]), float(A[i])) for i in chosen]


# ----------------------------------------------------------------------------------------------
# Filters (zero-phase, FFT based, smooth cosine transition to limit ringing)
# ----------------------------------------------------------------------------------------------
def _taper(f, fc, width):
    """1 below fc - width/2, 0 above fc + width/2, raised-cosine in between."""
    x = np.clip((f - (fc - width / 2)) / width, 0.0, 1.0)
    return 0.5 * (1 + np.cos(np.pi * x))


def fft_filter(t, y, kind, f1, f2=None):
    """kind: 'lowpass' | 'highpass' | 'bandpass' | 'bandstop'. Returns values at the original t."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    out = np.full_like(y, np.nan)
    ok = np.isfinite(t) & np.isfinite(y)
    u = uniform(t[ok], y[ok])
    if u is None:
        return out
    tt, yy, dt = u
    n = len(yy)
    pad = n - 1   # mirror the signal at both ends so the filter doesn't see a jump at the edges
    ext = np.concatenate([yy[1:pad + 1][::-1], yy, yy[-pad - 1:-1][::-1]])
    F = np.fft.rfft(ext)
    f = np.fft.rfftfreq(len(ext), dt)
    df = f[1] if len(f) > 1 else 1.0

    def lp(fc):
        return _taper(f, fc, max(0.1 * fc, 2 * df))

    if kind == "lowpass":
        g = lp(f1)
    elif kind == "highpass":
        g = 1 - lp(f1)
    elif kind in ("bandpass", "bandstop"):
        lo, hi = sorted((f1, f2 if f2 is not None else f1))
        g = (1 - lp(lo)) * lp(hi)
        if kind == "bandstop":
            g = 1 - g
    else:
        raise ValueError(f"unknown filter '{kind}'")
    filt = np.fft.irfft(F * g, len(ext))[pad:pad + n]
    out[ok] = np.interp(t[ok], tt, filt)
    return out


# ----------------------------------------------------------------------------------------------
# Correlation
# ----------------------------------------------------------------------------------------------
def common_grid(t1, y1, t2, y2):
    """Both signals on one even grid over their overlap (at the coarser sample interval)."""
    t1, y1 = finite_xy(t1, y1)
    t2, y2 = finite_xy(t2, y2)
    if len(t1) < 4 or len(t2) < 4:
        return None
    a, b = max(t1[0], t2[0]), min(t1[-1], t2[-1])
    if b <= a:
        return None
    d1, d2 = np.diff(t1), np.diff(t2)
    dt = max(np.median(d1[d1 > 0]), np.median(d2[d2 > 0]))
    n = int((b - a) / dt) + 1
    if n < 4:
        return None
    n = min(n, 2_000_000)
    tt = a + np.arange(n) * dt
    return tt, np.interp(tt, t1, y1), np.interp(tt, t2, y2), dt


def pearson(a, b):
    a = np.asarray(a, float) - np.mean(a)
    b = np.asarray(b, float) - np.mean(b)
    den = math.sqrt(np.sum(a * a) * np.sum(b * b))
    return float(np.sum(a * b) / den) if den > 0 else math.nan


def xcorr(a, b, max_lag):
    """Normalized cross-correlation r[k] for k = -max_lag..max_lag (samples).
    A peak at k > 0 means `a` lags (is a delayed copy of) `b` by k samples."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    n = len(a)
    a = (a - a.mean()) / (a.std() or 1.0)
    b = (b - b.mean()) / (b.std() or 1.0)
    N = 1 << int(math.ceil(math.log2(2 * n)))
    c = np.fft.irfft(np.fft.rfft(a, N) * np.conj(np.fft.rfft(b, N)), N) / n
    max_lag = int(min(max_lag, n - 1))
    lags = np.arange(-max_lag, max_lag + 1)
    return lags, c[lags % N]


# ----------------------------------------------------------------------------------------------
# Rolling statistics (time-based window, centered)
# ----------------------------------------------------------------------------------------------
def rolling(t, y, window_s, stat="mean"):
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    out = np.full_like(y, np.nan)
    ok = np.isfinite(t)
    if ok.sum() < 2 or window_s <= 0:
        return out
    tt = t[ok]
    s = pd.Series(y[ok], index=pd.to_timedelta(tt - tt[0], unit="s"))
    win = pd.Timedelta(seconds=float(window_s))
    if stat == "rms":
        r = np.sqrt((s ** 2).rolling(win, center=True, min_periods=1).mean())
    else:
        r = getattr(s.rolling(win, center=True, min_periods=1), stat)()
    out[ok] = r.to_numpy()
    return out


# ----------------------------------------------------------------------------------------------
# Histogram
# ----------------------------------------------------------------------------------------------
def histogram(y, bins="auto", density=False):
    y = np.asarray(y, float)
    y = y[np.isfinite(y)]
    if not len(y):
        return np.array([0.0, 1.0]), np.array([0.0])
    counts, edges = np.histogram(y, bins=bins, density=density)
    return edges, counts
