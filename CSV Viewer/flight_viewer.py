#!/usr/bin/env python3
"""
AFS Flight Data Viewer
======================

Interactive viewer for CSV flight logs (flights/flight_NNN.csv sensor logs, flights/nav_NNN.csv
navigation logs, or any CSV with a time column).

    python flight_viewer.py [file.csv ...] [--layout layout.json]

Highlights
  * Stacked graphs that share one time axis; drag columns from the browser onto a graph
  * Mouse wheel zooms time, Shift+wheel zooms Y, drag pans, double-click fits everything
  * Zoom-select (Z) and Crop-select (C) modes: drag across a graph to zoom to / crop to that span
  * Overview strip with a draggable window, view back/forward history
  * Live cursor readout, A/B measurement cursors with deltas, min/max/mean/std over the range
  * Flight-state shading and event jumps (launch, burnout, apogee, landing)
  * Derived signals from expressions (magnitudes, derivatives, smoothing, low-pass ...)
  * XY plots (e.g. east vs north trajectory), flight summary, PNG / CSV export, saved layouts
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

Qt = QtCore.Qt
APP_NAME = "AFS Flight Data Viewer"
SERIES_MIME = "application/x-afs-series"

# ----------------------------------------------------------------------------------------------
# Theme: validated categorical palette (fixed order) + chart surfaces, light and dark steps
# ----------------------------------------------------------------------------------------------
THEMES = {
    "light": dict(
        bg="#fcfcfb", fg="#0b0b0b", fg2="#52514e", border="#d9d8d3", active="#2a78d6",
        series=["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"],
    ),
    "dark": dict(
        bg="#1a1a19", fg="#f0efec", fg2="#c3c2b7", border="#3a3936", active="#3987e5",
        series=["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"],
    ),
}

# Firmware FlightState enum (flight_state.h)
STATE_NAMES = {0: "Launchpad", 1: "Powered ascent", 2: "Cruising", 3: "Descent", 4: "Landed"}
STATE_SHADE_SLOT = {1: 1, 2: 0, 3: 6}   # series slot used to tint each in-flight state
STATE_COLUMNS = ("flight_state", "state")

TIME_CANDIDATES = ["t_us", "time_us", "timestamp_us", "t_ms", "time_ms", "timestamp_ms", "t_s",
                   "time_s", "time", "timestamp", "t", "micros", "millis", "seconds", "sec"]
TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}

UNIT_SUFFIXES = [("_mps2", "m/s²"), ("_mps", "m/s"), ("_dps", "deg/s"), ("_deg", "deg"),
                 ("_mbar", "mbar"), ("_pa", "Pa"), ("_pct", "%"), ("_ma", "mA"), ("_mw", "mW"),
                 ("_us", "µs"), ("_ms", "ms"), ("_g", "g"), ("_G", "gauss"), ("_c", "°C"),
                 ("_m", "m"), ("_v", "V"), ("_s", "s")]

# Graph presets: (title, candidate columns). Each preset holds a single unit (one y-axis).
PRESETS = [
    ("Altitude", ["alt_m", "baro_alt_m"]),
    ("Velocity", ["vn_mps", "ve_mps", "vd_mps", "speed_mps"]),
    ("Acceleration (IMU)", ["imu_ax_g", "imu_ay_g", "imu_az_g"]),
    ("Acceleration (high-g)", ["kx_ax_g", "kx_ay_g", "kx_az_g"]),
    ("Acceleration (body)", ["ax_mps2", "ay_mps2", "az_mps2"]),
    ("Angular rate", ["imu_gx_dps", "imu_gy_dps", "imu_gz_dps", "p_dps", "q_dps", "r_dps", "roll_rate_dps"]),
    ("Attitude", ["roll_deg", "pitch_deg", "yaw_deg", "tilt_deg"]),
    ("Control tab", ["tab_deg"]),
    ("Mach", ["mach"]),
    ("Pressure", ["baro_mbar", "pressure_mbar"]),
    ("Dynamic pressure", ["qbar_pa"]),
    ("Magnetometer", ["mag_x_G", "mag_y_G", "mag_z_G"]),
    ("Temperature", ["imu_temp_c", "baro_temp_c", "tmp117_c"]),
    ("Bus voltage", ["bus_v"]),
    ("Current", ["current_ma"]),
    ("Attitude uncertainty", ["tilt_std_deg", "heading_std_deg"]),
    ("Position uncertainty", ["alt_std_m", "horiz_std_m"]),
    ("Flight state", ["flight_state"]),
    ("High-g accel (nav)", ["high_g_accel_g"]),
    ("EKF load", ["ekf_load_pct"]),
]
AUTO_LAYOUT_MAX = 5

# How flights are lined up in time when comparing
ALIGN_MODES = [("start", "File start"), ("launch", "Launch"), ("burnout", "Burnout"), ("apogee", "Apogee"),
               ("landing", "Landing"), ("peak_accel", "Peak acceleration"), ("manual", "Manual offsets")]

# Accelerometer column sets, with the factor that converts them to g
ACCEL_SETS = [(("imu_ax_g", "imu_ay_g", "imu_az_g"), 1.0), (("kx_ax_g", "kx_ay_g", "kx_az_g"), 1.0),
              (("ax_mps2", "ay_mps2", "az_mps2"), 1 / 9.80665)]
LINE_STYLES = [Qt.PenStyle.SolidLine, Qt.PenStyle.DashLine, Qt.PenStyle.DotLine, Qt.PenStyle.DashDotLine]


def format_name(signature) -> str:
    cols = set(signature)
    if "flight_state" in cols and "alt_m" in cols:
        return "nav"
    if "imu_ax_g" in cols and "baro_mbar" in cols:
        return "sensor"
    return f"{len(cols)} cols"


def unit_of(col: str) -> str:
    for suffix, unit in UNIT_SUFFIXES:
        if col.endswith(suffix):
            return unit
    return ""


def fmt(v, digits=6) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    return f"{v:.{digits}g}"


def fmt_t(t) -> str:
    return "—" if t is None or not math.isfinite(t) else f"{t:.4f} s"


# ----------------------------------------------------------------------------------------------
# Data
# ----------------------------------------------------------------------------------------------
class Dataset:
    _next_id = 0

    def __init__(self, path: str):
        self.id = Dataset._next_id
        Dataset._next_id += 1
        self.path = os.path.abspath(path)
        self.name = os.path.basename(path)
        self.short = os.path.splitext(self.name)[0]
        self.label = self.short          # display name, editable in the Flights list
        self.slot = 0                    # palette slot (color) when comparing flights
        self.color = None                # user color override
        self.visible = True
        self.warnings: list[str] = []

        df = pd.read_csv(path, on_bad_lines="skip", skipinitialspace=True, low_memory=False)
        df.columns = [str(c).strip() for c in df.columns]
        for c in df.columns:
            if not pd.api.types.is_numeric_dtype(df[c]):
                df[c] = pd.to_numeric(df[c], errors="coerce")
        df = df.dropna(axis=1, how="all")
        if df.empty:
            raise ValueError("no numeric data found")

        self.time_col = self.detect_time_column(list(df.columns))
        if self.time_col:
            df = df.dropna(subset=[self.time_col])
            if not df[self.time_col].is_monotonic_increasing:
                df = df.sort_values(self.time_col, kind="stable")
                self.warnings.append(f"{self.name}: time was not monotonic, rows were sorted")
        self.df = df.reset_index(drop=True)
        self.base_columns = list(self.df.columns)
        self.signature = tuple(self.base_columns)   # files with equal signatures can be compared
        self.derived: dict[str, str] = {}
        self.time_unit = "auto"
        self.zero_start = True
        self.offset = 0.0
        self._cache: dict[str, np.ndarray] = {}
        self.time = np.zeros(0)
        self.compute_time()

    @staticmethod
    def detect_time_column(cols):
        lower = {c.lower(): c for c in cols}
        for cand in TIME_CANDIDATES:
            if cand in lower:
                return lower[cand]
        for c in cols:
            if "time" in c.lower():
                return c
        return None

    @property
    def n(self):
        return len(self.df)

    def resolved_unit(self) -> str:
        if self.time_unit != "auto":
            return self.time_unit
        name = (self.time_col or "").lower()
        if name.endswith("us") or "micro" in name:
            return "us"
        if name.endswith("ns"):
            return "ns"
        if name.endswith("ms") or "milli" in name:
            return "ms"
        return "s"

    def array(self, col: str) -> np.ndarray:
        a = self._cache.get(col)
        if a is None:
            a = self.df[col].to_numpy(dtype=float, na_value=np.nan)
            self._cache[col] = a
        return a

    def compute_time(self):
        if self.time_col is None or self.time_col not in self.df:
            raw = np.arange(self.n, dtype=float)
        else:
            raw = self.array(self.time_col) * TIME_UNITS[self.resolved_unit()]
        t = raw.copy()
        if self.zero_start and len(t):
            finite = t[np.isfinite(t)]
            if len(finite):
                t -= finite[0]
        self.time = t + self.offset
        # derived signals may depend on t (ddt, integ, lowpass ...)
        for name, expr in list(self.derived.items()):
            try:
                self.set_derived(name, expr)
            except Exception as exc:  # keep the stale column rather than failing
                self.warnings.append(f"{name}: {exc}")

    # ---- derived signals ---------------------------------------------------------------
    def namespace(self):
        t = self.time

        def ddt(x):
            with np.errstate(divide="ignore", invalid="ignore"):
                return np.gradient(np.asarray(x, float), t)

        def integ(x):
            x = np.nan_to_num(np.asarray(x, float))
            out = np.zeros_like(x)
            out[1:] = np.cumsum(0.5 * (x[1:] + x[:-1]) * np.diff(t))
            return out

        def smooth(x, n=5):
            return pd.Series(np.asarray(x, float)).rolling(int(n), center=True, min_periods=1).mean().to_numpy()

        def median(x, n=5):
            return pd.Series(np.asarray(x, float)).rolling(int(n), center=True, min_periods=1).median().to_numpy()

        def lowpass(x, fc):
            """First-order low-pass, cutoff fc in Hz, handles uneven sample spacing."""
            x = np.asarray(x, float)
            out = np.empty_like(x)
            rc = 1.0 / (2 * math.pi * float(fc))
            dt = np.diff(t, prepend=t[0] if len(t) else 0.0)
            alpha = dt / (rc + dt)
            acc = np.nan
            for i in range(len(x)):
                xi = x[i]
                if not math.isfinite(acc):
                    acc = xi
                elif math.isfinite(xi):
                    acc += alpha[i] * (xi - acc)
                out[i] = acc
            return out

        def mag(*xs):
            return np.sqrt(sum(np.asarray(v, float) ** 2 for v in xs))

        def col(name):
            return self.array(name)

        ns = {k: getattr(np, k) for k in (
            "sqrt", "abs", "sin", "cos", "tan", "arcsin", "arccos", "arctan", "arctan2", "exp", "log",
            "log10", "degrees", "radians", "hypot", "minimum", "maximum", "where", "clip", "pi",
            "sign", "floor", "ceil", "cumsum", "unwrap", "isfinite", "nan")}
        ns.update(np=np, t=t, ddt=ddt, integ=integ, smooth=smooth, median=median, lowpass=lowpass,
                  mag=mag, col=col, g0=9.80665)
        for c in self.df.columns:
            if c.isidentifier():
                ns[c] = self.array(c)
        return ns

    def set_derived(self, name: str, expr: str):
        res = eval(expr, {"__builtins__": {}}, self.namespace())  # local analysis tool
        res = np.broadcast_to(np.asarray(res, dtype=float), (self.n,)).copy()
        self.df[name] = res
        self._cache.pop(name, None)
        self.derived[name] = expr

    def remove_derived(self, name):
        self.derived.pop(name, None)
        self._cache.pop(name, None)
        if name in self.df:
            del self.df[name]

    def state_column(self):
        for c in STATE_COLUMNS:
            if c in self.df:
                return c
        return None

    def segments(self, col):
        """Contiguous runs of a discrete column: [(t0, t1, value)]."""
        v = self.array(col)
        t = self.time
        ok = np.isfinite(v) & np.isfinite(t)
        v, t = v[ok], t[ok]
        if not len(v):
            return []
        cuts = np.flatnonzero(np.diff(v) != 0) + 1
        starts = np.r_[0, cuts]
        ends = np.r_[cuts, len(v)]
        return [(t[s], t[e] if e < len(t) else t[-1], int(v[s])) for s, e in zip(starts, ends)]

    def index_range(self, a, b):
        return np.searchsorted(self.time, a, "left"), np.searchsorted(self.time, b, "right")

    # ---- flight events & metrics (for comparing flights) -------------------------------
    def accel_g(self):
        """|acceleration| in g from the first accelerometer column set present, or None."""
        for cols, k in ACCEL_SETS:
            present = [c for c in cols if c in self.df]
            if present:
                return np.sqrt(sum(self.array(c) ** 2 for c in present)) * k
        return None

    def alt_col(self):
        for c in ("alt_m", "baro_alt_m"):
            if c in self.df and np.isfinite(self.array(c)).any():
                return c
        return None

    def _first_second(self):
        t = self.time
        fin = t[np.isfinite(t)]
        return max(1, int(np.searchsorted(t, fin[0] + 1.0))) if len(fin) else 1

    def event_time(self, kind):
        """Time (current time base) of 'start', 'launch', 'burnout', 'apogee', 'landing', 'peak_accel'."""
        t = self.time
        fin = np.isfinite(t)
        if not fin.any():
            return None
        if kind == "start":
            return float(t[fin][0])
        st = self.state_column()
        if st and kind in ("launch", "burnout", "landing"):
            for t0, _t1, s in self.segments(st):
                if (kind == "launch" and 1 <= s <= 3) or (kind == "burnout" and s == 2) or \
                        (kind == "landing" and s == 4):
                    return float(t0)
        if kind == "apogee":
            c = self.alt_col()
            return float(t[int(np.nanargmax(self.array(c)))]) if c else None
        if kind in ("launch", "peak_accel"):
            a = self.accel_g()
            if a is not None and np.isfinite(a).any():
                if kind == "peak_accel":
                    return float(t[int(np.nanargmax(a))])
                base = np.nanmedian(a[: self._first_second()])
                idx = np.flatnonzero(a > base + 1.5)          # 1.5 g above the resting reading
                if len(idx):
                    return float(t[idx[0]])
            if kind == "launch":
                c = self.alt_col()
                if c:
                    h = self.array(c)
                    idx = np.flatnonzero(h > np.nanmedian(h[: self._first_second()]) + 10.0)
                    if len(idx):
                        return float(t[idx[0]])
        return None

    def metrics(self):
        """Ordered {name: (value, unit)} of headline numbers for the Compare flights table."""
        m = {}
        t = self.time
        fin = t[np.isfinite(t)]
        if len(fin):
            m["Duration"] = (fin[-1] - fin[0], "s")
        dt = np.diff(fin)
        dt = dt[dt > 0]
        if len(dt):
            m["Sample rate"] = (1 / np.median(dt), "Hz")
        launch, burnout = self.event_time("launch"), self.event_time("burnout")
        apogee, landing = self.event_time("apogee"), self.event_time("landing")
        if launch is not None:
            m["Launch at"] = (launch, "s")
            if burnout is not None:
                m["Burn time"] = (burnout - launch, "s")
            if apogee is not None:
                m["Time to apogee"] = (apogee - launch, "s")
            if landing is not None:
                m["Flight time"] = (landing - launch, "s")

        def peak(col, absval=False):
            if col in self.df:
                a = self.array(col)
                a = np.abs(a) if absval else a
                if np.isfinite(a).any():
                    return float(np.nanmax(a))
            return None

        for col, name, unit, absval in (
                ("alt_m", "Apogee (EKF)", "m", False), ("baro_alt_m", "Apogee (baro)", "m", False),
                ("speed_mps", "Max speed", "m/s", False), ("mach", "Max Mach", "", False),
                ("qbar_pa", "Max dynamic pressure", "Pa", False), ("vd_mps", "Max descent rate", "m/s", False),
                ("tilt_deg", "Max tilt", "deg", False), ("roll_rate_dps", "Max |roll rate|", "deg/s", True),
                ("tab_deg", "Max |tab|", "deg", True), ("dropped", "Dropped rows", "", False)):
            v = peak(col, absval)
            if v is not None:
                m[name] = (v, unit)
        a = self.accel_g()
        if a is not None and np.isfinite(a).any():
            m["Peak acceleration"] = (float(np.nanmax(a)), "g")
        if "bus_v" in self.df and np.isfinite(self.array("bus_v")).any():
            m["Min bus voltage"] = (float(np.nanmin(self.array("bus_v"))), "V")
        return m


# ----------------------------------------------------------------------------------------------
# Graph widgets
# ----------------------------------------------------------------------------------------------
class FlightViewBox(pg.ViewBox):
    """Wheel zooms time only (Shift+wheel zooms Y); left-drag selects a span in zoom/crop mode."""

    def __init__(self, main):
        super().__init__()
        self.main = main
        self.panel = None
        self._sel = None

    def wheelEvent(self, ev, axis=None):
        if axis is None:
            shift = bool(ev.modifiers() & Qt.KeyboardModifier.ShiftModifier)
            axis = 1 if shift else 0
        super().wheelEvent(ev, axis=axis)

    def mouseClickEvent(self, ev):
        if self.panel is not None:
            self.main.set_active_panel(self.panel)
        if ev.button() == Qt.MouseButton.LeftButton and ev.double():
            ev.accept()
            self.main.fit_all()
            return
        super().mouseClickEvent(ev)

    def mouseDragEvent(self, ev, axis=None):
        mode = self.main.drag_mode
        if axis is None and mode in ("zoom", "crop") and ev.button() == Qt.MouseButton.LeftButton:
            ev.accept()
            x0 = self.mapSceneToView(ev.buttonDownScenePos()).x()
            x1 = self.mapSceneToView(ev.scenePos()).x()
            if self._sel is None:
                th = self.main.theme
                color = QtGui.QColor(th["series"][1] if mode == "crop" else th["series"][0])
                color.setAlpha(70)
                self._sel = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(color))
                self.addItem(self._sel, ignoreBounds=True)
            self._sel.setRegion((x0, x1))
            if ev.isFinish():
                self.removeItem(self._sel)
                self._sel = None
                self.main.on_span_selected(mode, x0, x1, self)
            return
        super().mouseDragEvent(ev, axis)


class OverviewViewBox(pg.ViewBox):
    def __init__(self, main):
        super().__init__(enableMouse=False)
        self.main = main

    def mouseClickEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            ev.accept()
            self.main.center_view(self.mapSceneToView(ev.scenePos()).x())

    def wheelEvent(self, ev, axis=None):
        ev.ignore()


def decode_series(md: QtCore.QMimeData):
    try:
        return [tuple(k) for k in json.loads(bytes(md.data(SERIES_MIME)).decode())]
    except Exception:
        return []


class DropPlotWidget(pg.PlotWidget):
    def __init__(self, on_series_drop, on_files_drop, on_wheel=None, **kw):
        super().__init__(**kw)
        self.on_series_drop = on_series_drop
        self.on_files_drop = on_files_drop
        self.on_wheel = on_wheel   # returns True when it consumed the event (scrolled the graph list)
        self.setAcceptDrops(True)

    def wheelEvent(self, ev):
        if self.on_wheel is not None and self.on_wheel(ev):
            ev.accept()
            return
        super().wheelEvent(ev)

    def _ok(self, md):
        return md.hasFormat(SERIES_MIME) or md.hasUrls()

    def dragEnterEvent(self, ev):
        if self._ok(ev.mimeData()):
            ev.acceptProposedAction()
        else:
            super().dragEnterEvent(ev)

    def dragMoveEvent(self, ev):
        if self._ok(ev.mimeData()):
            ev.acceptProposedAction()
        else:
            super().dragMoveEvent(ev)

    def dropEvent(self, ev):
        md = ev.mimeData()
        if md.hasFormat(SERIES_MIME):
            self.on_series_drop(decode_series(md))
            ev.acceptProposedAction()
        elif md.hasUrls():
            self.on_files_drop([u.toLocalFile() for u in md.urls()])
            ev.acceptProposedAction()
        else:
            super().dropEvent(ev)


DEFAULT_PANEL_H = 260
MIN_PANEL_H = 130
MAX_PANEL_H = 3000


class HeightGrip(QtWidgets.QWidget):
    """Strip along a graph's bottom edge: drag to change that graph's height."""

    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel
        self.setFixedHeight(9)
        self.setCursor(Qt.CursorShape.SizeVerCursor)
        self.setToolTip("Drag to resize this graph · double-click for the default height")
        self._start = None

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            self._start = (ev.globalPosition().y(), self.panel.height())
            ev.accept()

    def mouseMoveEvent(self, ev):
        if self._start is not None:
            y0, h0 = self._start
            self.panel.set_height(h0 + ev.globalPosition().y() - y0)
            ev.accept()

    def mouseReleaseEvent(self, ev):
        self._start = None

    def mouseDoubleClickEvent(self, ev):
        self.panel.set_height(DEFAULT_PANEL_H)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(self.panel.main.theme["fg2"]))
        cx, cy = self.width() / 2, self.height() / 2
        for dx in (-12, -6, 0, 6, 12):
            p.drawEllipse(QtCore.QPointF(cx + dx, cy), 1.3, 1.3)


class PlotPanel(QtWidgets.QFrame):
    """One graph: header bar + plot + resize grip. All panels share the time axis."""

    def __init__(self, main, title=""):
        super().__init__()
        self.main = main
        self.series: list[dict] = []   # {key, item, slot, color(optional override)}
        self.title = title
        self.setObjectName("plotPanel")
        self.setFixedHeight(DEFAULT_PANEL_H)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(0)

        hdr = QtWidgets.QHBoxLayout()
        hdr.setContentsMargins(6, 0, 2, 0)
        self.title_lbl = QtWidgets.QLabel()
        self.title_lbl.setStyleSheet("font-weight:600;")
        hdr.addWidget(self.title_lbl)
        hdr.addStretch(1)

        def tb(text, tip, slot=None, checkable=False):
            b = QtWidgets.QToolButton()
            b.setText(text)
            b.setToolTip(tip)
            b.setAutoRaise(True)
            b.setCheckable(checkable)
            if slot:
                (b.toggled if checkable else b.clicked).connect(slot)
            hdr.addWidget(b)
            return b

        self.series_btn = tb("Series ▾", "Show, hide, recolor or remove series in this graph")
        self.series_menu = QtWidgets.QMenu(self)
        self.series_menu.aboutToShow.connect(self._build_series_menu)
        self.series_btn.setMenu(self.series_menu)
        self.series_btn.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
        tb("Fit Y", "Autoscale Y to the visible data", self.fit_y)
        tb("−", "Make this graph shorter", lambda: self.set_height(self.height() - 60))
        tb("+", "Make this graph taller", lambda: self.set_height(self.height() + 60))
        self.log_btn = tb("Log Y", "Logarithmic Y axis", self._set_log, checkable=True)
        tb("▲", "Move graph up", lambda: main.move_panel(self, -1))
        tb("▼", "Move graph down", lambda: main.move_panel(self, +1))
        tb("✕", "Close graph", lambda: main.remove_panel(self))
        lay.addLayout(hdr)

        self.vb = FlightViewBox(main)
        self.vb.panel = self
        self.pw = DropPlotWidget(lambda keys: main.add_series(keys, self), main.open_files,
                                 on_wheel=main.plot_wheel, viewBox=self.vb)
        self.pw.setMinimumHeight(60)
        self.plot = self.pw.getPlotItem()
        self.plot.showGrid(x=True, y=True, alpha=0.12)
        self.plot.setDownsampling(auto=True, mode="peak")
        self.plot.setClipToView(True)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.legend = self.plot.addLegend(offset=(8, 6), labelTextSize="9pt")
        self.vb.setAutoVisible(y=True)
        lay.addWidget(self.pw, 1)
        self.grip = HeightGrip(self)
        lay.addWidget(self.grip)

        self.cursor_line = pg.InfiniteLine(angle=90, movable=False)
        self.cursor_line.setZValue(50)
        self.plot.addItem(self.cursor_line, ignoreBounds=True)
        self.ab_lines = {}
        for which in ("A", "B"):
            ln = pg.InfiniteLine(angle=90, movable=True, label=which,
                                 labelOpts=dict(position=0.96, fill=(0, 0, 0, 0)))
            ln.setZValue(60)
            ln.setVisible(False)
            ln.sigPositionChanged.connect(lambda l, w=which: main.on_ab_moved(w, l.value()))
            self.plot.addItem(ln, ignoreBounds=True)
            self.ab_lines[which] = ln
        self.state_items = []
        self.pw.scene().sigMouseMoved.connect(lambda pos: main.on_mouse_moved(self, pos))
        self.set_title(title)

    # ---- appearance --------------------------------------------------------------------
    def set_height(self, h):
        self.setFixedHeight(int(min(max(h, MIN_PANEL_H), MAX_PANEL_H)))

    def set_title(self, title):
        self.title = title
        self.title_lbl.setText(title or "Graph")

    def comparing(self):
        """True when this graph overlays more than one file: color by file, line style by column."""
        return self.main.compare_mode and len({s["key"][0] for s in self.series}) > 1

    def restyle(self):
        for s in self.series:
            s["item"].setPen(self.pen_for(s))
        self._rebuild_legend()

    def _rebuild_legend(self):
        """Comparing: a grid with one legend column per flight and one row per signal."""
        lg = self.legend
        lg.clear()
        order, ncol = self.series, 1
        if self.comparing():
            ids = {s["key"][0] for s in self.series}
            files = [i for i in self.main.datasets if i in ids]
            cols = list(dict.fromkeys(s["key"][1] for s in self.series))
            order = sorted(self.series, key=lambda s: (cols.index(s["key"][1]),
                                                       files.index(s["key"][0]) if s["key"][0] in files else 0))
            ncol = len(files)
        lg.setColumnCount(max(1, ncol))
        for s in order:
            lg.addItem(s["item"], self.main.series_label(s["key"]))

    def pen_for(self, s):
        th = self.main.theme
        if self.comparing():
            cols = list(dict.fromkeys(x["key"][1] for x in self.series))
            style = LINE_STYLES[cols.index(s["key"][1]) % len(LINE_STYLES)] if s["key"][1] in cols else LINE_STYLES[0]
            color = s.get("color") or self.main.file_color(self.main.datasets.get(s["key"][0]))
            return pg.mkPen(color, width=2, style=style)
        color = s.get("color") or th["series"][s["slot"] % len(th["series"])]
        style = Qt.PenStyle.SolidLine if s["slot"] < len(th["series"]) else Qt.PenStyle.DashLine
        return pg.mkPen(color, width=2, style=style)

    def apply_theme(self, active=False):
        th = self.main.theme
        self.pw.setBackground(th["bg"])
        for name in ("left", "bottom"):
            ax = self.plot.getAxis(name)
            ax.setPen(th["fg2"])
            ax.setTextPen(th["fg2"])
        self.legend.setLabelTextColor(th["fg"])
        lb = QtGui.QColor(th["bg"])
        lb.setAlpha(215)
        self.legend.setBrush(pg.mkBrush(lb))
        self.legend.setPen(pg.mkPen(th["border"]))
        for s in self.series:
            s["item"].setPen(self.pen_for(s))
        self._relabel()
        self.cursor_line.setPen(pg.mkPen(th["fg2"], width=1, style=Qt.PenStyle.DashLine))
        for ln in self.ab_lines.values():
            ln.setPen(pg.mkPen(th["fg"], width=1.5))
            ln.setHoverPen(pg.mkPen(th["active"], width=3))
            ln.label.setColor(th["fg"])
        self.title_lbl.setStyleSheet(f"font-weight:600; color:{th['fg']};")
        border = th["active"] if active else th["border"]
        self.setStyleSheet(f"#plotPanel {{ border: 2px solid {border}; border-radius: 4px; }}")

    def update_axis_label(self):
        units = {unit_of(s["key"][1]) for s in self.series}
        unit = units.pop() if len(units) == 1 else ""
        self.plot.setLabel("left", unit)
        ax = self.plot.getAxis("left")
        if self.series and all(s["key"][1] in STATE_COLUMNS for s in self.series):
            ax.setTicks([[(k, v) for k, v in STATE_NAMES.items()]])
        else:
            ax.setTicks(None)

    # ---- series ------------------------------------------------------------------------
    def has(self, key):
        return any(s["key"] == key for s in self.series)

    def add(self, key, slot=None, color=None, visible=True):
        if self.has(key):
            return
        if slot is None:
            used = {s["slot"] for s in self.series}
            slot = next(i for i in range(len(used) + 1) if i not in used)
        s = dict(key=key, slot=slot, color=color)
        s["item"] = self.plot.plot([], [], name=self.main.series_label(key), connect="finite")
        ds = self.main.datasets.get(key[0])
        s["item"].setVisible(visible and (ds is None or ds.visible))
        self.series.append(s)
        self.restyle()
        self.refresh_series(s)
        if not self.title:
            self.set_title(key[1])
        self.update_axis_label()

    def remove(self, key):
        for s in list(self.series):
            if s["key"] == key:
                self.plot.removeItem(s["item"])
                self.series.remove(s)
        self.restyle()
        self.update_axis_label()
        self.main.series_changed()

    def refresh_series(self, s):
        xy = self.main.series_xy(s["key"])
        if xy is None:
            s["item"].setData([], [])
        else:
            s["item"].setData(*xy)

    def refresh_all(self):
        for s in self.series:
            self.refresh_series(s)
        self._relabel()
        self.update_axis_label()

    def _relabel(self):
        for s in self.series:
            lbl = self.legend.getLabel(s["item"])
            if lbl is not None:
                lbl.setText(self.main.series_label(s["key"]), color=self.main.theme["fg"])

    def fit_y(self):
        self.vb.enableAutoRange(axis="y")
        self.vb.setAutoVisible(y=True)

    def _set_log(self, on):
        self.plot.setLogMode(y=on)
        self.fit_y()

    def _build_series_menu(self):
        m = self.series_menu
        m.clear()
        if not self.series:
            a = m.addAction("(empty: drag columns here)")
            a.setEnabled(False)
        for s in self.series:
            sub = m.addMenu(self.main.series_label(s["key"]))
            pm = QtGui.QPixmap(12, 12)
            pm.fill(QtGui.QColor(self.pen_for(s).color()))
            sub.setIcon(QtGui.QIcon(pm))
            vis = sub.addAction("Visible")
            vis.setCheckable(True)
            vis.setChecked(s["item"].isVisible())
            vis.toggled.connect(lambda on, s=s: (s["item"].setVisible(on), self.main.series_changed()))
            sub.addAction("Change color…", lambda s=s: self._recolor(s))
            sub.addAction("Plot alone in a new graph", lambda s=s: self.main.add_series([s["key"]], None))
            sub.addAction("Remove", lambda k=s["key"]: self.remove(k))
        m.addSeparator()
        m.addAction("Rename graph…", self._rename)
        m.addAction("Remove all series", self._clear)

    def _recolor(self, s):
        c = QtWidgets.QColorDialog.getColor(QtGui.QColor(self.pen_for(s).color()), self, "Series color")
        if c.isValid():
            s["color"] = c.name()
            s["item"].setPen(self.pen_for(s))
            self.main.series_changed()

    def _rename(self):
        text, ok = QtWidgets.QInputDialog.getText(self, "Rename graph", "Title:", text=self.title)
        if ok:
            self.set_title(text)

    def _clear(self):
        for s in list(self.series):
            self.plot.removeItem(s["item"])
        self.series.clear()
        self.update_axis_label()
        self.main.series_changed()

    # ---- flight-state shading ----------------------------------------------------------
    def set_state_shading(self, segments):
        for it in self.state_items:
            self.plot.removeItem(it)
        self.state_items = []
        th = self.main.theme
        for i, (t0, t1, st) in enumerate(segments):
            slot = STATE_SHADE_SLOT.get(st)
            if slot is not None:
                c = QtGui.QColor(th["series"][slot])
                c.setAlpha(45 if self.main.dark else 28)
                reg = pg.LinearRegionItem((t0, t1), movable=False, brush=pg.mkBrush(c),
                                          pen=pg.mkPen(None))
                reg.setZValue(-100)
                self.plot.addItem(reg, ignoreBounds=True)
                self.state_items.append(reg)
            if i > 0:
                ln = pg.InfiniteLine(t0, angle=90, movable=False,
                                     pen=pg.mkPen(th["fg2"], width=1, style=Qt.PenStyle.DotLine),
                                     label=STATE_NAMES.get(st, str(st)),
                                     labelOpts=dict(position=0.93 if i % 2 else 0.80, color=th["fg2"],
                                                    anchors=[(0, 0.5), (0, 0.5)]))
                ln.setZValue(-90)
                self.plot.addItem(ln, ignoreBounds=True)
                self.state_items.append(ln)


class ColumnTree(QtWidgets.QTreeWidget):
    def mimeTypes(self):
        return [SERIES_MIME]

    def mimeData(self, items):
        keys = [it.data(0, Qt.ItemDataRole.UserRole) for it in items]
        keys = [list(k) for k in keys if k and len(k) == 2]
        md = QtCore.QMimeData()
        md.setData(SERIES_MIME, QtCore.QByteArray(json.dumps(keys).encode()))
        return md


# ----------------------------------------------------------------------------------------------
# Dialogs
# ----------------------------------------------------------------------------------------------
DERIVED_HELP = """<b>Expression</b> uses column names directly (or <code>col("name")</code>) and numpy.
<br><code>t</code> = time (s) &nbsp; <code>g0</code> = 9.80665
<br><code>mag(a, b, c)</code> vector magnitude &nbsp; <code>ddt(x)</code> derivative &nbsp; <code>integ(x)</code> integral
<br><code>smooth(x, n)</code> moving average &nbsp; <code>median(x, n)</code> median filter
&nbsp; <code>lowpass(x, fc_hz)</code>
<br>sqrt abs sin cos tan arctan2 degrees radians exp log where clip unwrap …
<br><i>e.g.</i> <code>mag(imu_ax_g, imu_ay_g, imu_az_g)</code>, <code>-vd_mps</code>,
<code>ddt(baro_alt_m)</code>, <code>smooth(tab_deg, 25)</code>"""


class DerivedDialog(QtWidgets.QDialog):
    def __init__(self, main, ds: Dataset, name="", expr=""):
        super().__init__(main)
        self.setWindowTitle("Derived signal")
        self.main, self.ds = main, ds
        form = QtWidgets.QFormLayout(self)
        self.ds_lbl = QtWidgets.QLabel(ds.name)
        self.name_edit = QtWidgets.QLineEdit(name)
        self.name_edit.setPlaceholderText("e.g. accel_mag_g")
        self.expr_edit = QtWidgets.QLineEdit(expr)
        self.expr_edit.setMinimumWidth(460)
        cols = QtWidgets.QCompleter(list(ds.df.columns), self)
        cols.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.add_cb = QtWidgets.QCheckBox("Add to the active graph")
        self.add_cb.setChecked(True)
        helpl = QtWidgets.QLabel(DERIVED_HELP)
        helpl.setWordWrap(True)
        helpl.setTextFormat(Qt.TextFormat.RichText)
        col_combo = QtWidgets.QComboBox()
        col_combo.addItem("Insert column…")
        col_combo.addItems(list(ds.df.columns))
        col_combo.activated.connect(lambda i: i > 0 and (self.expr_edit.insert(col_combo.itemText(i)),
                                                         col_combo.setCurrentIndex(0)))
        form.addRow("Dataset", self.ds_lbl)
        form.addRow("Name", self.name_edit)
        form.addRow("Expression", self.expr_edit)
        form.addRow("", col_combo)
        form.addRow(helpl)
        form.addRow(self.add_cb)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok |
                                        QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self._accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def _accept(self):
        name = self.name_edit.text().strip()
        expr = self.expr_edit.text().strip()
        if not name or not expr:
            return
        if name in self.ds.base_columns:
            QtWidgets.QMessageBox.warning(self, "Derived signal", f"'{name}' is an original column.")
            return
        try:
            self.ds.set_derived(name, expr)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Derived signal", f"Could not evaluate:\n{exc}")
            return
        self.accept()


class XYWindow(QtWidgets.QWidget):
    """One column against another (e.g. east vs north ground track)."""

    def __init__(self, main):
        super().__init__(main, Qt.WindowType.Window)
        self.main = main
        self.setWindowTitle("XY plot")
        self.resize(760, 640)
        lay = QtWidgets.QVBoxLayout(self)
        row = QtWidgets.QHBoxLayout()
        self.ds_combo = QtWidgets.QComboBox()
        self.x_combo = QtWidgets.QComboBox()
        self.y_combo = QtWidgets.QComboBox()
        self.aspect_cb = QtWidgets.QCheckBox("Equal aspect")
        self.color_cb = QtWidgets.QCheckBox("Color by time")
        self.range_combo = QtWidgets.QComboBox()
        self.range_combo.addItems(["Crop range", "Visible range", "A–B range"])
        for w, label in ((self.ds_combo, "Data"), (self.x_combo, "X"), (self.y_combo, "Y"),
                         (self.range_combo, "Range")):
            row.addWidget(QtWidgets.QLabel(label))
            row.addWidget(w, 1)
        row.addWidget(self.aspect_cb)
        row.addWidget(self.color_cb)
        lay.addLayout(row)
        self.pw = pg.PlotWidget()
        self.pw.showGrid(x=True, y=True, alpha=0.12)
        lay.addWidget(self.pw, 1)
        self.info = QtWidgets.QLabel()
        lay.addWidget(self.info)

        if len(main.datasets) > 1:
            self.ds_combo.addItem("All visible files (overlay)", -1)
        for ds in main.datasets.values():
            self.ds_combo.addItem(ds.label, ds.id)
        self.ds_combo.currentIndexChanged.connect(self._fill_columns)
        for w in (self.x_combo, self.y_combo, self.range_combo):
            w.currentIndexChanged.connect(self.refresh)
        self.aspect_cb.toggled.connect(self.refresh)
        self.color_cb.toggled.connect(self.refresh)
        main.view_changed.connect(self.refresh)
        self._fill_columns()

    def _selected(self):
        if self.ds_combo.currentData() == -1:
            return [d for d in self.main.datasets.values() if d.visible]
        ds = self.main.datasets.get(self.ds_combo.currentData())
        return [ds] if ds is not None else []

    def _fill_columns(self):
        sel = self._selected()
        if not sel:
            return
        ds = sel[0]
        cols = list(ds.df.columns)
        for combo, prefs in ((self.x_combo, ["east_m", "ve_mps"]), (self.y_combo, ["north_m", "vn_mps"])):
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(cols)
            for p in prefs:
                if p in cols:
                    combo.setCurrentText(p)
                    break
            combo.blockSignals(False)
        if "east_m" in cols and "north_m" in cols:
            self.aspect_cb.setChecked(True)
        self.refresh()

    def refresh(self, *_):
        if not self.isVisible() and _:
            return
        sel = self._selected()
        xc, yc = self.x_combo.currentText(), self.y_combo.currentText()
        if not sel or not xc or not yc:
            return
        th = self.main.theme
        overlay = len(sel) > 1
        self.pw.setBackground(th["bg"])
        for name in ("left", "bottom"):
            self.pw.getAxis(name).setPen(th["fg2"])
            self.pw.getAxis(name).setTextPen(th["fg2"])
        a, b = self.main.range_for(self.range_combo.currentText())
        self.pw.clear()
        legend = self.pw.addLegend(offset=(8, 6)) if overlay else None
        if legend is not None:
            legend.setLabelTextColor(th["fg"])
        self.pw.setLabel("bottom", f"{xc}")
        self.pw.setLabel("left", f"{yc}")
        total = 0
        for ds in sel:
            if xc not in ds.df or yc not in ds.df:
                continue
            i0, i1 = ds.index_range(a, b)
            x, y = ds.array(xc)[i0:i1], ds.array(yc)[i0:i1]
            color = self.main.file_color(ds) if overlay else th["series"][0]
            self.pw.plot(x, y, pen=pg.mkPen(color, width=2), connect="finite",
                         name=ds.label if overlay else None)
            ok = np.isfinite(x) & np.isfinite(y)
            total += int(ok.sum())
            if self.color_cb.isChecked() and ok.any() and not overlay:
                idx = np.flatnonzero(ok)
                idx = idx[:: max(1, len(idx) // 15000)]
                cmap = pg.ColorMap([0, 1], [QtGui.QColor("#cde2fb"), QtGui.QColor("#0d366b")])
                tt = ds.time[i0:i1][idx]
                span = (tt[-1] - tt[0]) or 1.0
                brushes = cmap.mapToQColor((tt - tt[0]) / span)
                self.pw.addItem(pg.ScatterPlotItem(x[idx], y[idx], size=4, pen=None, brush=brushes))
            if ok.any():
                f, l = np.flatnonzero(ok)[[0, -1]]
                start_b = color if overlay else th["series"][2]
                end_b = color if overlay else th["series"][7]
                self.pw.addItem(pg.ScatterPlotItem([x[f]], [y[f]], symbol="o", size=10,
                                                   brush=start_b, pen=pg.mkPen(th["bg"], width=2)))
                self.pw.addItem(pg.ScatterPlotItem([x[l]], [y[l]], symbol="s", size=10,
                                                   brush=end_b, pen=pg.mkPen(th["bg"], width=2)))
        span = f"{a:.3f} → {b:.3f} s" if math.isfinite(a) and math.isfinite(b) else "whole file"
        self.info.setText(f"{total:,} points, {span}.   ● start   ■ end")
        self.pw.setAspectLocked(self.aspect_cb.isChecked())
        self.pw.autoRange()


class CompareWindow(QtWidgets.QWidget):
    """Headline numbers of every open flight side by side."""

    def __init__(self, main):
        super().__init__(main, Qt.WindowType.Window)
        self.main = main
        self.setWindowTitle("Compare flights")
        self.resize(980, 600)
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        self.info = QtWidgets.QLabel()
        top.addWidget(self.info, 1)
        self.hidden_cb = QtWidgets.QCheckBox("Include hidden files")
        self.hidden_cb.toggled.connect(self.refresh)
        top.addWidget(self.hidden_cb)
        lay.addLayout(top)
        self.table = QtWidgets.QTableWidget()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        self.table.horizontalHeader().setDefaultSectionSize(120)
        self.table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        lay.addWidget(self.table, 1)
        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        for text, fn in (("Copy", self.copy), ("Save CSV…", self.save), ("Refresh", self.refresh)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        lay.addLayout(row)
        main.files_changed.connect(self.refresh)
        self.refresh()

    def refresh(self, *_):
        dss = [d for d in self.main.datasets.values() if d.visible or self.hidden_cb.isChecked()]
        mets = [d.metrics() for d in dss]
        names, units = [], {}
        for m in mets:
            for k, (_v, u) in m.items():
                if k not in units:
                    names.append(k)
                    units[k] = u
        extra = ["Mean", "Spread (max − min)"] if len(dss) > 1 else []
        self.table.clear()
        self.table.setColumnCount(len(dss) + len(extra))
        self.table.setRowCount(len(names))
        for c, d in enumerate(dss):
            it = QtWidgets.QTableWidgetItem(d.label)
            pm = QtGui.QPixmap(12, 12)
            pm.fill(QtGui.QColor(self.main.file_color(d)))
            it.setIcon(QtGui.QIcon(pm))
            it.setToolTip(d.path)
            self.table.setHorizontalHeaderItem(c, it)
        for j, e in enumerate(extra):
            self.table.setHorizontalHeaderItem(len(dss) + j, QtWidgets.QTableWidgetItem(e))
        bold = self.table.font()
        bold.setBold(True)
        for r, name in enumerate(names):
            unit = units[name]
            self.table.setVerticalHeaderItem(r, QtWidgets.QTableWidgetItem(f"{name} [{unit}]" if unit else name))
            vals = [m.get(name, (math.nan, ""))[0] for m in mets]
            finite = [v for v in vals if math.isfinite(v)]
            vmax = max(finite) if len(finite) > 1 else None
            for c, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(fmt(v))
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                if vmax is not None and v == vmax and min(finite) != vmax:
                    it.setFont(bold)
                    it.setToolTip("Highest of the compared files")
                self.table.setItem(r, c, it)
            if extra:
                for j, v in enumerate((np.mean(finite) if finite else math.nan,
                                       (max(finite) - min(finite)) if finite else math.nan)):
                    it = QtWidgets.QTableWidgetItem(fmt(float(v)))
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    self.table.setItem(r, len(dss) + j, it)
        mode = dict(ALIGN_MODES).get(self.main.align_mode, "")
        self.info.setText(f"{len(dss)} file(s). Event times use the current alignment ({mode}). "
                          "Bold = highest value in the row.")

    def rows(self):
        hdr = ["metric"] + [self.table.horizontalHeaderItem(c).text() for c in range(self.table.columnCount())]
        out = [hdr]
        for r in range(self.table.rowCount()):
            out.append([self.table.verticalHeaderItem(r).text()] +
                       [(self.table.item(r, c).text() if self.table.item(r, c) else "")
                        for c in range(self.table.columnCount())])
        return out

    def copy(self):
        QtWidgets.QApplication.clipboard().setText("\n".join("\t".join(r) for r in self.rows()))

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save comparison", self.main._default_out("_compare.csv"),
                                                        "CSV (*.csv)")
        if path:
            import csv
            with open(path, "w", newline="", encoding="utf-8") as fh:
                csv.writer(fh).writerows(self.rows())


class TextDialog(QtWidgets.QDialog):
    def __init__(self, parent, title, text):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(620, 560)
        lay = QtWidgets.QVBoxLayout(self)
        ed = QtWidgets.QPlainTextEdit(text)
        ed.setReadOnly(True)
        ed.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        lay.addWidget(ed)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)


HELP_TEXT = """MOUSE
  Wheel ............... scroll through the graphs (PgUp / PgDn too)
  Ctrl + wheel ........ zoom time (around the pointer)
  Ctrl+Shift + wheel .. zoom Y of that graph
                        (View ▸ "Mouse wheel zooms graphs" swaps wheel and Ctrl+wheel)
  Bottom edge grip .... drag to make a graph taller/shorter; double-click resets it
                        (or the − / + buttons in the graph header)
  Left drag ........... pan (Pan mode) / select span (Zoom or Crop mode)
  Middle drag ......... pan in any mode
  Right drag .......... stretch X/Y freely;  right click: pyqtgraph menu (export a single graph)
  Double click ........ fit everything
  Overview strip ...... drag or resize the window, click to jump there
  A / B lines ......... drag to move the measurement cursors

KEYS
  Ctrl+O open   Ctrl+S save layout   Ctrl+L load layout   Ctrl+E export CSV   Ctrl+P export PNG
  P pan mode    Z zoom-select mode   C crop-select mode   Esc back to pan mode
  F / Home fit all      Y fit Y      + / - zoom in/out     Ctrl+Left / Ctrl+Right pan
  Backspace view back   Shift+Backspace view forward
  M measure cursors on/off     A / B place cursor A / B at the mouse
  Ctrl+K crop to view   Ctrl+Shift+K crop to A–B   Ctrl+Z undo crop   Ctrl+R reset crop
  S flight-state shading on/off   D derived signal   G new empty graph   T dark/light theme
  Ctrl+0 fit all graphs to the window   Ctrl+Shift+Up / Down active graph taller / shorter

COMPARING FLIGHTS
  * Open several logs of the same format (File ▸ Open folder… takes e.g. nav_*.csv).
  * With "Compare mode" on, plotting a column overlays it for every flight of that format:
    each flight keeps one color, each column gets its own line style.
  * Flights list: tick to show/hide, double-click a name to rename, double-click the offset to
    shift a flight, right-click to recolor / show only / close.
  * "Align on" puts launch, burnout, apogee, landing or peak acceleration at t = 0 for every flight.
  * Tools ▸ Compare flights (Ctrl+Shift+C): apogee, burn time, max speed, ... side by side.

TIPS
  * Drag columns from the browser onto a graph (or onto the empty area) to plot them.
    Double-click a column to add it to the active graph (blue outline).
  * Several files can be open at once; use the per-file time offset to align them.
  * Crop limits the data shown, the overview, the statistics and CSV export.
"""


# ----------------------------------------------------------------------------------------------
# Main window
# ----------------------------------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    view_changed = QtCore.Signal()
    files_changed = QtCore.Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1500, 920)
        self.setAcceptDrops(True)
        self.settings = QtCore.QSettings("AFS", "FlightViewer")
        self.datasets: dict[int, Dataset] = {}
        self.panels: list[PlotPanel] = []
        self.active_panel: PlotPanel | None = None
        self.crop = None
        self.crop_history = []
        self.drag_mode = "pan"
        self.measure_on = False
        self.t_ab = {"A": 0.0, "B": 0.0}
        self.cursor_t = None
        self.shade_states = True
        self.dark = self.settings.value("dark", False, type=bool)
        self.wheel_zooms = self.settings.value("wheel_zooms", False, type=bool)
        self.compare_mode = self.settings.value("compare_mode", True, type=bool)
        self.align_mode = "start"
        self._rebuilding_files = False
        self.view_hist, self.view_pos = [], -1
        self._syncing = False
        self._last_range = None

        self._stats = {}
        self.stats_timer = self._timer(120, self.update_stats)
        self.readout_timer = self._timer(30, self.update_readout)
        self.history_timer = self._timer(450, self._push_view)
        self.overview_timer = self._timer(0, self.refresh_overview)

        self._build_ui()
        self._build_actions()
        self.apply_theme()

    def _timer(self, ms, fn):
        t = QtCore.QTimer(self)
        t.setSingleShot(True)
        t.setInterval(ms)
        t.timeout.connect(fn)
        return t

    @property
    def theme(self):
        return THEMES["dark" if self.dark else "light"]

    # ---- UI construction --------------------------------------------------------------
    def _build_ui(self):
        # Center: graphs + overview
        center = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(center)
        cl.setContentsMargins(4, 4, 4, 4)
        cl.setSpacing(4)
        # Graphs live in a scrollable column; each graph has its own height
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.verticalScrollBar().setSingleStep(40)
        self.graph_host = QtWidgets.QWidget()
        self.graph_layout = QtWidgets.QVBoxLayout(self.graph_host)
        self.graph_layout.setContentsMargins(0, 0, 4, 0)
        self.graph_layout.setSpacing(6)
        self.empty_lbl = QtWidgets.QLabel(
            "Open a CSV (Ctrl+O) or drop files here.\n\n"
            "Then drag columns from the browser on the left onto a graph,\n"
            "or use Graphs ▸ Auto layout.")
        self.empty_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.empty_lbl.setMinimumHeight(300)
        self.graph_layout.addWidget(self.empty_lbl)   # index 0; graphs go at 1 .. n
        self.graph_layout.addStretch(1)
        self.scroll.setWidget(self.graph_host)
        cl.addWidget(self.scroll, 1)

        self.ov_vb = OverviewViewBox(self)
        self.overview = DropPlotWidget(lambda keys: self.add_series(keys, None), self.open_files,
                                       viewBox=self.ov_vb)
        self.overview.setFixedHeight(80)
        self.overview.hideButtons()
        self.overview.setMenuEnabled(False)
        self.overview.hideAxis("left")
        self.overview.getPlotItem().setDownsampling(auto=True, mode="peak")
        self.ov_curve = self.overview.plot([], [], connect="finite")
        self.ov_region = pg.LinearRegionItem()
        self.ov_region.setZValue(10)
        self.ov_region.sigRegionChanged.connect(self._overview_region_changed)
        self.overview.addItem(self.ov_region, ignoreBounds=True)
        self.ov_state_panel = None
        cl.addWidget(self.overview)
        self.setCentralWidget(center)

        # Left dock: files + column browser + time settings
        left = QtWidgets.QWidget()
        ll = QtWidgets.QVBoxLayout(left)
        ll.setContentsMargins(4, 4, 4, 4)
        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("Filter columns (e.g. 'alt', 'imu g')")
        self.filter_edit.setClearButtonEnabled(True)
        self.filter_edit.textChanged.connect(self._apply_filter)
        ll.addWidget(self.filter_edit)
        self.tree = ColumnTree()
        self.tree.setHeaderLabels(["Column", "Unit", "Range"])
        self.tree.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.tree.setDragEnabled(True)
        self.tree.setDragDropMode(QtWidgets.QAbstractItemView.DragDropMode.DragOnly)
        self.tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._tree_menu)
        self.tree.itemDoubleClicked.connect(self._tree_double_clicked)
        self.tree.currentItemChanged.connect(lambda *_: self._sync_time_controls())
        self.tree.header().setStretchLastSection(True)
        self.tree.setColumnWidth(0, 170)
        self.tree.setColumnWidth(1, 52)
        ll.addWidget(self.tree, 1)

        btns = QtWidgets.QHBoxLayout()
        for text, fn, tip in (("Add to graph", lambda: self.add_series(self.selected_keys(), self.active_panel),
                               "Add selected columns to the active graph"),
                              ("New graph", lambda: self.add_series(self.selected_keys(), None),
                               "Plot selected columns in a new graph"),
                              ("Derived…", self.new_derived, "Create a signal from an expression")):
            b = QtWidgets.QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            btns.addWidget(b)
        ll.addLayout(btns)

        grp = QtWidgets.QGroupBox("Time axis (selected file)")
        gl = QtWidgets.QFormLayout(grp)
        self.time_col_combo = QtWidgets.QComboBox()
        self.time_unit_combo = QtWidgets.QComboBox()
        self.time_unit_combo.addItems(["auto", "s", "ms", "us", "ns"])
        self.zero_cb = QtWidgets.QCheckBox("Start at 0")
        self.offset_spin = QtWidgets.QDoubleSpinBox()
        self.offset_spin.setRange(-1e7, 1e7)
        self.offset_spin.setDecimals(4)
        self.offset_spin.setSuffix(" s")
        self.offset_spin.setKeyboardTracking(False)
        self.offset_spin.setToolTip("Shift this file in time to line it up with another log")
        gl.addRow("Column", self.time_col_combo)
        gl.addRow("Unit", self.time_unit_combo)
        gl.addRow("Offset", self.offset_spin)
        gl.addRow(self.zero_cb)
        close_btn = QtWidgets.QPushButton("Close file")
        close_btn.clicked.connect(self.close_current_file)
        gl.addRow(close_btn)
        self.time_col_combo.activated.connect(self._time_controls_changed)
        self.time_unit_combo.activated.connect(self._time_controls_changed)
        self.zero_cb.toggled.connect(self._time_controls_changed)
        self.offset_spin.valueChanged.connect(self._time_controls_changed)
        ll.addWidget(grp)
        self.time_grp = grp

        dock = QtWidgets.QDockWidget("Data", self)
        dock.setObjectName("dataDock")
        dock.setWidget(left)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                         QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        self.data_dock = dock

        # Left dock (top): list of open flights, for comparing several logs
        fw = QtWidgets.QWidget()
        fl = QtWidgets.QVBoxLayout(fw)
        fl.setContentsMargins(4, 4, 4, 4)
        row = QtWidgets.QHBoxLayout()
        self.compare_cb = QtWidgets.QCheckBox("Compare mode")
        self.compare_cb.setToolTip("Plotting a column overlays it for every file with the same format;\n"
                                   "each file keeps one color, each column gets its own line style")
        self.compare_cb.setChecked(self.compare_mode)
        self.compare_cb.toggled.connect(self.set_compare_mode)
        row.addWidget(self.compare_cb)
        row.addStretch(1)
        row.addWidget(QtWidgets.QLabel("Align on"))
        self.align_combo = QtWidgets.QComboBox()
        for key, label in ALIGN_MODES:
            self.align_combo.addItem(label, key)
        self.align_combo.setToolTip("Shift every file so this event is at t = 0")
        self.align_combo.activated.connect(lambda _i: self.set_align_mode(self.align_combo.currentData()))
        row.addWidget(self.align_combo)
        fl.addLayout(row)
        self.files_table = QtWidgets.QTableWidget(0, 5)
        self.files_table.setHorizontalHeaderLabels(["", "Flight", "Format", "Length", "Offset s"])
        self.files_table.verticalHeader().setVisible(False)
        self.files_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.files_table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.files_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked |
                                         QtWidgets.QAbstractItemView.EditTrigger.EditKeyPressed)
        fh = self.files_table.horizontalHeader()
        fh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        fh.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.files_table.verticalHeader().setDefaultSectionSize(22)
        self.files_table.setToolTip("Tick to show/hide a flight · double-click the name to rename it, "
                                    "or the offset to shift it · right-click for more")
        self.files_table.itemChanged.connect(self._files_item_changed)
        self.files_table.currentCellChanged.connect(self._files_current_changed)
        self.files_table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.files_table.customContextMenuRequested.connect(self._files_menu)
        fl.addWidget(self.files_table, 1)
        brow = QtWidgets.QHBoxLayout()
        for text, fn, tip in (("Open…", self.open_dialog, "Open more CSV logs"),
                              ("Open folder…", self.open_folder_dialog, "Open every CSV in a folder"),
                              ("Compare table…", self.open_compare, "Headline numbers of every flight side by side")):
            b = QtWidgets.QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            brow.addWidget(b)
        fl.addLayout(brow)
        fdock = QtWidgets.QDockWidget("Flights", self)
        fdock.setObjectName("flightsDock")
        fdock.setWidget(fw)
        fdock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                          QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable |
                          QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetClosable)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, fdock)
        self.splitDockWidget(fdock, self.data_dock, Qt.Orientation.Vertical)
        self.resizeDocks([fdock, self.data_dock], [190, 700], Qt.Orientation.Vertical)
        self.files_dock = fdock

        # Bottom dock: readout table
        rd = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(rd)
        rl.setContentsMargins(4, 2, 4, 4)
        self.readout_lbl = QtWidgets.QLabel()
        self.readout_lbl.setTextFormat(Qt.TextFormat.RichText)
        self.readout_lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.readout_lbl.setWordWrap(True)
        rl.addWidget(self.readout_lbl)
        self.table = QtWidgets.QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(["Graph", "Series", "At cursor", "At A", "At B", "B − A",
                                              "Min", "Max", "Mean", "Std"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        # Fixed-ish column widths so a narrow window gets a horizontal scrollbar instead of squashing
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        hh.setStretchLastSection(True)
        hh.setMinimumSectionSize(60)
        for c, w in enumerate((140, 210, 105, 95, 95, 95, 95, 95, 95, 95)):
            self.table.setColumnWidth(c, w)
        self.table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.table.setMinimumHeight(60)
        self.table.verticalHeader().setDefaultSectionSize(22)
        rl.addWidget(self.table, 1)
        rdock = QtWidgets.QDockWidget("Readout & statistics", self)
        rdock.setObjectName("readoutDock")
        rdock.setWidget(rd)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, rdock)
        self.resizeDocks([rdock], [220], Qt.Orientation.Vertical)
        self.resizeDocks([dock], [370], Qt.Orientation.Horizontal)
        self.readout_dock = rdock

        self.status = self.statusBar()
        self.mode_lbl = QtWidgets.QLabel()
        self.crop_lbl = QtWidgets.QLabel()
        self.status.addPermanentWidget(self.crop_lbl)
        self.status.addPermanentWidget(self.mode_lbl)

    def _build_actions(self):
        mb = self.menuBar()
        tbar = self.addToolBar("Main")
        tbar.setObjectName("mainToolbar")
        tbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)

        def act(menu, text, fn=None, key=None, tip=None, checkable=False, toolbar=False):
            a = QtGui.QAction(text, self)
            if key:
                a.setShortcuts([QtGui.QKeySequence(k) for k in (key if isinstance(key, list) else [key])])
            if tip:
                a.setToolTip(f"{tip} ({a.shortcut().toString()})" if key else tip)
                a.setStatusTip(tip)
            a.setCheckable(checkable)
            if fn:
                (a.toggled if checkable else a.triggered).connect(fn)
            menu.addAction(a)
            if toolbar:
                tbar.addAction(a)
            self.addAction(a)
            return a

        f = mb.addMenu("&File")
        act(f, "Open CSV…", self.open_dialog, "Ctrl+O", "Open one or more CSV logs", toolbar=True)
        act(f, "Open folder…", self.open_folder_dialog, "Ctrl+Shift+O", "Open every CSV in a folder to compare them")
        self.recent_menu = f.addMenu("Open recent")
        self.recent_menu.aboutToShow.connect(self._build_recent)
        f.addSeparator()
        act(f, "Load layout…", self.load_layout_dialog, "Ctrl+L", "Restore saved files, graphs and derived signals")
        act(f, "Save layout…", self.save_layout_dialog, "Ctrl+S", "Save files, graphs and derived signals")
        f.addSeparator()
        act(f, "Export graphs as PNG…", self.export_png, "Ctrl+P", "Save an image of all graphs")
        act(f, "Export data as CSV…", self.export_csv, "Ctrl+E", "Save cropped / selected data to CSV")
        f.addSeparator()
        act(f, "Quit", self.close, "Ctrl+Q")
        tbar.addSeparator()

        v = mb.addMenu("&View")
        grp = QtGui.QActionGroup(self)
        self.mode_actions = {}
        for mode, text, key, tip in (("pan", "Pan", "P", "Drag pans the view"),
                                     ("zoom", "Zoom select", "Z", "Drag across a graph to zoom to that span"),
                                     ("crop", "Crop select", "C", "Drag across a graph to crop the data to that span")):
            a = act(v, text, lambda on, m=mode: on and self.set_mode(m), key, tip, checkable=True, toolbar=True)
            grp.addAction(a)
            self.mode_actions[mode] = a
        self.mode_actions["pan"].setChecked(True)
        act(v, "Pan mode (Esc)", lambda: self.mode_actions["pan"].setChecked(True), "Esc")
        v.addSeparator()
        tbar.addSeparator()
        act(v, "Fit all", self.fit_all, ["F", "Home"], "Show all data", toolbar=True)
        act(v, "Fit Y", self.fit_y_all, "Y", "Autoscale every graph's Y axis")
        act(v, "Zoom in", lambda: self.zoom(0.5), ["+", "=", "Ctrl+="], "Zoom in on time", toolbar=True)
        act(v, "Zoom out", lambda: self.zoom(2.0), ["-", "Ctrl+-"], "Zoom out on time", toolbar=True)
        act(v, "Pan left", lambda: self.pan(-0.25), "Ctrl+Left")
        act(v, "Pan right", lambda: self.pan(0.25), "Ctrl+Right")
        act(v, "View back", self.view_back, "Backspace", "Previous view", toolbar=True)
        act(v, "View forward", self.view_forward, "Shift+Backspace", "Next view")
        v.addSeparator()
        tbar.addSeparator()
        self.measure_act = act(v, "Measure (A/B cursors)", self.set_measure, "M",
                               "Two draggable cursors with deltas and range statistics",
                               checkable=True, toolbar=True)
        act(v, "Place cursor A at mouse", lambda: self.place_ab("A"), "A")
        act(v, "Place cursor B at mouse", lambda: self.place_ab("B"), "B")
        self.shade_act = act(v, "Flight-state shading", self.set_shading, "S",
                             "Shade powered ascent / cruising / descent", checkable=True)
        self.shade_act.setChecked(True)
        self.ov_act = act(v, "Overview strip", lambda on: self.overview.setVisible(on), None, checkable=True)
        self.ov_act.setChecked(True)
        self.dark_act = act(v, "Dark theme", self.set_dark, "T", checkable=True)
        self.dark_act.setChecked(self.dark)
        self.wheel_act = act(v, "Mouse wheel zooms graphs (Ctrl+wheel scrolls)", self.set_wheel_zooms, None,
                             "Swap what the wheel does over a graph", checkable=True)
        self.wheel_act.setChecked(self.wheel_zooms)
        v.addSeparator()
        v.addAction(self.files_dock.toggleViewAction())
        v.addAction(self.data_dock.toggleViewAction())
        v.addAction(self.readout_dock.toggleViewAction())

        d = mb.addMenu("&Data")
        tbar.addSeparator()
        act(d, "Crop to view", self.crop_to_view, "Ctrl+K", "Keep only the visible time span", toolbar=True)
        act(d, "Crop to A–B", self.crop_to_ab, "Ctrl+Shift+K", "Keep only the span between the cursors")
        act(d, "Undo crop", self.undo_crop, "Ctrl+Z", "Return to the previous crop", toolbar=True)
        act(d, "Reset crop", self.reset_crop, "Ctrl+R", "Show the full flight", toolbar=True)
        d.addSeparator()
        act(d, "New derived signal…", self.new_derived, "D", "Create a signal from an expression")
        self.events_menu = d.addMenu("Jump to event")
        self.events_menu.aboutToShow.connect(self._build_events_menu)

        g = mb.addMenu("&Graphs")
        act(g, "New empty graph", lambda: self.new_panel(""), "G")
        act(g, "Auto layout", lambda: self.auto_layout(clear=True), None,
            "Build standard graphs for the selected file")
        self.presets_menu = g.addMenu("Add preset graph")
        self.presets_menu.aboutToShow.connect(self._build_presets_menu)
        act(g, "Remove all graphs", self.clear_panels)
        g.addSeparator()
        act(g, "Fit graphs to window", self.fit_graphs_to_window, "Ctrl+0", "Size graphs so all fit without scrolling")
        act(g, "All graphs default height", self.default_heights)
        act(g, "Active graph taller", lambda: self._resize_active(+60), "Ctrl+Shift+Up")
        act(g, "Active graph shorter", lambda: self._resize_active(-60), "Ctrl+Shift+Down")
        act(g, "Scroll graphs up", lambda: self._scroll_page(-1), "PgUp")
        act(g, "Scroll graphs down", lambda: self._scroll_page(+1), "PgDown")

        tl = mb.addMenu("&Tools")
        act(tl, "Compare flights…", self.open_compare, "Ctrl+Shift+C",
            "Headline numbers of every open flight side by side", toolbar=True)
        act(tl, "XY plot…", self.open_xy, None, "Plot one column against another (e.g. ground track)",
            toolbar=False)
        act(tl, "Flight summary…", self.flight_summary, None, "Key numbers for each open file")
        act(tl, "Column statistics…", self.column_stats, None, "Statistics of every column over the range")

        h = mb.addMenu("&Help")
        act(h, "Mouse && keyboard…", lambda: TextDialog(self, "Mouse & keyboard", HELP_TEXT).exec(), "F1")
        self._update_mode_label()

    # ---- theme ------------------------------------------------------------------------
    def set_dark(self, on):
        self.dark = on
        self.settings.setValue("dark", on)
        self.apply_theme()

    def apply_theme(self):
        th = self.theme
        app = QtWidgets.QApplication.instance()
        app.setStyle("Fusion")
        pal = QtGui.QPalette()
        if self.dark:
            base, win, text = QtGui.QColor("#232322"), QtGui.QColor("#2a2a28"), QtGui.QColor(th["fg"])
            for role, c in ((QtGui.QPalette.ColorRole.Window, win), (QtGui.QPalette.ColorRole.WindowText, text),
                            (QtGui.QPalette.ColorRole.Base, base),
                            (QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor("#2f2f2d")),
                            (QtGui.QPalette.ColorRole.Text, text), (QtGui.QPalette.ColorRole.Button, win),
                            (QtGui.QPalette.ColorRole.ButtonText, text),
                            (QtGui.QPalette.ColorRole.ToolTipBase, base),
                            (QtGui.QPalette.ColorRole.ToolTipText, text),
                            (QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(th["active"])),
                            (QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor("#ffffff")),
                            (QtGui.QPalette.ColorRole.PlaceholderText, QtGui.QColor("#8a8983"))):
                pal.setColor(role, c)
            for role in (QtGui.QPalette.ColorRole.Text, QtGui.QPalette.ColorRole.ButtonText,
                         QtGui.QPalette.ColorRole.WindowText):
                pal.setColor(QtGui.QPalette.ColorGroup.Disabled, role, QtGui.QColor("#77766f"))
        else:
            pal = app.style().standardPalette()
        app.setPalette(pal)
        for p in self.panels:
            p.apply_theme(p is self.active_panel)
        self.overview.setBackground(th["bg"])
        ax = self.overview.getAxis("bottom")
        ax.setPen(th["fg2"])
        ax.setTextPen(th["fg2"])
        c = QtGui.QColor(th["active"])
        c.setAlpha(45)
        self.ov_region.setBrush(pg.mkBrush(c))
        for ln in self.ov_region.lines:
            ln.setPen(pg.mkPen(th["active"], width=2))
        self.ov_curve.setPen(pg.mkPen(th["fg2"], width=1))
        self.empty_lbl.setStyleSheet(f"color:{th['fg2']}; font-size:13pt;")
        self.refresh_state_shading()
        self.series_changed()

    # ---- files ------------------------------------------------------------------------
    def open_dialog(self):
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Open flight logs", self.settings.value("last_dir", ""),
            "CSV files (*.csv *.CSV *.txt);;All files (*)")
        if paths:
            self.open_files(paths)

    def open_files(self, paths):
        opened = []
        for p in paths:
            if not p or not os.path.isfile(p):
                continue
            ap = os.path.abspath(p)
            if any(d.path == ap for d in self.datasets.values()):
                continue
            QtWidgets.QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                ds = Dataset(ap)
            except Exception as exc:
                QtWidgets.QApplication.restoreOverrideCursor()
                QtWidgets.QMessageBox.warning(self, "Open", f"Could not read {p}:\n{exc}")
                continue
            QtWidgets.QApplication.restoreOverrideCursor()
            self._register(ds)
            opened.append(ds)
            self._add_recent(ap)
            self.settings.setValue("last_dir", os.path.dirname(ap))
            for w in ds.warnings:
                self.status.showMessage(w, 8000)
        if not opened:
            return []
        missing = []
        for ds in opened:
            self._inherit_derived(ds)
            if not self._align_dataset(ds):
                missing.append(ds.label)
        self.rebuild_tree(select=opened[-1])
        if not self.panels:
            self.auto_layout(ds=opened[0])
        else:
            if self.compare_mode:
                self._add_to_existing_graphs(opened)
            self.refresh_all()
        if missing:
            self._warn_alignment(missing)
        self.setWindowTitle(f"{APP_NAME} — {', '.join(d.name for d in self.datasets.values())}")
        n = sum(d.n for d in opened)
        self.status.showMessage(f"Loaded {', '.join(d.name for d in opened)} ({n:,} rows)", 6000)
        return opened

    # ---- flights (comparing several files) --------------------------------------------
    def _register(self, ds):
        used = {d.slot for d in self.datasets.values()}
        ds.slot = next(i for i in range(len(used) + 1) if i not in used)
        self.datasets[ds.id] = ds

    def file_color(self, ds):
        th = self.theme
        if ds is None:
            return th["fg2"]
        return ds.color or th["series"][ds.slot % len(th["series"])]

    def same_format(self, ds):
        return [d for d in self.datasets.values() if d.signature == ds.signature]

    def expand_compare(self, keys):
        """(file, column) -> that column in every visible file of the same format."""
        out = []
        for ds_id, col in keys:
            ds = self.datasets[ds_id]
            for d in self.same_format(ds):
                k = (d.id, col)
                if col in d.df and (d.id == ds_id or d.visible) and k not in out:
                    out.append(k)
        return out

    def _inherit_derived(self, ds):
        for other in self.same_format(ds):
            if other is not ds:
                for name, expr in other.derived.items():
                    if name not in ds.derived:
                        try:
                            ds.set_derived(name, expr)
                        except Exception:
                            pass

    def _add_to_existing_graphs(self, new_datasets):
        for ds in new_datasets:
            for p in self.panels:
                cols = []
                for s in p.series:
                    d = self.datasets.get(s["key"][0])
                    if d is not None and d is not ds and d.signature == ds.signature and s["key"][1] not in cols:
                        cols.append(s["key"][1])
                for c in cols:
                    if c in ds.df:
                        p.add((ds.id, c))

    def set_compare_mode(self, on):
        self.compare_mode = on
        self.settings.setValue("compare_mode", on)
        if self.compare_cb.isChecked() != on:
            self.compare_cb.setChecked(on)
        if on:   # overlay the other flights on every existing graph
            for p in self.panels:
                for k in self.expand_compare([s["key"] for s in p.series]):
                    p.add(k)
            self.refresh_all()
        for p in self.panels:
            p.restyle()
        self.series_changed()

    def set_align_mode(self, mode):
        self.align_mode = mode
        i = self.align_combo.findData(mode)
        if i >= 0 and self.align_combo.currentIndex() != i:
            self.align_combo.setCurrentIndex(i)
        if mode == "manual":
            return
        missing = [ds.label for ds in self.datasets.values() if not self._align_dataset(ds)]
        self.crop, self.crop_history = None, []      # crop was in the old time base
        self._update_mode_label()
        self.rebuild_files()
        self._sync_time_controls()
        self.refresh_all()
        self.fit_all()
        if missing:
            self._warn_alignment(missing)

    def _align_dataset(self, ds) -> bool:
        """Shift ds so the alignment event is at t = 0. False if the event wasn't found."""
        mode = self.align_mode
        if mode == "manual":
            return True
        ds.offset = 0.0
        ds.compute_time()
        if mode == "start":
            return True
        e = ds.event_time(mode)
        if e is None:
            return False
        ds.offset = -e
        ds.compute_time()
        return True

    def _warn_alignment(self, labels):
        name = dict(ALIGN_MODES).get(self.align_mode, self.align_mode)
        self.status.showMessage(f"No '{name}' event found in {', '.join(labels)}; left at file start", 10000)

    def set_file_visible(self, ds, on):
        ds.visible = on
        for p in self.panels:
            for s in p.series:
                if s["key"][0] == ds.id:
                    s["item"].setVisible(on)
        self.refresh_state_shading()
        self.series_changed()
        self.files_changed.emit()

    def show_all(self):
        for d in self.datasets.values():
            self.set_file_visible(d, True)
        self.rebuild_files()

    def show_only(self, ds):
        for d in self.datasets.values():
            self.set_file_visible(d, d is ds)
        self.rebuild_files()

    def rebuild_files(self):
        self._rebuilding_files = True
        tbl = self.files_table
        tbl.setRowCount(len(self.datasets))
        groups = {}
        for d in self.datasets.values():
            groups.setdefault(d.signature, []).append(d)
        cur = self.current_dataset()
        for r, ds in enumerate(self.datasets.values()):
            t = ds.time[np.isfinite(ds.time)]
            dur = t[-1] - t[0] if len(t) else 0.0
            n_same = len(groups[ds.signature])
            fmt_txt = format_name(ds.signature) + (f" ×{n_same}" if n_same > 1 else "")
            items = [QtWidgets.QTableWidgetItem(""), QtWidgets.QTableWidgetItem(ds.label),
                     QtWidgets.QTableWidgetItem(fmt_txt), QtWidgets.QTableWidgetItem(f"{dur:.1f} s"),
                     QtWidgets.QTableWidgetItem(f"{ds.offset:.3f}")]
            items[0].setToolTip("Show / hide this flight")
            items[3].setToolTip(f"{ds.n:,} rows")
            base = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
            items[0].setFlags(base | Qt.ItemFlag.ItemIsUserCheckable)
            items[0].setCheckState(Qt.CheckState.Checked if ds.visible else Qt.CheckState.Unchecked)
            items[1].setFlags(base | Qt.ItemFlag.ItemIsEditable)
            items[1].setData(Qt.ItemDataRole.DecorationRole, QtGui.QColor(self.file_color(ds)))
            items[1].setToolTip(f"{ds.path}\nDouble-click to rename")
            items[2].setToolTip(f"{n_same} open file(s) share this column layout and can be overlaid")
            items[4].setFlags(base | Qt.ItemFlag.ItemIsEditable)
            items[4].setToolTip("Seconds added to this file's time (double-click to edit)")
            for c, it in enumerate(items):
                if c not in (0, 1, 4):
                    it.setFlags(base)
                if c >= 3:
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                it.setData(Qt.ItemDataRole.UserRole, ds.id)
                tbl.setItem(r, c, it)
            if cur is not None and ds.id == cur.id:
                tbl.selectRow(r)
        self._rebuilding_files = False
        self.files_changed.emit()

    def _files_item_changed(self, it):
        if self._rebuilding_files:
            return
        ds = self.datasets.get(it.data(Qt.ItemDataRole.UserRole))
        if ds is None:
            return
        if it.column() == 0:
            self.set_file_visible(ds, it.checkState() == Qt.CheckState.Checked)
        elif it.column() == 1:
            ds.label = it.text().strip() or ds.short
            for p in self.panels:
                p._relabel()
            self.rebuild_files()
            self.series_changed()
        elif it.column() == 4:
            try:
                ds.offset = float(it.text())
            except ValueError:
                self.rebuild_files()
                return
            self.set_align_mode("manual")
            ds.compute_time()
            self.rebuild_files()
            self._sync_time_controls()
            self.refresh_all()

    def _files_current_changed(self, row, _col, _prow, _pcol):
        if self._rebuilding_files or row < 0:
            return
        it = self.files_table.item(row, 0)
        ds_id = it.data(Qt.ItemDataRole.UserRole) if it else None
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            if top.data(0, Qt.ItemDataRole.UserRole + 1) == ds_id:
                self.tree.setCurrentItem(top)
                self.tree.scrollToItem(top, QtWidgets.QAbstractItemView.ScrollHint.PositionAtTop)
                break

    def _files_menu(self, pos):
        it = self.files_table.itemAt(pos)
        ds = self.datasets.get(it.data(Qt.ItemDataRole.UserRole)) if it else None
        m = QtWidgets.QMenu(self)
        if ds is not None:
            m.addAction(f"Show only {ds.label}", lambda: self.show_only(ds))
            m.addAction("Change color…", lambda: self._recolor_file(ds))
            if ds.color:
                m.addAction("Reset color", lambda: (setattr(ds, "color", None), self._colors_changed()))
            m.addAction("Rename…", lambda: self.files_table.editItem(self.files_table.item(it.row(), 1)))
            m.addAction("Close file", lambda: self.close_file(ds))
            m.addSeparator()
        m.addAction("Show all", self.show_all)
        m.addAction("Compare table…", self.open_compare)
        m.addAction("Open folder…", self.open_folder_dialog)
        m.exec(self.files_table.viewport().mapToGlobal(pos))

    def _recolor_file(self, ds):
        c = QtWidgets.QColorDialog.getColor(QtGui.QColor(self.file_color(ds)), self, f"Color of {ds.label}")
        if c.isValid():
            ds.color = c.name()
            self._colors_changed()

    def _colors_changed(self):
        for p in self.panels:
            p.restyle()
        self.rebuild_files()
        self.series_changed()

    def open_folder_dialog(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Open every CSV in a folder",
                                                       self.settings.value("last_dir", ""))
        if not d:
            return
        pattern, ok = QtWidgets.QInputDialog.getText(
            self, "Open folder", "Files to open (e.g. nav_*.csv, flight_*.csv, *.csv):",
            text=self.settings.value("folder_pattern", "*.csv"))
        if not ok or not pattern.strip():
            return
        self.settings.setValue("folder_pattern", pattern.strip())
        import glob
        paths = []
        for pat in pattern.replace(";", ",").split(","):
            paths += glob.glob(os.path.join(d, pat.strip()))
        paths = sorted({p for p in paths if os.path.isfile(p)})
        if not paths:
            QtWidgets.QMessageBox.information(self, "Open folder", f"No files matching '{pattern}' in\n{d}")
            return
        self.open_files(paths)

    def open_compare(self):
        if not self.datasets:
            return
        if getattr(self, "_compare_win", None) is None:
            self._compare_win = CompareWindow(self)
        self._compare_win.refresh()
        self._compare_win.show()
        self._compare_win.raise_()

    def close_current_file(self):
        self.close_file(self.current_dataset())

    def close_file(self, ds):
        if ds is None:
            return
        for p in self.panels:
            for s in list(p.series):
                if s["key"][0] == ds.id:
                    p.remove(s["key"])
        del self.datasets[ds.id]
        self.rebuild_tree()
        self.refresh_all()

    def _add_recent(self, path):
        rec = [r for r in (self.settings.value("recent", [], type=list) or []) if r != path]
        self.settings.setValue("recent", [path] + rec[:9])

    def _build_recent(self):
        self.recent_menu.clear()
        rec = self.settings.value("recent", [], type=list) or []
        for r in rec:
            self.recent_menu.addAction(r, lambda r=r: self.open_files([r]))
        if not rec:
            self.recent_menu.addAction("(none)").setEnabled(False)

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasUrls() or ev.mimeData().hasFormat(SERIES_MIME):
            ev.acceptProposedAction()

    def dropEvent(self, ev):
        md = ev.mimeData()
        if md.hasFormat(SERIES_MIME):
            self.add_series(decode_series(md), None)
        elif md.hasUrls():
            paths = [u.toLocalFile() for u in md.urls()]
            layouts = [p for p in paths if p.lower().endswith(".json")]
            if layouts:
                self.load_layout(layouts[0])
            self.open_files([p for p in paths if not p.lower().endswith(".json")])

    # ---- column browser ---------------------------------------------------------------
    def rebuild_tree(self, select: Dataset | None = None):
        cur = select or self.current_dataset()
        self.tree.clear()
        bold = self.tree.font()
        bold.setBold(True)
        for ds in self.datasets.values():
            name = ds.name if ds.label == ds.short else f"{ds.label} ({ds.name})"
            top = QtWidgets.QTreeWidgetItem([name, "", f"{ds.n:,} rows"])
            top.setFont(0, bold)
            top.setData(0, Qt.ItemDataRole.UserRole + 1, ds.id)
            top.setToolTip(0, ds.path)
            top.setFlags(top.flags() & ~Qt.ItemFlag.ItemIsDragEnabled)
            self.tree.addTopLevelItem(top)
            for c in ds.df.columns:
                a = ds.array(c)
                fin = a[np.isfinite(a)]
                rng = f"{fmt(fin.min(), 4)} … {fmt(fin.max(), 4)}" if len(fin) else "empty"
                name = c + ("  ƒ" if c in ds.derived else "") + ("  ⏱" if c == ds.time_col else "")
                it = QtWidgets.QTreeWidgetItem([name, unit_of(c), rng])
                it.setData(0, Qt.ItemDataRole.UserRole, [ds.id, c])
                it.setData(0, Qt.ItemDataRole.UserRole + 1, ds.id)
                if c in ds.derived:
                    it.setToolTip(0, f"{c} = {ds.derived[c]}")
                top.addChild(it)
            top.setExpanded(True)
            if cur is not None and ds.id == cur.id:
                self.tree.setCurrentItem(top)
        self._apply_filter(self.filter_edit.text())
        self._sync_time_controls()
        self.rebuild_files()

    def _apply_filter(self, text):
        terms = text.lower().split()
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            for j in range(top.childCount()):
                ch = top.child(j)
                name = ch.text(0).lower() + " " + ch.text(1).lower()
                ch.setHidden(bool(terms) and not all(t in name for t in terms))

    def selected_keys(self):
        keys = []
        for it in self.tree.selectedItems():
            k = it.data(0, Qt.ItemDataRole.UserRole)
            if k:
                keys.append((int(k[0]), str(k[1])))
        return keys

    def current_dataset(self) -> Dataset | None:
        it = self.tree.currentItem()
        if it is not None:
            ds = self.datasets.get(it.data(0, Qt.ItemDataRole.UserRole + 1))
            if ds:
                return ds
        return next(iter(self.datasets.values()), None)

    def _tree_double_clicked(self, it, _col):
        k = it.data(0, Qt.ItemDataRole.UserRole)
        if k:
            self.add_series([(int(k[0]), str(k[1]))], self.active_panel)

    def _tree_menu(self, pos):
        keys = self.selected_keys()
        m = QtWidgets.QMenu(self)
        if keys:
            m.addAction("Add to active graph", lambda: self.add_series(keys, self.active_panel))
            m.addAction("Plot in new graph", lambda: self.add_series(keys, None))
            m.addAction("Each in its own graph", lambda: [self.add_series([k], None) for k in keys])
            if len(self.datasets) > 1:
                m.addAction("Compare across all flights (new graph)",
                            lambda: self.add_series(keys, None, expand=True))
                m.addAction("This file only (new graph)", lambda: self.add_series(keys, None, expand=False))
            m.addSeparator()
            ds = self.datasets[keys[0][0]]
            same = all(k[0] == ds.id for k in keys)
            if same and len(keys) >= 2:
                cols = ", ".join(k[1] for k in keys)
                m.addAction(f"Magnitude of {len(keys)} columns…",
                            lambda: self.new_derived(ds, "_".join(k[1] for k in keys[:1]) + "_mag", f"mag({cols})"))
            if len(keys) == 1:
                c = keys[0][1]
                m.addAction("Derivative d/dt…", lambda: self.new_derived(ds, f"d_{c}", f"ddt({c})"))
                m.addAction("Smoothed…", lambda: self.new_derived(ds, f"{c}_smooth", f"smooth({c}, 25)"))
                m.addAction("Low-pass filtered…", lambda: self.new_derived(ds, f"{c}_lp", f"lowpass({c}, 5)"))
                m.addAction("Negated…", lambda: self.new_derived(ds, f"neg_{c}", f"-{c}"))
                m.addAction("Use as time column", lambda: self._use_time_col(ds, c))
                if c in ds.derived:
                    m.addSeparator()
                    m.addAction("Edit derived signal…", lambda: self.new_derived(ds, c, ds.derived[c]))
                    m.addAction("Delete derived signal", lambda: self.delete_derived(ds, c))
        else:
            m.addAction("New derived signal…", self.new_derived)
        m.exec(self.tree.viewport().mapToGlobal(pos))

    # ---- time settings ----------------------------------------------------------------
    def _sync_time_controls(self):
        ds = self.current_dataset()
        self.time_grp.setEnabled(ds is not None)
        widgets = (self.time_col_combo, self.time_unit_combo, self.zero_cb, self.offset_spin)
        for w in widgets:
            w.blockSignals(True)
        self.time_col_combo.clear()
        if ds is not None:
            self.time_grp.setTitle(f"Time axis: {ds.name}")
            self.time_col_combo.addItem("(row number)", None)
            for c in ds.base_columns:
                self.time_col_combo.addItem(c, c)
            idx = self.time_col_combo.findData(ds.time_col)
            self.time_col_combo.setCurrentIndex(max(idx, 0))
            self.time_unit_combo.setCurrentText(ds.time_unit)
            self.zero_cb.setChecked(ds.zero_start)
            self.offset_spin.setValue(ds.offset)
        for w in widgets:
            w.blockSignals(False)

    def _time_controls_changed(self, *_):
        ds = self.current_dataset()
        if ds is None:
            return
        if self.offset_spin.value() != ds.offset:
            self.set_align_mode("manual")      # hand-edited offset overrides automatic alignment
        ds.time_col = self.time_col_combo.currentData()
        ds.time_unit = self.time_unit_combo.currentText()
        ds.zero_start = self.zero_cb.isChecked()
        ds.offset = self.offset_spin.value()
        ds.compute_time()
        self.rebuild_files()
        if ds.time_col and not np.all(np.diff(ds.time[np.isfinite(ds.time)]) >= 0):
            self.status.showMessage(f"Warning: '{ds.time_col}' is not monotonic; readout may be off", 8000)
        self.refresh_all()
        self.fit_all()

    def _use_time_col(self, ds, c):
        ds.time_col = c
        ds.compute_time()
        self.rebuild_tree(select=ds)
        self.refresh_all()
        self.fit_all()

    # ---- derived ----------------------------------------------------------------------
    def new_derived(self, ds=None, name="", expr=""):
        ds = ds or self.current_dataset()
        if ds is None:
            return
        editing = name in ds.derived
        dlg = DerivedDialog(self, ds, name, expr)
        if dlg.exec() == QtWidgets.QDialog.DialogCode.Accepted:
            new = dlg.name_edit.text().strip()
            if self.compare_mode:   # same signal for every flight of this format
                for d in self.same_format(ds):
                    if d is not ds:
                        try:
                            d.set_derived(new, ds.derived[new])
                        except Exception as exc:
                            self.status.showMessage(f"{new} failed for {d.label}: {exc}", 8000)
            self.rebuild_tree(select=ds)
            if dlg.add_cb.isChecked() and not editing:
                self.add_series([(ds.id, new)], self.active_panel)
            self.refresh_all()

    def delete_derived(self, ds, name):
        targets = [d for d in self.same_format(ds) if name in d.derived] if self.compare_mode else [ds]
        for d in targets:
            for p in self.panels:
                if p.has((d.id, name)):
                    p.remove((d.id, name))
            d.remove_derived(name)
        self.rebuild_tree(select=ds)

    # ---- panels -----------------------------------------------------------------------
    # ---- graph list scrolling & heights -----------------------------------------------
    def set_wheel_zooms(self, on):
        self.wheel_zooms = on
        self.settings.setValue("wheel_zooms", on)

    def plot_wheel(self, ev) -> bool:
        """Wheel over a graph: scroll the graph list (True) or let the graph zoom (False)."""
        ctrl = bool(ev.modifiers() & Qt.KeyboardModifier.ControlModifier)
        if ctrl != self.wheel_zooms:
            return False
        sb = self.scroll.verticalScrollBar()
        dy = ev.pixelDelta().y() or int(ev.angleDelta().y() * 0.8)
        if sb.maximum() == 0 or dy == 0:
            return True   # nothing to scroll; swallow so the graph doesn't zoom unexpectedly
        sb.setValue(sb.value() - dy)
        return True

    def _scroll_page(self, direction):
        sb = self.scroll.verticalScrollBar()
        sb.setValue(sb.value() + direction * sb.pageStep())

    def fit_graphs_to_window(self):
        n = len(self.panels)
        if not n:
            return
        avail = self.scroll.viewport().height() - self.graph_layout.spacing() * (n - 1)
        for p in self.panels:
            p.set_height(avail // n)

    def default_heights(self):
        for p in self.panels:
            p.set_height(DEFAULT_PANEL_H)

    def _resize_active(self, dh):
        if self.active_panel is not None:
            self.active_panel.set_height(self.active_panel.height() + dh)
            self._ensure_visible(self.active_panel)

    def _ensure_visible(self, p):
        QtCore.QTimer.singleShot(0, lambda: p in self.panels and self.scroll.ensureWidgetVisible(p, 0, 0))

    def new_panel(self, title="", height=DEFAULT_PANEL_H) -> PlotPanel:
        self.empty_lbl.hide()
        p = PlotPanel(self, title)
        p.set_height(height)
        self.graph_layout.insertWidget(1 + len(self.panels), p)
        self.panels.append(p)
        p.vb.sigXRangeChanged.connect(self._x_range_changed)
        self._relink()
        p.apply_theme()
        self.set_active_panel(p)
        self.refresh_state_shading()
        if self.measure_on:
            for w, ln in p.ab_lines.items():
                ln.blockSignals(True)
                ln.setValue(self.t_ab[w])
                ln.blockSignals(False)
                ln.setVisible(True)
        return p

    def remove_panel(self, p):
        if p not in self.panels:
            return
        self.panels.remove(p)
        self.graph_layout.removeWidget(p)
        p.setParent(None)
        p.deleteLater()
        if self.active_panel is p:
            self.active_panel = None
            if self.panels:
                self.set_active_panel(self.panels[-1])
        self._relink()
        self.empty_lbl.setVisible(not self.panels)
        self.series_changed()

    def clear_panels(self):
        for p in list(self.panels):
            self.remove_panel(p)

    def move_panel(self, p, delta):
        i = self.panels.index(p)
        j = i + delta
        if 0 <= j < len(self.panels):
            self.panels.insert(j, self.panels.pop(i))
            self.graph_layout.removeWidget(p)
            self.graph_layout.insertWidget(1 + j, p)
            self._relink()
            self.series_changed()
            self._ensure_visible(p)

    def _relink(self):
        if not self.panels:
            return
        master = self.panels[0].plot
        master.setXLink(None)
        for p in self.panels:
            p.plot.getAxis("bottom").setStyle(showValues=True)
            p.plot.showAxis("bottom")
        for p in self.panels[1:]:
            p.plot.setXLink(master)
        # Only the bottom graph shows time labels, to save space
        for p in self.panels[:-1]:
            p.plot.getAxis("bottom").setLabel("")
        self.panels[-1].plot.setLabel("bottom", "Time", units="s")
        self._apply_limits()

    def set_active_panel(self, p):
        if self.active_panel is p:
            return
        old = self.active_panel
        self.active_panel = p
        if old is not None and old in self.panels:
            old.apply_theme(False)
        if p is not None:
            p.apply_theme(True)

    def add_series(self, keys, panel=None, title=None, expand=None):
        keys = [k for k in keys if k[0] in self.datasets and k[1] in self.datasets[k[0]].df]
        if not keys:
            return None
        if self.compare_mode if expand is None else expand:
            keys = self.expand_compare(keys)
        if panel is None or panel not in self.panels:
            panel = self.new_panel(title or (keys[0][1] if len(keys) == 1 else ""))
            self._ensure_visible(panel)
        first = not any(p.series for p in self.panels if p is not panel) and not panel.series
        for k in keys:
            panel.add(k)
        self.set_active_panel(panel)
        self.series_changed()
        if first:
            self.fit_all()
        return panel

    def series_label(self, key):
        ds = self.datasets.get(key[0])
        if ds is None:
            return key[1]
        return key[1] if len(self.datasets) <= 1 else f"{ds.label} · {key[1]}"

    def auto_layout(self, ds=None, clear=False):
        ds = ds or self.current_dataset()
        if ds is None:
            return
        if clear:
            self.clear_panels()
        made = 0
        for title, cols in PRESETS:
            present = [c for c in cols if c in ds.df]
            if present:
                self.add_series([(ds.id, c) for c in present], None, title)
                made += 1
                if made >= AUTO_LAYOUT_MAX:
                    break
        if not made:
            numeric = [c for c in ds.df.columns if c != ds.time_col][:3]
            for c in numeric:
                self.add_series([(ds.id, c)], None)
        self.fit_all()
        QtCore.QTimer.singleShot(0, lambda: self.scroll.verticalScrollBar().setValue(0))

    def _build_presets_menu(self):
        m = self.presets_menu
        m.clear()
        ds = self.current_dataset()
        if ds is None:
            m.addAction("(open a file first)").setEnabled(False)
            return
        for title, cols in PRESETS:
            present = [c for c in cols if c in ds.df]
            if present:
                m.addAction(f"{title}  ({', '.join(present)})",
                            lambda p=present, t=title: self.add_series([(ds.id, c) for c in p], None, t))
        m.addSeparator()
        m.addAction("All presets", lambda: [self.add_series([(ds.id, c) for c in cols if c in ds.df], None, t)
                                            for t, cols in PRESETS if any(c in ds.df for c in cols)])

    # ---- data access ------------------------------------------------------------------
    def series_xy(self, key):
        ds = self.datasets.get(key[0])
        if ds is None or key[1] not in ds.df:
            return None
        t, y = ds.time, ds.array(key[1])
        if self.crop:
            i0, i1 = ds.index_range(*self.crop)
            return t[i0:i1], y[i0:i1]
        return t, y

    def data_extent(self):
        lo, hi = math.inf, -math.inf
        keys = {s["key"] for p in self.panels for s in p.series}
        ids = {k[0] for k in keys} or set(self.datasets)
        for i in ids:
            t = self.datasets[i].time
            t = t[np.isfinite(t)]
            if len(t):
                lo, hi = min(lo, t[0]), max(hi, t[-1])
        if self.crop:
            lo, hi = max(lo, self.crop[0]), min(hi, self.crop[1])
        return (lo, hi) if lo < hi else None

    def value_at(self, key, t):
        ds = self.datasets.get(key[0])
        if ds is None or t is None or not len(ds.time) or key[1] not in ds.df:
            return math.nan
        tt = ds.time
        if t < tt[0] or t > tt[-1]:
            return math.nan
        i = int(np.searchsorted(tt, t))
        i = min(max(i, 0), len(tt) - 1)
        if i > 0 and abs(tt[i - 1] - t) < abs(tt[i] - t):
            i -= 1
        return float(ds.array(key[1])[i])

    def x_range(self):
        if not self.panels:
            return None
        return tuple(self.panels[0].vb.viewRange()[0])

    def range_for(self, name):
        if name.startswith("A") and self.measure_on:
            return tuple(sorted(self.t_ab.values()))
        if name.startswith("Visible") and self.x_range():
            return self.x_range()
        if self.crop:
            return self.crop
        return (-math.inf, math.inf)

    def stats_range(self):
        if self.measure_on:
            return tuple(sorted(self.t_ab.values())), "A–B"
        r = self.x_range()
        return (r, "visible") if r else ((-math.inf, math.inf), "all")

    # ---- refresh ----------------------------------------------------------------------
    def refresh_all(self):
        for p in self.panels:
            p.refresh_all()
        self._apply_limits()
        self.refresh_state_shading()
        self.series_changed()

    def series_changed(self):
        self.overview_timer.start()
        self.stats_timer.start()
        self.readout_timer.start()

    def _apply_limits(self):
        ext = self.data_extent()
        for p in self.panels:
            if ext:
                span = ext[1] - ext[0]
                p.vb.setLimits(xMin=ext[0] - 0.05 * span, xMax=ext[1] + 0.05 * span, minXRange=1e-6)
            else:
                p.vb.setLimits(xMin=None, xMax=None)

    def refresh_overview(self):
        visible = [s["key"] for p in self.panels for s in p.series if s["item"].isVisible()]
        # Prefer an altitude trace (most recognisable flight shape), else the first visible series
        ref = next((k for k in visible if k[1] in ("alt_m", "baro_alt_m")), None) or \
            next((k for k in visible if "alt" in k[1].lower()), None) or (visible[0] if visible else None)
        if ref is None:
            self.ov_curve.setData([], [])
            return
        xy = self.series_xy(ref)
        if xy is not None:
            self.ov_curve.setData(*xy)
        ext = self.data_extent()
        if ext:
            self.overview.setXRange(*ext, padding=0.01)
            self.overview.getPlotItem().enableAutoRange(axis="y")
        self._x_range_changed()

    def refresh_state_shading(self):
        segs = []
        if self.shade_states:
            for ds in self.datasets.values():
                col = ds.state_column() if ds.visible else None
                if col:
                    segs = ds.segments(col)
                    break
        if self.crop:
            c0, c1 = self.crop
            segs = [(max(a, c0), min(b, c1), s) for a, b, s in segs if b > c0 and a < c1]
        for p in self.panels:
            p.set_state_shading(segs)

    # ---- view control -----------------------------------------------------------------
    def fit_all(self):
        ext = self.data_extent()
        if ext and self.panels:
            self.panels[0].vb.setXRange(*ext, padding=0.0)
        self.fit_y_all()

    def fit_y_all(self):
        for p in self.panels:
            p.fit_y()

    def set_x(self, a, b):
        if self.panels and b > a:
            self.panels[0].vb.setXRange(a, b, padding=0.0)

    def zoom(self, factor):
        r = self.x_range()
        if not r:
            return
        c = self.cursor_t if (self.cursor_t is not None and r[0] <= self.cursor_t <= r[1]) else sum(r) / 2
        self.set_x(c - (c - r[0]) * factor, c + (r[1] - c) * factor)

    def pan(self, frac):
        r = self.x_range()
        if r:
            d = (r[1] - r[0]) * frac
            self.set_x(r[0] + d, r[1] + d)

    def center_view(self, x):
        r = self.x_range()
        if r:
            h = (r[1] - r[0]) / 2
            self.set_x(x - h, x + h)

    def _x_range_changed(self, *_):
        r = self.x_range()
        if r is None:
            return
        self._syncing = True
        self.ov_region.setRegion(r)
        self._syncing = False
        self.stats_timer.start()
        self.history_timer.start()

    def _overview_region_changed(self):
        if self._syncing:
            return
        a, b = self.ov_region.getRegion()
        self.set_x(a, b)

    def _push_view(self):
        r = self.x_range()
        if r is None:
            return
        if self.view_pos >= 0 and np.allclose(self.view_hist[self.view_pos], r, rtol=0, atol=1e-9):
            return
        self.view_hist = self.view_hist[: self.view_pos + 1] + [r]
        self.view_hist = self.view_hist[-100:]
        self.view_pos = len(self.view_hist) - 1
        self.view_changed.emit()

    def _goto_view(self, pos):
        if 0 <= pos < len(self.view_hist):
            self.view_pos = pos
            r = self.view_hist[pos]
            self.set_x(*r)
            self.history_timer.stop()

    def view_back(self):
        self._goto_view(self.view_pos - 1)

    def view_forward(self):
        self._goto_view(self.view_pos + 1)

    def set_mode(self, mode):
        self.drag_mode = mode
        cursor = Qt.CursorShape.ArrowCursor if mode == "pan" else Qt.CursorShape.CrossCursor
        for p in self.panels:
            p.pw.viewport().setCursor(cursor)
        self._update_mode_label()

    def _update_mode_label(self):
        names = {"pan": "Pan", "zoom": "Zoom select: drag across a graph",
                 "crop": "Crop select: drag across a graph"}
        self.mode_lbl.setText(f"  Mode: {names[self.drag_mode]}  ")
        if self.crop:
            self.crop_lbl.setText(f"  Cropped {self.crop[0]:.3f} → {self.crop[1]:.3f} s  ")
        else:
            self.crop_lbl.setText("  No crop  ")

    def on_span_selected(self, mode, x0, x1, vb):
        a, b = sorted((x0, x1))
        px = vb.viewPixelSize()[0]
        if (b - a) < 3 * px:
            return
        if mode == "zoom":
            self.set_x(a, b)
        else:
            self.apply_crop(a, b)

    # ---- crop -------------------------------------------------------------------------
    def apply_crop(self, a, b):
        if b <= a:
            return
        self.crop_history.append(self.crop)
        self.crop = (a, b)
        self._crop_changed()

    def crop_to_view(self):
        r = self.x_range()
        if r:
            self.apply_crop(*r)

    def crop_to_ab(self):
        if not self.measure_on:
            self.status.showMessage("Turn on the measurement cursors (M) and place A and B first", 5000)
            return
        self.apply_crop(*sorted(self.t_ab.values()))

    def undo_crop(self):
        if self.crop_history:
            self.crop = self.crop_history.pop()
            self._crop_changed()

    def reset_crop(self):
        if self.crop is not None:
            self.crop_history.append(self.crop)
            self.crop = None
            self._crop_changed()

    def _crop_changed(self):
        self.refresh_all()
        self.fit_all()
        self._update_mode_label()
        self.view_changed.emit()

    # ---- cursors & readout ------------------------------------------------------------
    def on_mouse_moved(self, panel, pos):
        if not panel.vb.sceneBoundingRect().contains(pos):
            return
        self.cursor_t = panel.vb.mapSceneToView(pos).x()
        for p in self.panels:
            p.cursor_line.setValue(self.cursor_t)
        if not self.readout_timer.isActive():
            self.readout_timer.start()

    def set_measure(self, on):
        self.measure_on = on
        if on:
            r = self.x_range() or (0.0, 1.0)
            span = r[1] - r[0]
            for w, f in (("A", 1 / 3), ("B", 2 / 3)):
                if not (r[0] <= self.t_ab[w] <= r[1]):
                    self.t_ab[w] = r[0] + f * span
        for p in self.panels:
            for w, ln in p.ab_lines.items():
                ln.blockSignals(True)
                ln.setValue(self.t_ab[w])
                ln.blockSignals(False)
                ln.setVisible(on)
        self.stats_timer.start()
        self.readout_timer.start()
        self.view_changed.emit()

    def place_ab(self, which):
        if self.cursor_t is None:
            return
        self.t_ab[which] = self.cursor_t
        if not self.measure_on:
            self.measure_act.setChecked(True)
        self.on_ab_moved(which, self.cursor_t)

    def on_ab_moved(self, which, value):
        if self._syncing:
            return
        self._syncing = True
        self.t_ab[which] = value
        for p in self.panels:
            ln = p.ab_lines[which]
            if ln.value() != value:
                ln.setValue(value)
        self._syncing = False
        self.stats_timer.start()
        self.readout_timer.start()

    def set_shading(self, on):
        self.shade_states = on
        self.refresh_state_shading()

    def _rows(self):
        return [(p, s) for p in self.panels for s in p.series if s["item"].isVisible()]

    def update_stats(self):
        (a, b), _ = self.stats_range()
        if self.crop:
            a, b = max(a, self.crop[0]), min(b, self.crop[1])
        self._stats = {}
        for _p, s in self._rows():
            key = s["key"]
            ds = self.datasets.get(key[0])
            if ds is None or key[1] not in ds.df:
                continue
            i0, i1 = ds.index_range(a, b)
            y = ds.array(key[1])[i0:i1]
            y = y[np.isfinite(y)]
            if len(y):
                self._stats[key] = (y.min(), y.max(), y.mean(), y.std())
            else:
                self._stats[key] = (math.nan,) * 4
        self.update_readout()

    def update_readout(self):
        rows = self._rows()
        stats = getattr(self, "_stats", {})
        th = self.theme
        self.table.setColumnHidden(3, not self.measure_on)
        self.table.setColumnHidden(4, not self.measure_on)
        self.table.setColumnHidden(5, not self.measure_on)
        if self.table.rowCount() != len(rows):
            self.table.setRowCount(len(rows))
        tA, tB = self.t_ab["A"], self.t_ab["B"]
        for r, (p, s) in enumerate(rows):
            key = s["key"]
            vc = self.value_at(key, self.cursor_t)
            va = self.value_at(key, tA) if self.measure_on else math.nan
            vb = self.value_at(key, tB) if self.measure_on else math.nan
            st = stats.get(key, (math.nan,) * 4)
            unit = unit_of(key[1])
            vals = [p.title, self.series_label(key) + (f" [{unit}]" if unit else ""), fmt(vc), fmt(va),
                    fmt(vb), fmt(vb - va), *[fmt(x) for x in st]]
            for c, v in enumerate(vals):
                it = self.table.item(r, c)
                if it is None:
                    it = QtWidgets.QTableWidgetItem()
                    if c >= 2:
                        it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    self.table.setItem(r, c, it)
                if it.text() != v:
                    it.setText(v)
            sw = self.table.item(r, 1)
            sw.setData(Qt.ItemDataRole.DecorationRole, QtGui.QColor(p.pen_for(s).color()))
        (a, b), label = self.stats_range()
        parts = [f"<b>Cursor</b> {fmt_t(self.cursor_t) if self.cursor_t is not None else '—'}"]
        if self.measure_on:
            dt = tB - tA
            hz = f" ({1 / abs(dt):.4g} Hz)" if dt else ""
            parts.append(f"<b>A</b> {fmt_t(tA)} &nbsp; <b>B</b> {fmt_t(tB)} &nbsp; <b>Δt</b> {dt:.4f} s{hz}")
        if math.isfinite(a) and math.isfinite(b):
            parts.append(f"<b>Stats over {label}</b> {a:.3f} → {b:.3f} s (Δ {b - a:.3f} s)")
        state = self._state_at(self.cursor_t)
        if state:
            parts.append(f"<b>State</b> {state}")
        self.readout_lbl.setText(" &nbsp;&nbsp;│&nbsp;&nbsp; ".join(parts))
        if self.cursor_t is not None:
            self.status.showMessage(f"t = {self.cursor_t:.4f} s")

    def _state_at(self, t):
        if t is None:
            return None
        for ds in self.datasets.values():
            col = ds.state_column()
            if col:
                v = self.value_at((ds.id, col), t)
                if math.isfinite(v):
                    return STATE_NAMES.get(int(v), str(int(v)))
        return None

    # ---- events -----------------------------------------------------------------------
    def find_events(self):
        events = []
        for ds in self.datasets.values():
            if not ds.visible:
                continue
            pre = "" if len(self.datasets) == 1 else f"{ds.label}: "
            col = ds.state_column()
            if col:
                for t0, _t1, st in ds.segments(col)[1:]:
                    events.append((t0, f"{pre}{STATE_NAMES.get(st, st)} begins"))
            for c in ("alt_m", "baro_alt_m"):
                if c in ds.df:
                    a = ds.array(c)
                    if np.isfinite(a).any():
                        i = int(np.nanargmax(a))
                        events.append((ds.time[i], f"{pre}Apogee ({c} = {a[i]:.1f} m)"))
                    break
            for c in ("imu_az_g", "kx_az_g", "ax_mps2", "high_g_accel_g"):
                if c in ds.df:
                    a = np.abs(ds.array(c))
                    if np.isfinite(a).any():
                        i = int(np.nanargmax(a))
                        events.append((ds.time[i], f"{pre}Peak |{c}| = {a[i]:.2f}"))
                    break
        return sorted(events)

    def _build_events_menu(self):
        m = self.events_menu
        m.clear()
        ev = self.find_events()
        if not ev:
            m.addAction("(no events found)").setEnabled(False)
        for t, name in ev:
            m.addAction(f"{t:9.3f} s   {name}", lambda t=t: self.center_view(t))

    # ---- tools ------------------------------------------------------------------------
    def open_xy(self):
        if not self.datasets:
            return
        w = XYWindow(self)
        w.show()

    def flight_summary(self):
        lines = []
        for ds in self.datasets.values():
            t = ds.time[np.isfinite(ds.time)]
            dt = np.diff(t)
            dt = dt[dt > 0]
            lines.append(f"=== {ds.label} ({ds.name}) ===")
            lines.append(f"rows {ds.n:,}   columns {len(ds.base_columns)}   time column {ds.time_col}")
            if len(t):
                lines.append(f"duration {t[-1] - t[0]:.3f} s   ({t[0]:.3f} → {t[-1]:.3f} s)")
            if len(dt):
                med = np.median(dt)
                gaps = int((dt > 5 * med).sum())
                lines.append(f"sample rate ≈ {1 / med:.1f} Hz (median Δt {med * 1e3:.3f} ms), "
                             f"max gap {dt.max() * 1e3:.1f} ms, {gaps} gaps > 5× median")

            def peak(c, absval=False, unit=""):
                if c in ds.df:
                    a = ds.array(c)
                    a2 = np.abs(a) if absval else a
                    if np.isfinite(a2).any():
                        i = int(np.nanargmax(a2))
                        lines.append(f"  max {'|' + c + '|' if absval else c:<22} {a2[i]:12.3f} {unit:6} at t = {ds.time[i]:.3f} s")

            lines.append("Peaks:")
            for c, u in (("alt_m", "m"), ("baro_alt_m", "m"), ("speed_mps", "m/s"), ("mach", ""),
                         ("qbar_pa", "Pa"), ("tilt_deg", "deg")):
                peak(c, unit=u)
            for c, u in (("vd_mps", "m/s"), ("imu_ax_g", "g"), ("imu_ay_g", "g"), ("imu_az_g", "g"),
                         ("kx_ax_g", "g"), ("kx_ay_g", "g"), ("kx_az_g", "g"), ("high_g_accel_g", "g"),
                         ("roll_rate_dps", "deg/s"), ("p_dps", "deg/s"), ("imu_gz_dps", "deg/s"),
                         ("tab_deg", "deg"), ("current_ma", "mA")):
                peak(c, absval=True, unit=u)
            if "bus_v" in ds.df:
                v = ds.array("bus_v")
                lines.append(f"  bus voltage {np.nanmin(v):.3f} … {np.nanmax(v):.3f} V")
            col = ds.state_column()
            if col:
                lines.append("Flight states:")
                for t0, t1, st in ds.segments(col):
                    lines.append(f"  {t0:10.3f} → {t1:10.3f} s  ({t1 - t0:8.3f} s)  {STATE_NAMES.get(st, st)}")
            if "dropped" in ds.df:
                lines.append(f"rows dropped by logger: {int(np.nanmax(ds.array('dropped')))}")
            if ds.derived:
                lines.append("Derived: " + ", ".join(f"{k} = {v}" for k, v in ds.derived.items()))
            lines.append("")
        TextDialog(self, "Flight summary", "\n".join(lines) or "No files open").exec()

    def column_stats(self):
        ds = self.current_dataset()
        if ds is None:
            return
        (a, b), label = self.stats_range()
        if self.crop:
            a, b = max(a, self.crop[0]), min(b, self.crop[1])
        i0, i1 = ds.index_range(a, b)
        lines = [f"{ds.name}: {label} range, {i1 - i0:,} rows", "",
                 f"{'column':<20}{'min':>14}{'max':>14}{'mean':>14}{'std':>14}"]
        for c in ds.df.columns:
            y = ds.array(c)[i0:i1]
            y = y[np.isfinite(y)]
            if len(y):
                lines.append(f"{c:<20}{y.min():>14.6g}{y.max():>14.6g}{y.mean():>14.6g}{y.std():>14.6g}")
            else:
                lines.append(f"{c:<20}{'—':>14}")
        TextDialog(self, "Column statistics", "\n".join(lines)).exec()

    # ---- export -----------------------------------------------------------------------
    def export_png(self):
        if not self.panels:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export graphs", self._default_out(".png"),
                                                        "PNG image (*.png)")
        if path:
            # Whole graph column, including graphs scrolled out of view
            bottom = max(p.geometry().bottom() for p in self.panels) + 1
            self.graph_host.grab(QtCore.QRect(0, 0, self.graph_host.width(), bottom)).save(path)
            self.status.showMessage(f"Saved {path}", 6000)

    def _default_out(self, ext):
        ds = self.current_dataset()
        d = self.settings.value("last_dir", "")
        return os.path.join(d, (ds.short if ds else "flight") + ("_crop" if self.crop else "") + ext)

    def export_csv(self):
        ds = self.current_dataset()
        if ds is None:
            return
        ranges = ["Crop range" if self.crop else "Whole file", "Visible range"]
        if self.measure_on:
            ranges.append("A–B range")
        rng, ok = QtWidgets.QInputDialog.getItem(self, "Export CSV", f"Rows of {ds.name}:", ranges, 0, False)
        if not ok:
            return
        cols_opt, ok = QtWidgets.QInputDialog.getItem(
            self, "Export CSV", "Columns:", ["All columns (incl. derived)", "Only plotted columns"], 0, False)
        if not ok:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export CSV", self._default_out(".csv"),
                                                        "CSV (*.csv)")
        if not path:
            return
        a, b = self.range_for(rng)
        i0, i1 = ds.index_range(a, b)
        cols = list(ds.df.columns)
        if cols_opt.startswith("Only"):
            plotted = {s["key"][1] for p in self.panels for s in p.series if s["key"][0] == ds.id}
            cols = [c for c in cols if c in plotted or c == ds.time_col]
        out = ds.df.iloc[i0:i1][cols].copy()
        out.insert(0, "time_s", ds.time[i0:i1])
        out.to_csv(path, index=False)
        self.status.showMessage(f"Wrote {len(out):,} rows to {path}", 6000)

    # ---- layouts ----------------------------------------------------------------------
    def layout_dict(self):
        ids = list(self.datasets)
        return {
            "version": 1,
            "compare_mode": self.compare_mode, "align": self.align_mode,
            "files": [{"path": d.path, "time_col": d.time_col, "time_unit": d.time_unit,
                       "zero_start": d.zero_start, "offset": d.offset, "derived": d.derived,
                       "label": d.label, "color": d.color, "visible": d.visible}
                      for d in self.datasets.values()],
            "graphs": [{"title": p.title, "log_y": p.log_btn.isChecked(), "height": p.height(),
                        "series": [{"file": ids.index(s["key"][0]), "column": s["key"][1], "slot": s["slot"],
                                    "color": s.get("color"), "visible": s["item"].isVisible()}
                                   for s in p.series]} for p in self.panels],
            "crop": self.crop, "x_range": self.x_range(), "shade": self.shade_states,
        }

    def save_layout_dialog(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save layout", self._default_out("_layout.json"),
                                                        "Layout (*.json)")
        if path:
            with open(path, "w") as fh:
                json.dump(self.layout_dict(), fh, indent=2)
            self.status.showMessage(f"Saved layout {path}", 6000)

    def load_layout_dialog(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load layout", self.settings.value("last_dir", ""),
                                                        "Layout (*.json)")
        if path:
            self.load_layout(path)

    def load_layout(self, path):
        try:
            with open(path) as fh:
                lay = json.load(fh)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Layout", f"Could not read layout:\n{exc}")
            return
        files = []
        for f in lay.get("files", []):
            p = f["path"]
            if not os.path.isfile(p):   # try next to the layout file
                alt = os.path.join(os.path.dirname(os.path.abspath(path)), os.path.basename(p))
                p = alt if os.path.isfile(alt) else p
            ds = next((d for d in self.datasets.values() if d.path == os.path.abspath(p)), None)
            if ds is None:
                if not os.path.isfile(p):
                    QtWidgets.QMessageBox.warning(self, "Layout", f"Missing file: {f['path']}")
                    files.append(None)
                    continue
                QtWidgets.QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
                try:
                    ds = Dataset(p)
                finally:
                    QtWidgets.QApplication.restoreOverrideCursor()
                self._register(ds)
            ds.label = f.get("label") or ds.short
            ds.color = f.get("color")
            ds.visible = f.get("visible", True)
            ds.time_col = f.get("time_col", ds.time_col)
            ds.time_unit = f.get("time_unit", "auto")
            ds.zero_start = f.get("zero_start", True)
            ds.offset = f.get("offset", 0.0)
            ds.compute_time()
            for name, expr in f.get("derived", {}).items():
                try:
                    ds.set_derived(name, expr)
                except Exception as exc:
                    self.status.showMessage(f"Derived {name} failed: {exc}", 8000)
            files.append(ds)
        # Restore compare settings without re-aligning (the saved offsets already encode it)
        self.compare_mode = lay.get("compare_mode", self.compare_mode)
        self.compare_cb.blockSignals(True)
        self.compare_cb.setChecked(self.compare_mode)
        self.compare_cb.blockSignals(False)
        self.align_mode = lay.get("align", "manual")
        self.align_combo.setCurrentIndex(max(0, self.align_combo.findData(self.align_mode)))
        self.clear_panels()
        self.crop = tuple(lay["crop"]) if lay.get("crop") else None
        self.crop_history = []
        self.shade_act.setChecked(lay.get("shade", True))
        for g in lay.get("graphs", []):
            p = self.new_panel(g.get("title", ""), g.get("height", DEFAULT_PANEL_H))
            for s in g.get("series", []):
                ds = files[s["file"]] if s["file"] < len(files) else None
                if ds is not None and s["column"] in ds.df:
                    p.add((ds.id, s["column"]), slot=s.get("slot"), color=s.get("color"),
                          visible=s.get("visible", True))
            if g.get("log_y"):
                p.log_btn.setChecked(True)
        self.rebuild_tree()
        self.refresh_all()
        self._update_mode_label()
        if lay.get("x_range"):
            self.set_x(*lay["x_range"])
        else:
            self.fit_all()


def main():
    pg.setConfigOptions(antialias=False, useOpenGL=False)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    win = MainWindow()
    args = sys.argv[1:]
    layout = None
    if "--layout" in args:
        i = args.index("--layout")
        layout = args[i + 1] if i + 1 < len(args) else None
        args = args[:i] + args[i + 2:]
    win.show()
    if args:
        win.open_files(args)
    if layout:
        win.load_layout(layout)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
