"""
tof_view.py

Turns the DFRobot SEN0628 8x8 ToF array into a small depth camera:

- Depth image: the 64 zones as a colour-mapped image (near = warm), with
  optional smoothing between zones and between frames.
- Top-down map: every zone projected into real millimetres in front of
  the sensor, using its 60° field of view.
- Object detection: groups of neighbouring zones that stand out, reported
  with bearing, distance and approximate width. With a captured background
  (the empty scene) anything closer than the background counts, so the
  floor and walls stop being "objects".

The processing half of this file is plain numpy so it can be tested
without a window. Zone keys arrive as tof.array.r<row>c<col>; the firmware
sends tof.array_valid_zones straight after them, which marks a full frame.
"""

from __future__ import annotations

import csv
import json
import math
import time
import warnings
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QPointF, QRectF, QSettings, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGraphicsRectItem,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

import theme


# =====================================================================
# Sensor geometry (DFRobot SEN0628: VL53L7CX, 60° FoV, 3.5 m range)
# =====================================================================

GRID = 8
FOV_DEG = 60.0
MAX_RANGE_MM = 3500.0
ZONE_DEG = FOV_DEG / GRID

ZONE_KEYS = [[f"tof.array.r{row}c{col}" for col in range(GRID)] for row in range(GRID)]

# Neighbouring zones only join one object if their depths are this close,
# so a box in front of a wall isn't merged into the wall.
JOIN_DEPTH_MM = 150.0
BACKGROUND_FRAMES = 10


def zone_bearing_deg(col: float) -> float:
    """Horizontal angle of a (possibly fractional) column centre; + is right."""
    return (col + 0.5 - GRID / 2) * ZONE_DEG


def zone_elevation_deg(row: float) -> float:
    """Vertical angle of a row centre; + is up (row 0 is the top)."""
    return -(row + 0.5 - GRID / 2) * ZONE_DEG


# =====================================================================
# Processing (no Qt)
# =====================================================================

def frame_from_telemetry(telemetry: dict) -> np.ndarray:
    frame = np.full((GRID, GRID), np.nan)
    for row in range(GRID):
        for col in range(GRID):
            value = telemetry.get(ZONE_KEYS[row][col])
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                frame[row, col] = float(value)
    return frame


def orient(frame: np.ndarray, mirror: bool, flip: bool) -> np.ndarray:
    """Raw -> display orientation. Its own inverse."""
    if mirror:
        frame = frame[:, ::-1]
    if flip:
        frame = frame[::-1, :]
    return frame


def smooth_frames(previous: np.ndarray | None, new: np.ndarray, alpha: float) -> np.ndarray:
    """Exponential moving average that never smears invalid zones."""
    if previous is None or alpha <= 0:
        return new.copy()
    blended = alpha * previous + (1 - alpha) * new
    blended = np.where(np.isnan(previous), new, blended)
    blended[np.isnan(new)] = np.nan
    return blended


def upsample(grid: np.ndarray, size: int) -> np.ndarray:
    """Bilinear resize of a square grid to size x size."""
    n = grid.shape[0]
    coords = np.clip((np.arange(size) + 0.5) * n / size - 0.5, 0, n - 1)
    lo = np.floor(coords).astype(int)
    hi = np.minimum(lo + 1, n - 1)
    t = coords - lo
    rows = grid[lo] * (1 - t)[:, None] + grid[hi] * t[:, None]
    return rows[:, lo] * (1 - t)[None, :] + rows[:, hi] * t[None, :]


def median_background(frames: list[np.ndarray]) -> np.ndarray:
    """Per-zone median; zones valid in under half the frames stay invalid."""
    stack = np.stack(frames)
    valid_counts = np.sum(~np.isnan(stack), axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        background = np.nanmedian(stack, axis=0)
    background[valid_counts < len(frames) / 2] = np.nan
    return background


@dataclass
class DetectedObject:
    index: int
    zones: list[tuple[int, int]]
    nearest_mm: float
    mean_mm: float
    bearing_deg: float
    elevation_deg: float
    width_mm: float
    x_mm: float
    y_mm: float
    row_range: tuple[int, int]
    col_range: tuple[int, int]


def detect_objects(
    depth: np.ndarray,
    background: np.ndarray | None,
    near_threshold_mm: float,
    background_margin_mm: float,
    min_zones: int,
) -> list[DetectedObject]:
    valid = ~np.isnan(depth)
    if background is None:
        mask = valid & (depth <= near_threshold_mm)
    else:
        background_valid = ~np.isnan(background)
        with np.errstate(invalid="ignore"):
            closer = valid & background_valid & (background - depth >= background_margin_mm)
        # Something in range where the empty scene had nothing at all.
        appeared = valid & ~background_valid & (depth <= MAX_RANGE_MM)
        mask = closer | appeared

    seen = np.zeros_like(mask)
    blobs: list[list[tuple[int, int]]] = []
    for row in range(GRID):
        for col in range(GRID):
            if not mask[row, col] or seen[row, col]:
                continue
            seen[row, col] = True
            blob, stack = [], [(row, col)]
            while stack:
                r, c = stack.pop()
                blob.append((r, c))
                for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if (
                        0 <= nr < GRID and 0 <= nc < GRID
                        and mask[nr, nc] and not seen[nr, nc]
                        and abs(depth[nr, nc] - depth[r, c]) <= JOIN_DEPTH_MM
                    ):
                        seen[nr, nc] = True
                        stack.append((nr, nc))
            if len(blob) >= min_zones:
                blobs.append(blob)

    objects = []
    for blob in blobs:
        rows = [r for r, _ in blob]
        cols = [c for _, c in blob]
        depths = [depth[r, c] for r, c in blob]
        nearest = float(min(depths))
        bearing = zone_bearing_deg(sum(cols) / len(cols))
        elevation = zone_elevation_deg(sum(rows) / len(rows))
        span_rad = math.radians((max(cols) - min(cols) + 1) * ZONE_DEG)
        ground = nearest * math.cos(math.radians(elevation))
        objects.append(DetectedObject(
            index=0,
            zones=blob,
            nearest_mm=nearest,
            mean_mm=float(sum(depths) / len(depths)),
            bearing_deg=bearing,
            elevation_deg=elevation,
            width_mm=2 * nearest * math.tan(span_rad / 2),
            x_mm=ground * math.sin(math.radians(bearing)),
            y_mm=ground * math.cos(math.radians(bearing)),
            row_range=(min(rows), max(rows)),
            col_range=(min(cols), max(cols)),
        ))

    objects.sort(key=lambda obj: obj.nearest_mm)
    for number, obj in enumerate(objects, start=1):
        obj.index = number
    return objects


def project_zones(depth: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Valid zones -> top-down (x right, y forward) millimetres, plus depths."""
    xs, ys, ds = [], [], []
    for row in range(GRID):
        elevation = math.radians(zone_elevation_deg(row))
        for col in range(GRID):
            d = depth[row, col]
            if np.isnan(d):
                continue
            bearing = math.radians(zone_bearing_deg(col))
            ground = d * math.cos(elevation)
            xs.append(ground * math.sin(bearing))
            ys.append(ground * math.cos(bearing))
            ds.append(d)
    return np.array(xs), np.array(ys), np.array(ds)


def describe_angle(degrees: float, positive: str, negative: str) -> str:
    if abs(degrees) < 0.5:
        return "0°"
    return f"{abs(degrees):.0f}° {positive if degrees > 0 else negative}"


# =====================================================================
# Widget
# =====================================================================

SMOOTHING_LEVELS = (("Off", 0.0), ("Light", 0.3), ("Medium", 0.55), ("Heavy", 0.75))


def _depth_colormap() -> pg.ColorMap:
    """Turbo, reversed so near is red and far is blue. Built by hand
    because ColorMap.reversed() doesn't exist on older pyqtgraph."""
    base = pg.colormap.get("turbo")
    # Byte colours: the constructor reads floats as 0-255, so passing the
    # stored 0-1 floats straight through gives an almost-black map.
    positions, colours = base.getStops(mode=pg.ColorMap.BYTE)
    return pg.ColorMap(1.0 - positions[::-1], colours[::-1])


class TofView(QWidget):
    zone_plot_requested = pyqtSignal(str)

    STALE_AFTER_S = 2.0
    UPSAMPLE_SIZE = 64

    def __init__(
        self,
        settings: QSettings,
        telemetry_source: Callable[[], dict],
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.settings = settings
        self.telemetry_source = telemetry_source
        self.colormap = _depth_colormap()

        self._raw: np.ndarray | None = None
        self._smoothed: np.ndarray | None = None
        self._display: np.ndarray | None = None
        self._objects: list[DetectedObject] = []
        self._pending = False
        self._frame_times: deque[float] = deque(maxlen=40)
        self._capture_frames: list[np.ndarray] | None = None
        self._selected: tuple[int, int] | None = None  # raw orientation
        self._hovered: tuple[int, int] | None = None   # raw orientation
        self._was_stale = True

        self.background = self._load_background()

        self._build()
        self._load_settings()
        self._render()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(40)

    # ---------------- settings ----------------

    def _setting(self, key: str, default):
        value = self.settings.value(f"tof/{key}", default)
        if isinstance(default, bool):
            return value in (True, "true", "True", 1, "1")
        try:
            return type(default)(value)
        except (TypeError, ValueError):
            return default

    def _load_settings(self):
        widgets = (
            (self.smooth_image_check, "smooth_image", True),
            (self.show_values_check, "show_values", False),
            (self.mirror_check, "mirror", False),
            (self.flip_check, "flip", False),
        )
        for widget, key, default in widgets:
            widget.blockSignals(True)
            widget.setChecked(self._setting(key, default))
            widget.blockSignals(False)
        for spin, key, default in (
            (self.range_spin, "range_mm", 2000),
            (self.threshold_spin, "near_threshold_mm", 600),
            (self.margin_spin, "background_margin_mm", 80),
            (self.min_zones_spin, "min_zones", 2),
        ):
            spin.blockSignals(True)
            spin.setValue(self._setting(key, default))
            spin.blockSignals(False)
        self.smoothing_combo.blockSignals(True)
        self.smoothing_combo.setCurrentIndex(
            max(0, min(self._setting("smoothing", 1), len(SMOOTHING_LEVELS) - 1))
        )
        self.smoothing_combo.blockSignals(False)
        self._update_background_ui()
        self._rebuild_map_guides()

    def _save_setting(self, key: str, value):
        self.settings.setValue(f"tof/{key}", value)

    def _load_background(self) -> np.ndarray | None:
        try:
            data = json.loads(str(self.settings.value("tof/background", "")))
            grid = np.array(
                [[np.nan if v is None else float(v) for v in row] for row in data["zones"]]
            )
            if grid.shape == (GRID, GRID):
                self.background_time = str(data.get("captured", ""))
                return grid
        except (TypeError, ValueError, KeyError):
            pass
        self.background_time = ""
        return None

    def _save_background(self):
        if self.background is None:
            self.settings.remove("tof/background")
            return
        zones = [[None if np.isnan(v) else round(float(v), 1) for v in row] for row in self.background]
        self.settings.setValue(
            "tof/background", json.dumps({"zones": zones, "captured": self.background_time})
        )

    # ---------------- layout ----------------

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)

        top = QHBoxLayout()
        title = QLabel("Depth Camera")
        title.setObjectName("sectionTitle")
        top.addWidget(title)
        self.device_label = QLabel("8×8 ToF array")
        self.device_label.setObjectName("hint")
        top.addWidget(self.device_label)
        top.addStretch()

        self.freeze_button = QPushButton("Freeze")
        self.freeze_button.setCheckable(True)
        self.freeze_button.setToolTip("Hold the current frame")
        self.save_image_button = QPushButton("Save PNG")
        self.export_button = QPushButton("Export CSV")
        self.export_button.setToolTip("Current frame in raw sensor orientation (matches tof.array.rXcY)")
        for button in (self.freeze_button, self.save_image_button, self.export_button):
            top.addWidget(button)
        top.addSpacing(8)

        self.nearest_pill = self._pill(top)
        self.valid_pill = self._pill(top)
        self.objects_pill = self._pill(top)
        self.fps_pill = self._pill(top)
        root.addLayout(top)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        # ---- depth image ----
        self.image_plot = pg.PlotWidget()
        theme.style_plot(self.image_plot)
        item = self.image_plot.getPlotItem()
        item.showGrid(False, False)
        item.setTitle("Depth image", color=theme.TEXT_MUTED, size="10pt")
        item.hideButtons()
        item.setMenuEnabled(False)
        view = item.getViewBox()
        view.setAspectLocked(True)
        view.invertY(True)
        view.setMouseEnabled(False, False)
        view.setRange(xRange=(0, GRID), yRange=(0, GRID), padding=0.02)
        # Ticks on zone edges, which land on whole angles (±30°, ±15°, 0°).
        edge_ticks = (0, 2, 4, 6, 8)
        item.getAxis("bottom").setTicks([[
            (edge, describe_angle((edge - GRID / 2) * ZONE_DEG, "R", "L")) for edge in edge_ticks
        ]])
        item.getAxis("left").setTicks([[
            (edge, describe_angle(-(edge - GRID / 2) * ZONE_DEG, "up", "dn")) for edge in edge_ticks
        ]])

        self.image_item = pg.ImageItem()
        self.image_item.setColorMap(self.colormap)
        # Sized in _render() after each setImage: pyqtgraph 0.13 can't size
        # an ImageItem that doesn't hold an image yet.
        item.addItem(self.image_item)

        grid_pen = pg.mkPen(QColor(0, 0, 0, 70), width=1)
        for i in range(1, GRID):
            item.addItem(pg.InfiniteLine(pos=i, angle=90, pen=grid_pen, movable=False))
            item.addItem(pg.InfiniteLine(pos=i, angle=0, pen=grid_pen, movable=False))

        value_font = QFont()
        value_font.setPointSize(9)
        value_font.setBold(True)
        self.value_items = [[None] * GRID for _ in range(GRID)]
        for row in range(GRID):
            for col in range(GRID):
                text = pg.TextItem("", color="#ffffff", anchor=(0.5, 0.5))
                text.setFont(value_font)
                text.setPos(col + 0.5, row + 0.5)
                text.setZValue(20)
                item.addItem(text)
                self.value_items[row][col] = text

        self.selection_rect = QGraphicsRectItem(0, 0, 1, 1)
        selection_pen = QPen(QColor("#ffffff"), 2.5)
        selection_pen.setCosmetic(True)
        self.selection_rect.setPen(selection_pen)
        self.selection_rect.setZValue(40)
        self.selection_rect.hide()
        item.addItem(self.selection_rect)

        self.object_rects: list[QGraphicsRectItem] = []
        self.object_labels: list[pg.TextItem] = []

        self.image_plot.scene().sigMouseClicked.connect(self._on_image_click)
        self.image_plot.scene().sigMouseMoved.connect(self._on_image_hover)
        splitter.addWidget(self.image_plot)

        # ---- top-down map ----
        self.map_plot = pg.PlotWidget()
        theme.style_plot(self.map_plot)
        map_item = self.map_plot.getPlotItem()
        map_item.showGrid(False, False)
        map_item.setTitle("Top-down map", color=theme.TEXT_MUTED, size="10pt")
        # Plain labels: with units="mm" pyqtgraph auto-prefixes to "kmm".
        map_item.setLabel("bottom", "Right of sensor (mm)")
        map_item.setLabel("left", "Forward (mm)")
        for axis in ("bottom", "left"):
            map_item.getAxis(axis).enableAutoSIPrefix(False)
        map_item.hideButtons()
        map_item.getViewBox().setAspectLocked(True)
        self.map_guides: list = []

        self.map_points = pg.ScatterPlotItem(size=9, pen=None)
        self.map_points.setZValue(10)
        map_item.addItem(self.map_points)

        self.map_objects = pg.ScatterPlotItem(
            size=30, symbol="o", brush=None, pen=pg.mkPen("#ffffff", width=2)
        )
        self.map_objects.setZValue(20)
        map_item.addItem(self.map_objects)
        self.map_object_labels: list[pg.TextItem] = []

        sensor = pg.ScatterPlotItem(
            [0], [0], size=16, symbol="t1",
            brush=pg.mkBrush(theme.ACCENT), pen=pg.mkPen(theme.TEXT, width=1),
        )
        sensor.setZValue(30)
        map_item.addItem(sensor)
        splitter.addWidget(self.map_plot)

        # ---- sidebar ----
        splitter.addWidget(self._build_sidebar())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([430, 430, 300])
        root.addWidget(splitter, 1)

    def _pill(self, layout: QHBoxLayout) -> QLabel:
        pill = QLabel("—")
        pill.setObjectName("statusPill")
        layout.addWidget(pill)
        return pill

    def _build_sidebar(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(290)
        scroll.setMaximumWidth(360)
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.setSpacing(10)

        # ---- display ----
        display = QGroupBox("Display")
        form = QFormLayout(display)
        form.setVerticalSpacing(6)
        self.smooth_image_check = QCheckBox("Smooth between zones")
        self.show_values_check = QCheckBox("Show zone values (mm)")
        self.mirror_check = QCheckBox("Mirror left ↔ right")
        self.flip_check = QCheckBox("Flip up ↕ down")
        self.mirror_check.setToolTip(
            "Wave a hand on the sensor's left. If it appears on the right of "
            "the image, tick this."
        )
        for check in (self.smooth_image_check, self.show_values_check,
                      self.mirror_check, self.flip_check):
            form.addRow(check)

        self.smoothing_combo = QComboBox()
        for label, _ in SMOOTHING_LEVELS:
            self.smoothing_combo.addItem(label)
        self.smoothing_combo.setToolTip("Averages frames over time to calm flicker")
        form.addRow("Frame smoothing", self.smoothing_combo)

        self.range_spin = QSpinBox()
        self.range_spin.setRange(300, int(MAX_RANGE_MM))
        self.range_spin.setSingleStep(100)
        self.range_spin.setSuffix(" mm")
        self.range_spin.setToolTip("Distance shown as the far (blue) end of the colour scale")
        form.addRow("Colour range", self.range_spin)

        # ---- detection ----
        detection = QGroupBox("Object Detection")
        detection_layout = QVBoxLayout(detection)
        detection_layout.setSpacing(6)

        self.background_label = QLabel()
        self.background_label.setObjectName("hint")
        self.background_label.setWordWrap(True)
        detection_layout.addWidget(self.background_label)

        background_buttons = QHBoxLayout()
        self.capture_button = QPushButton("Capture Background")
        self.capture_button.setToolTip(
            f"Point the sensor at the empty scene. Averages {BACKGROUND_FRAMES} "
            "frames; afterwards only things closer than it count as objects."
        )
        self.clear_background_button = QPushButton("Clear")
        background_buttons.addWidget(self.capture_button, 1)
        background_buttons.addWidget(self.clear_background_button)
        detection_layout.addLayout(background_buttons)

        detection_form = QFormLayout()
        self.threshold_spin = QSpinBox()
        self.threshold_spin.setRange(50, int(MAX_RANGE_MM))
        self.threshold_spin.setSingleStep(50)
        self.threshold_spin.setSuffix(" mm")
        self.threshold_spin.setToolTip("Without a background: anything nearer than this")
        detection_form.addRow("Nearer than", self.threshold_spin)

        self.margin_spin = QSpinBox()
        self.margin_spin.setRange(20, 1000)
        self.margin_spin.setSingleStep(10)
        self.margin_spin.setSuffix(" mm")
        self.margin_spin.setToolTip("With a background: this much nearer than the empty scene")
        detection_form.addRow("Beats background by", self.margin_spin)

        self.min_zones_spin = QSpinBox()
        self.min_zones_spin.setRange(1, 16)
        self.min_zones_spin.setToolTip("Smallest cluster of zones reported, to ignore single-zone noise")
        detection_form.addRow("Min zones", self.min_zones_spin)
        detection_layout.addLayout(detection_form)

        self.objects_table = QTableWidget(0, 4)
        self.objects_table.setHorizontalHeaderLabels(["#", "Bearing", "mm", "≈ Wide"])
        self.objects_table.verticalHeader().setVisible(False)
        self.objects_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.objects_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.objects_table.setMinimumHeight(128)
        header = self.objects_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        objects = QGroupBox("Objects")
        objects_layout = QVBoxLayout(objects)
        objects_layout.addWidget(self.objects_table)

        # ---- zone ----
        zone = QGroupBox("Zone")
        zone_layout = QVBoxLayout(zone)
        self.zone_label = QLabel("Click a zone to select it; double-click to plot it.")
        self.zone_label.setObjectName("hint")
        self.zone_label.setWordWrap(True)
        zone_layout.addWidget(self.zone_label)
        self.plot_zone_button = QPushButton("Plot Selected Zone")
        self.plot_zone_button.setEnabled(False)
        zone_layout.addWidget(self.plot_zone_button)

        # Most useful first: what was found, then how, then display options.
        for group in (objects, detection, zone, display):
            layout.addWidget(group)
        layout.addStretch()

        scroll.setWidget(container)

        for check, key in (
            (self.smooth_image_check, "smooth_image"),
            (self.show_values_check, "show_values"),
        ):
            check.toggled.connect(lambda on, k=key: self._on_setting(k, on))
        for check, key in ((self.mirror_check, "mirror"), (self.flip_check, "flip")):
            check.toggled.connect(lambda on, k=key: self._on_setting(k, on, reset_smoothing=True))
        self.smoothing_combo.currentIndexChanged.connect(
            lambda index: self._on_setting("smoothing", index, reset_smoothing=True)
        )
        self.range_spin.valueChanged.connect(self._on_range_changed)
        for spin, key in (
            (self.threshold_spin, "near_threshold_mm"),
            (self.margin_spin, "background_margin_mm"),
            (self.min_zones_spin, "min_zones"),
        ):
            spin.valueChanged.connect(lambda value, k=key: self._on_setting(k, value))
        self.capture_button.clicked.connect(self.capture_background)
        self.clear_background_button.clicked.connect(self.clear_background)
        self.plot_zone_button.clicked.connect(self._plot_selected_zone)
        self.freeze_button.toggled.connect(self._on_freeze)
        self.save_image_button.clicked.connect(self.save_image)
        self.export_button.clicked.connect(self.export_csv)

        return scroll

    def _rebuild_map_guides(self):
        item = self.map_plot.getPlotItem()
        for guide in self.map_guides:
            item.removeItem(guide)
        self.map_guides.clear()

        reach = float(self.range_spin.value())
        half = math.radians(FOV_DEG / 2)
        guide_pen = pg.mkPen(theme.BORDER_STRONG, width=1)
        edge_pen = pg.mkPen(theme.TEXT_FAINT, width=1, style=Qt.PenStyle.DashLine)

        for sign in (-1, 1):
            edge = pg.PlotDataItem(
                [0, sign * reach * math.sin(half)], [0, reach * math.cos(half)], pen=edge_pen
            )
            self.map_guides.append(edge)

        step = 250 if reach <= 1000 else 500
        angles = np.linspace(-half, half, 48)
        for radius in range(step, int(reach) + 1, step):
            arc = pg.PlotDataItem(radius * np.sin(angles), radius * np.cos(angles), pen=guide_pen)
            label = pg.TextItem(f"{radius}", color=theme.TEXT_FAINT, anchor=(0.5, 1))
            label.setPos(0, radius)
            self.map_guides.extend([arc, label])

        for guide in self.map_guides:
            guide.setZValue(-10)
            item.addItem(guide)

        width = reach * math.sin(half) * 1.15
        item.getViewBox().setRange(xRange=(-width, width), yRange=(-reach * 0.04, reach * 1.04), padding=0)

    # ---------------- inputs ----------------

    def _on_setting(self, key: str, value, reset_smoothing: bool = False):
        self._save_setting(key, value)
        if reset_smoothing:
            self._smoothed = None
        self._process()
        self._render()

    def _on_range_changed(self, value: int):
        self._save_setting("range_mm", value)
        self._rebuild_map_guides()
        self._render()

    def _on_freeze(self, frozen: bool):
        self.freeze_button.setText("Frozen" if frozen else "Freeze")
        self._pending = False
        self._render()

    def notify_frame(self):
        """Called when a complete 8×8 frame has arrived."""
        if self.freeze_button.isChecked():
            return
        self._pending = True
        self._frame_times.append(time.monotonic())

    def set_device_name(self, name: str):
        self.device_label.setText(name)

    def reset(self):
        self._raw = self._smoothed = self._display = None
        self._objects = []
        self._pending = False
        self._frame_times.clear()
        self._capture_frames = None
        self._update_background_ui()
        self._render()

    def _tick(self):
        if self._pending:
            self._pending = False
            raw = frame_from_telemetry(self.telemetry_source())
            self._raw = raw
            if self._capture_frames is not None:
                self._capture_frames.append(raw)
                if len(self._capture_frames) >= BACKGROUND_FRAMES:
                    self._finish_capture()
                else:
                    self._update_background_ui()
            self._process(advance=True)
            self._render()
            self._was_stale = False
        elif not self._was_stale and self._is_stale():
            self._was_stale = True
            self._render()

    def _is_stale(self) -> bool:
        if self.freeze_button.isChecked():
            return False
        return not self._frame_times or time.monotonic() - self._frame_times[-1] > self.STALE_AFTER_S

    # ---------------- background ----------------

    def capture_background(self):
        self._capture_frames = []
        self._update_background_ui()

    def _finish_capture(self):
        self.background = median_background(self._capture_frames or [])
        self.background_time = datetime.now().strftime("%d %b %H:%M")
        self._capture_frames = None
        self._save_background()
        self._update_background_ui()

    def clear_background(self):
        self.background = None
        self.background_time = ""
        self._capture_frames = None
        self._save_background()
        self._update_background_ui()
        self._process()
        self._render()

    def _update_background_ui(self):
        capturing = self._capture_frames is not None
        if capturing:
            self.background_label.setText(
                f"Capturing background… {len(self._capture_frames)}/{BACKGROUND_FRAMES} frames"
            )
        elif self.background is not None:
            valid = int(np.sum(~np.isnan(self.background)))
            self.background_label.setText(
                f"Background captured {self.background_time} ({valid}/64 zones). "
                "Objects are things nearer than it."
            )
        else:
            self.background_label.setText(
                "No background — objects are anything nearer than the threshold."
            )
        self.capture_button.setEnabled(not capturing)
        self.capture_button.setText("Capturing…" if capturing else "Capture Background")
        self.clear_background_button.setEnabled(self.background is not None or capturing)
        has_background = self.background is not None
        self.threshold_spin.setEnabled(not has_background)
        self.margin_spin.setEnabled(has_background)

    # ---------------- processing ----------------

    def _process(self, advance: bool = False):
        if self._raw is None:
            self._display = None
            self._objects = []
            return

        oriented = orient(self._raw, self.mirror_check.isChecked(), self.flip_check.isChecked())
        alpha = SMOOTHING_LEVELS[self.smoothing_combo.currentIndex()][1]
        if advance or self._smoothed is None:
            self._smoothed = smooth_frames(self._smoothed, oriented, alpha)
        self._display = self._smoothed

        background = None
        if self.background is not None:
            background = orient(self.background, self.mirror_check.isChecked(), self.flip_check.isChecked())
        self._objects = detect_objects(
            self._display,
            background,
            near_threshold_mm=self.threshold_spin.value(),
            background_margin_mm=self.margin_spin.value(),
            min_zones=self.min_zones_spin.value(),
        )

    # ---------------- rendering ----------------

    def _render(self):
        stale = self._is_stale() and self._display is not None
        depth = self._display
        reach = float(self.range_spin.value())

        # ---- image ----
        if depth is None:
            self.image_item.setImage(np.full((GRID, GRID), np.nan), levels=(0, reach))
        else:
            if self.smooth_image_check.isChecked():
                filled = np.where(np.isnan(depth), reach, np.minimum(depth, reach))
                image = upsample(filled, self.UPSAMPLE_SIZE)
                invalid = np.repeat(np.repeat(np.isnan(depth), self.UPSAMPLE_SIZE // GRID, 0),
                                    self.UPSAMPLE_SIZE // GRID, 1)
                image[invalid] = np.nan
            else:
                image = depth
            # ImageItem indexes [x][y]; transpose so rows run down the screen.
            self.image_item.setImage(image.T, levels=(0, reach), autoLevels=False)
        self.image_item.setOpacity(0.45 if stale else 1.0)
        self.image_item.setRect(QRectF(0, 0, GRID, GRID))

        show_values = self.show_values_check.isChecked() and depth is not None
        if show_values:
            # Dark text on turbo's bright middle, light text on its ends.
            cell_colours = self.colormap.map(
                np.clip(np.nan_to_num(depth, nan=reach) / reach, 0, 1).ravel(), mode="byte"
            )
        for row in range(GRID):
            for col in range(GRID):
                text = self.value_items[row][col]
                if show_values and not np.isnan(depth[row, col]):
                    r, g, b = (int(v) for v in cell_colours[row * GRID + col][:3])
                    luminance = 0.2126 * r + 0.7152 * g + 0.0722 * b
                    text.setColor("#14161a" if luminance > 150 else "#ffffff")
                    text.setText(f"{depth[row, col]:.0f}")
                    text.show()
                else:
                    text.hide()

        self._render_object_boxes()
        self._render_selection()
        self._render_map(depth)
        self._render_objects_table()
        self._render_pills(depth, stale)
        self._render_zone_info()

    def _render_object_boxes(self):
        item = self.image_plot.getPlotItem()
        while len(self.object_rects) < len(self._objects):
            rect = QGraphicsRectItem()
            pen = QPen(QColor("#ffffff"), 2)
            pen.setCosmetic(True)
            pen.setStyle(Qt.PenStyle.DashLine)
            rect.setPen(pen)
            rect.setZValue(30)
            item.addItem(rect)
            label = pg.TextItem("", color="#14161a", fill=pg.mkBrush("#ffffff"), anchor=(0, 0))
            label.setZValue(31)
            item.addItem(label)
            self.object_rects.append(rect)
            self.object_labels.append(label)

        for index, (rect, label) in enumerate(zip(self.object_rects, self.object_labels)):
            if index < len(self._objects):
                obj = self._objects[index]
                r0, r1 = obj.row_range
                c0, c1 = obj.col_range
                rect.setRect(QRectF(c0 + 0.06, r0 + 0.06, c1 - c0 + 0.88, r1 - r0 + 0.88))
                label.setText(str(obj.index))
                label.setPos(c0 + 0.06, r0 + 0.06)
                rect.show()
                label.show()
            else:
                rect.hide()
                label.hide()

    def _render_selection(self):
        cell = self._selected
        if cell is None:
            self.selection_rect.hide()
            return
        row, col = self._raw_to_display(*cell)
        self.selection_rect.setRect(QRectF(col, row, 1, 1))
        self.selection_rect.show()

    def _render_map(self, depth: np.ndarray | None):
        map_item = self.map_plot.getPlotItem()
        if depth is None:
            self.map_points.setData([], [])
            self.map_objects.setData([], [])
        else:
            xs, ys, ds = project_zones(depth)
            reach = float(self.range_spin.value())
            colours = self.colormap.map(np.clip(ds / reach, 0, 1), mode="qcolor") if len(ds) else []
            self.map_points.setData(xs, ys, brush=[pg.mkBrush(c) for c in colours])
            self.map_objects.setData(
                [obj.x_mm for obj in self._objects], [obj.y_mm for obj in self._objects]
            )

        while len(self.map_object_labels) < len(self._objects):
            label = pg.TextItem("", color=theme.TEXT, anchor=(-0.28, 0.5))
            label.setZValue(25)
            map_item.addItem(label)
            self.map_object_labels.append(label)
        for index, label in enumerate(self.map_object_labels):
            if depth is not None and index < len(self._objects):
                obj = self._objects[index]
                label.setText(f"{obj.index}  {obj.nearest_mm:.0f} mm")
                label.setPos(obj.x_mm, obj.y_mm)
                label.show()
            else:
                label.hide()

    def _render_objects_table(self):
        self.objects_table.setRowCount(len(self._objects))
        for row, obj in enumerate(self._objects):
            cells = (
                str(obj.index),
                describe_angle(obj.bearing_deg, "R", "L"),
                f"{obj.nearest_mm:.0f}",
                f"{obj.width_mm:.0f}",
            )
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                item.setToolTip(
                    f"{len(obj.zones)} zones · mean {obj.mean_mm:.0f} mm · "
                    f"elevation {describe_angle(obj.elevation_deg, 'up', 'down')}"
                )
                self.objects_table.setItem(row, col, item)

    def _render_pills(self, depth: np.ndarray | None, stale: bool):
        if depth is None or np.all(np.isnan(depth)):
            self.nearest_pill.setText("NEAREST —")
            theme.set_pill_state(self.nearest_pill, "")
        else:
            self.nearest_pill.setText(f"NEAREST {np.nanmin(depth):.0f} mm")
            theme.set_pill_state(self.nearest_pill, "ok")

        valid = 0 if depth is None else int(np.sum(~np.isnan(depth)))
        self.valid_pill.setText(f"{valid}/64 ZONES")
        theme.set_pill_state(self.valid_pill, "ok" if valid else ("bad" if depth is not None else ""))

        count = len(self._objects)
        self.objects_pill.setText(f"{count} OBJECT{'S' if count != 1 else ''}")
        theme.set_pill_state(self.objects_pill, "busy" if count else "")

        if self.freeze_button.isChecked():
            self.fps_pill.setText("FROZEN")
            theme.set_pill_state(self.fps_pill, "busy")
        elif stale or len(self._frame_times) < 2:
            self.fps_pill.setText("NO DATA")
            theme.set_pill_state(self.fps_pill, "bad" if self._raw is not None else "")
        else:
            window = [t for t in self._frame_times if t >= self._frame_times[-1] - 2.0]
            span = window[-1] - window[0]
            fps = (len(window) - 1) / span if span > 0 else 0.0
            self.fps_pill.setText(f"{fps:.1f} FPS")
            theme.set_pill_state(self.fps_pill, "ok")

    def _render_zone_info(self):
        cell = self._hovered or self._selected
        self.plot_zone_button.setEnabled(self._selected is not None)
        if cell is None:
            self.zone_label.setText("Click a zone to select it; double-click to plot it.")
            return

        row, col = cell
        display_row, display_col = self._raw_to_display(row, col)
        value = None
        if self._display is not None:
            value = self._display[display_row, display_col]
        distance = "no target" if value is None or np.isnan(value) else f"{value:.0f} mm"
        prefix = "Hover" if cell == self._hovered and cell != self._selected else "Selected"
        self.zone_label.setText(
            f"{prefix} r{row}c{col}  ·  {distance}\n"
            f"{describe_angle(zone_bearing_deg(display_col), 'right', 'left')}, "
            f"{describe_angle(zone_elevation_deg(display_row), 'up', 'down')}"
        )

    # ---------------- zone picking ----------------

    def _raw_to_display(self, row: int, col: int) -> tuple[int, int]:
        if self.flip_check.isChecked():
            row = GRID - 1 - row
        if self.mirror_check.isChecked():
            col = GRID - 1 - col
        return row, col

    def _zone_at(self, scene_pos: QPointF) -> tuple[int, int] | None:
        view = self.image_plot.getPlotItem().getViewBox()
        if not view.sceneBoundingRect().contains(scene_pos):
            return None
        point = view.mapSceneToView(scene_pos)
        col, row = math.floor(point.x()), math.floor(point.y())
        if not (0 <= row < GRID and 0 <= col < GRID):
            return None
        # Display -> raw uses the same flip, since orientation is its own inverse.
        return self._raw_to_display(row, col)

    def _on_image_click(self, event):
        zone = self._zone_at(event.scenePos())
        if zone is None:
            return
        self._selected = zone
        if event.double():
            self._plot_selected_zone()
        self._render_selection()
        self._render_zone_info()

    def _on_image_hover(self, scene_pos: QPointF):
        zone = self._zone_at(scene_pos)
        if zone != self._hovered:
            self._hovered = zone
            self._render_zone_info()

    def _plot_selected_zone(self):
        if self._selected is not None:
            row, col = self._selected
            self.zone_plot_requested.emit(ZONE_KEYS[row][col])

    # ---------------- export ----------------

    def _data_directory(self) -> Path:
        directory = Path(__file__).resolve().parent / "Data"
        directory.mkdir(exist_ok=True)
        return directory

    def save_image(self):
        suggested = self._data_directory() / f"depth_{datetime.now():%Y-%m-%d_%H-%M-%S}.png"
        filename, _ = QFileDialog.getSaveFileName(self, "Save Depth Image", str(suggested), "PNG image (*.png)")
        if not filename:
            return
        left, right = self.image_plot.grab(), self.map_plot.grab()
        combined = QPixmap(left.width() + right.width(), max(left.height(), right.height()))
        combined.fill(QColor(theme.SURFACE))
        painter = QPainter(combined)
        painter.drawPixmap(0, 0, left)
        painter.drawPixmap(left.width(), 0, right)
        painter.end()
        combined.save(filename, "PNG")

    def export_csv(self):
        if self._raw is None:
            return
        suggested = self._data_directory() / f"depth_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        filename, _ = QFileDialog.getSaveFileName(self, "Export Depth Frame", str(suggested), "CSV (*.csv)")
        if not filename:
            return
        # Raw sensor orientation, so rows/columns match the tof.array.rXcY keys.
        with open(filename, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["row"] + [f"c{col}" for col in range(GRID)])
            for row in range(GRID):
                writer.writerow(
                    [f"r{row}"] + ["" if np.isnan(v) else f"{v:.0f}" for v in self._raw[row]]
                )
