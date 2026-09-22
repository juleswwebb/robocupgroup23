"""Group 23 live arena map and sensor-layout editor.

Adapted from the useful separation in Group 7's ArenaView: robot telemetry is
converted to local odometry and sensor rays on the desktop; autonomous motor
decisions remain in firmware behind the robot's safety gates.
"""

from __future__ import annotations

import math
from collections import deque

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QSpinBox, QVBoxLayout,
    QWidget,
)


def number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class ArenaModel:
    def __init__(self):
        self.left_mm_per_count = 0.095
        self.right_mm_per_count = 0.095
        self.track_width_mm = 300.0
        self.invert_left = False
        self.invert_right = False
        self.matrix_fov_deg = 60.0
        self.matrix_mirrored = False
        self.sensor_specs = []
        self.reset()

    def reset(self):
        self.x = self.y = 0.0
        self.theta = math.pi / 2
        self.trail = deque([(0.0, 0.0)], maxlen=3000)
        self.obstacles = deque(maxlen=8000)
        self.rays = []
        self.latest = {}
        self._frame = {}
        self._frame_time = None
        self._last_counts = None
        self._initial_heading = None
        self.distance_mm = 0.0

    def receive(self, name, value, timestamp):
        if timestamp is None:
            return False
        changed = self._frame_time is not None and timestamp != self._frame_time
        if changed:
            self.finish_frame()
        self._frame_time = timestamp
        self._frame[name] = value
        return changed

    def finish_frame(self):
        if not self._frame:
            return
        frame = self._frame
        self._frame = {}
        self.latest = frame.copy()
        self._integrate_pose(frame)
        self.rays = []
        for spec in self.sensor_specs:
            if not spec.get("enabled", True) or spec.get("kind") == "matrix":
                continue
            distance = number(frame.get(spec["signal"]))
            if distance is not None and 20 <= distance < 4000:
                self._observe(distance, spec)
        self._observe_matrix(frame)

    def _integrate_pose(self, frame):
        left = number(frame.get("encoder.0"))
        right = number(frame.get("encoder.1"))
        heading = number(frame.get("imu.heading"))
        if heading is not None:
            if self._initial_heading is None:
                self._initial_heading = heading
            clockwise = (heading - self._initial_heading + 180) % 360 - 180
            self.theta = math.pi / 2 - math.radians(clockwise)
        if left is None or right is None:
            return
        if self._last_counts is None:
            self._last_counts = (left, right)
            return
        dl, dr = left - self._last_counts[0], right - self._last_counts[1]
        self._last_counts = (left, right)
        if abs(dl) > 100000 or abs(dr) > 100000:
            return
        if self.invert_left:
            dl = -dl
        if self.invert_right:
            dr = -dr
        dl *= self.left_mm_per_count
        dr *= self.right_mm_per_count
        ds = (dl + dr) / 2.0
        if heading is None and self.track_width_mm > 0:
            self.theta += (dr - dl) / self.track_width_mm
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self.distance_mm += abs(ds)
        if math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 8:
            self.trail.append((self.x, self.y))

    def _origin_and_angle(self, spec, relative_angle=None):
        right = float(spec.get("x", 0))
        forward = float(spec.get("y", 0))
        ox = self.x + forward * math.cos(self.theta) + right * math.cos(self.theta - math.pi / 2)
        oy = self.y + forward * math.sin(self.theta) + right * math.sin(self.theta - math.pi / 2)
        angle = self.theta - math.radians(
            float(spec.get("angle", 0)) if relative_angle is None else relative_angle
        )
        return ox, oy, angle

    def _observe(self, distance, spec, relative_angle=None, source=None):
        ox, oy, angle = self._origin_and_angle(spec, relative_angle)
        ex = ox + distance * math.cos(angle)
        ey = oy + distance * math.sin(angle)
        self.rays.append((ox, oy, ex, ey, source or spec.get("name", "sensor")))
        self.obstacles.append((ex, ey, source or spec.get("name", "sensor")))

    def _observe_matrix(self, frame):
        spec = next((s for s in self.sensor_specs if s.get("kind") == "matrix"), None)
        if not spec or not spec.get("enabled", True):
            return
        side = -1 if self.matrix_mirrored else 1
        for col in range(8):
            values = []
            for row in range(6):
                value = number(frame.get(f"tof.array.r{row}c{col}"))
                if value is not None and 20 <= value < 4000:
                    values.append(value)
            if len(values) < 2:
                continue
            values.sort()
            distance = values[len(values) // 2]
            angle = float(spec.get("angle", 0)) + side * (3.5 - col) * self.matrix_fov_deg / 8
            self._observe(distance, spec, angle, "8x8")


class ArenaCanvas(QWidget):
    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.zoom = 0.22
        self.pan = QPointF()
        self._drag = None
        self.setMinimumSize(600, 500)

    def wheelEvent(self, event):
        self.zoom = max(0.03, min(2.0, self.zoom * (1.15 if event.angleDelta().y() > 0 else 1 / 1.15)))
        self.update()

    def mousePressEvent(self, event):
        self._drag = event.position()

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.position() - self._drag
            self.pan += delta
            self._drag = event.position()
            self.update()

    def mouseReleaseEvent(self, _event):
        self._drag = None

    def _point(self, x, y):
        return QPointF(self.width() / 2 + self.pan.x() + x * self.zoom,
                       self.height() / 2 + self.pan.y() - y * self.zoom)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#111318"))
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        spacing = max(10, int(250 * self.zoom))
        p.setPen(QPen(QColor("#252a33"), 1))
        cx, cy = int(self.width()/2 + self.pan.x()), int(self.height()/2 + self.pan.y())
        for x in range(cx % spacing, self.width(), spacing): p.drawLine(x, 0, x, self.height())
        for y in range(cy % spacing, self.height(), spacing): p.drawLine(0, y, self.width(), y)

        p.setPen(QPen(QColor("#3d8bfd"), 2))
        if len(self.model.trail) > 1:
            points = [self._point(x, y) for x, y in self.model.trail]
            for a, b in zip(points, points[1:]): p.drawLine(a, b)

        p.setPen(QPen(QColor(242, 84, 75, 150), 3))
        for x, y, _source in self.model.obstacles:
            q = self._point(x, y); p.drawEllipse(q, 2.5, 2.5)

        p.setPen(QPen(QColor(47, 191, 113, 150), 1))
        for ox, oy, ex, ey, _source in self.model.rays:
            p.drawLine(self._point(ox, oy), self._point(ex, ey))

        centre = self._point(self.model.x, self.model.y)
        angle = self.model.theta
        forward = QPointF(math.cos(angle), -math.sin(angle))
        right = QPointF(math.cos(angle - math.pi/2), -math.sin(angle - math.pi/2))
        half_l, half_w = 90 * self.zoom, 75 * self.zoom
        poly = QPolygonF([
            centre + forward*half_l + right*half_w,
            centre + forward*half_l - right*half_w,
            centre - forward*half_l - right*half_w,
            centre - forward*half_l + right*half_w,
        ])
        p.setBrush(QColor("#3d8bfd")); p.setPen(QPen(QColor("#dbeafe"), 2)); p.drawPolygon(poly)
        p.setPen(QPen(QColor("white"), 3)); p.drawLine(centre, centre + forward*(half_l+18))


class ArenaView(QWidget):
    command_requested = pyqtSignal(str, dict)
    parameter_requested = pyqtSignal(str, object)

    def __init__(self, settings, hardware_map, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.hardware_map = hardware_map
        self.model = ArenaModel()
        self._load_calibration()
        self._build_specs()

        root = QHBoxLayout(self)
        left = QVBoxLayout()
        controls = QHBoxLayout()
        self.run = QPushButton("START NAVIGATION")
        self.stop = QPushButton("STOP NAVIGATION")
        self.run.setObjectName("primaryButton")
        self.stop.setObjectName("dangerButton")
        self.run.clicked.connect(lambda: self.command_requested.emit("navigation_set", {"enabled": True}))
        self.stop.clicked.connect(lambda: self.command_requested.emit("navigation_set", {"enabled": False}))
        reset = QPushButton("Reset map origin")
        reset.clicked.connect(self.reset_map)
        controls.addWidget(self.run); controls.addWidget(self.stop); controls.addWidget(reset); controls.addStretch()
        self.status = QLabel("Waiting for encoder, IMU and range telemetry")
        left.addLayout(controls); left.addWidget(self.status)
        self.canvas = ArenaCanvas(self.model)
        left.addWidget(self.canvas, 1)
        root.addLayout(left, 1)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setMaximumWidth(390)
        side = QWidget(); form = QFormLayout(side)
        note = QLabel("Local map only — not absolute arena localisation. Calibrate encoder scale, direction and wheel track before trusting pose or autonomous motion.")
        note.setWordWrap(True); form.addRow(note)
        self.left_scale = self._spin(form, "Left mm/count", "left_scale", .095, 0, 10, 5)
        self.right_scale = self._spin(form, "Right mm/count", "right_scale", .095, 0, 10, 5)
        self.track = self._spin(form, "Wheel track (mm)", "track", 300, 1, 2000, 1)
        self.invert_left = QCheckBox("Invert left encoder")
        self.invert_right = QCheckBox("Invert right encoder")
        self.invert_left.setChecked(settings.value("arena/invert_left", False, type=bool))
        self.invert_right.setChecked(settings.value("arena/invert_right", False, type=bool))
        self.invert_left.toggled.connect(self._calibration_changed)
        self.invert_right.toggled.connect(self._calibration_changed)
        form.addRow(self.invert_left); form.addRow(self.invert_right)
        self.matrix_fov = self._spin(form, "8x8 horizontal FOV (°)", "matrix_fov", 60, 20, 120, 1)
        self.matrix_mirror = QCheckBox("Mirror 8x8 horizontally")
        self.matrix_mirror.setChecked(settings.value("arena/matrix_mirror", False, type=bool))
        self.matrix_mirror.toggled.connect(self._calibration_changed); form.addRow(self.matrix_mirror)

        nav = QGroupBox("Navigation tuning"); nav_form = QFormLayout(nav)
        self.nav_speed = self._int_spin(nav_form, "Forward speed (%)", 30, 5, 60)
        self.nav_turn = self._int_spin(nav_form, "Turn speed (%)", 25, 5, 60)
        self.nav_stop = self._int_spin(nav_form, "Obstacle distance (mm)", 300, 100, 1500)
        apply_nav = QPushButton("Apply to robot")
        apply_nav.clicked.connect(self._apply_navigation)
        nav_form.addRow(apply_nav); form.addRow(nav)

        sensor_group = QGroupBox("Sensor placement (robot coordinates)")
        sensor_form = QVBoxLayout(sensor_group)
        for spec in self.model.sensor_specs:
            box = QGroupBox(f"{spec['name']}  ·  {spec['signal']}")
            grid = QGridLayout(box)
            enabled = QCheckBox("Enabled"); enabled.setChecked(spec["enabled"])
            x = self._sensor_spin(spec, "x", -1000, 1000)
            y = self._sensor_spin(spec, "y", -1000, 1000)
            angle = self._sensor_spin(spec, "angle", -180, 180)
            enabled.toggled.connect(lambda value, s=spec: self._sensor_changed(s, "enabled", value))
            grid.addWidget(enabled, 0, 0, 1, 2)
            for row, (label, widget) in enumerate((("Right x (mm)", x), ("Forward y (mm)", y), ("Angle (°)", angle)), 1):
                grid.addWidget(QLabel(label), row, 0); grid.addWidget(widget, row, 1)
            sensor_form.addWidget(box)
        sensor_form.addStretch(); form.addRow(sensor_group)
        scroll.setWidget(side); root.addWidget(scroll)
        self._calibration_changed()

    def _spin(self, form, label, key, default, minimum, maximum, decimals):
        w = QDoubleSpinBox(); w.setRange(minimum, maximum); w.setDecimals(decimals)
        w.setValue(float(self.settings.value("arena/" + key, default))); w.valueChanged.connect(self._calibration_changed)
        form.addRow(label, w); return w

    @staticmethod
    def _int_spin(form, label, default, minimum, maximum):
        w = QSpinBox(); w.setRange(minimum, maximum); w.setValue(default); form.addRow(label, w); return w

    def _load_calibration(self):
        self.model.left_mm_per_count = float(self.settings.value("arena/left_scale", .095))
        self.model.right_mm_per_count = float(self.settings.value("arena/right_scale", .095))
        self.model.track_width_mm = float(self.settings.value("arena/track", 300))

    def _build_specs(self):
        specs = []
        point_devices = [d for d in self.hardware_map.devices if d.port.startswith("xshut")]
        for index, device in enumerate(point_devices):
            signal = "tof." + device.port
            lower = device.name.lower()
            x_default = -90 if "left" in lower else (90 if "right" in lower else 0)
            y_default = 140 if "front" in lower or "top" in lower else 0
            angle_default = -45 if device.port == "xshut0" else (45 if device.port == "xshut1" else 0)
            key = device.port
            specs.append({"name": device.name, "signal": signal, "kind": "point",
                          "x": float(self.settings.value(f"arena/sensors/{key}/x", x_default)),
                          "y": float(self.settings.value(f"arena/sensors/{key}/y", y_default)),
                          "angle": float(self.settings.value(f"arena/sensors/{key}/angle", angle_default)),
                          "enabled": self.settings.value(f"arena/sensors/{key}/enabled", "not in use" not in lower, type=bool),
                          "key": key})
        specs.append({"name": "8x8 TOF", "signal": "tof.array", "kind": "matrix", "key": "matrix",
                      "x": float(self.settings.value("arena/sensors/matrix/x", 0)),
                      "y": float(self.settings.value("arena/sensors/matrix/y", 150)),
                      "angle": float(self.settings.value("arena/sensors/matrix/angle", 0)),
                      "enabled": self.settings.value("arena/sensors/matrix/enabled", True, type=bool)})
        self.model.sensor_specs = specs

    def _sensor_spin(self, spec, field, minimum, maximum):
        w = QDoubleSpinBox(); w.setRange(minimum, maximum); w.setDecimals(1); w.setValue(spec[field])
        w.valueChanged.connect(lambda value, s=spec, f=field: self._sensor_changed(s, f, value)); return w

    def _sensor_changed(self, spec, field, value):
        spec[field] = value
        self.settings.setValue(f"arena/sensors/{spec['key']}/{field}", value)
        self.reset_map()

    def _calibration_changed(self, *_args):
        if not hasattr(self, "left_scale"):
            return
        m = self.model
        m.left_mm_per_count = self.left_scale.value(); m.right_mm_per_count = self.right_scale.value()
        m.track_width_mm = self.track.value(); m.invert_left = self.invert_left.isChecked(); m.invert_right = self.invert_right.isChecked()
        m.matrix_fov_deg = self.matrix_fov.value(); m.matrix_mirrored = self.matrix_mirror.isChecked()
        for key, value in (("left_scale", m.left_mm_per_count), ("right_scale", m.right_mm_per_count),
                           ("track", m.track_width_mm), ("invert_left", m.invert_left),
                           ("invert_right", m.invert_right), ("matrix_fov", m.matrix_fov_deg),
                           ("matrix_mirror", m.matrix_mirrored)):
            self.settings.setValue("arena/" + key, value)

    def _apply_navigation(self):
        self.parameter_requested.emit("navigation.speed_percent", self.nav_speed.value())
        self.parameter_requested.emit("navigation.turn_percent", self.nav_turn.value())
        self.parameter_requested.emit("navigation.front_stop_mm", self.nav_stop.value())

    def reset_map(self):
        self.model.reset(); self.canvas.update()

    def receive_telemetry(self, name, value, timestamp):
        if self.model.receive(name, value, timestamp):
            self._refresh()

    def _refresh(self):
        m = self.model; nav = m.latest.get("navigation.state", "IDLE")
        self.status.setText(
            f"Pose x={m.x:.0f} mm, y={m.y:.0f} mm, heading={math.degrees(m.theta)%360:.1f}°  ·  "
            f"travel={m.distance_mm:.0f} mm  ·  map points={len(m.obstacles)}  ·  navigation={nav}"
        )
        self.canvas.update()
