"""Group 23 live arena map and sensor-layout editor.

Adapted from the useful separation in Group 7's ArenaView: robot telemetry is
converted to local odometry and sensor rays on the desktop; autonomous motor
decisions remain in firmware behind the robot's safety gates.
"""

from __future__ import annotations

import math
import json
from collections import deque

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QSpinBox, QVBoxLayout,
    QWidget,
)
from arena_planner import plan_route
SENSOR_COLOURS = {
    "point": "#f5d567",       # VL53 ToF
    "matrix": "#b88cff",      # 8x8 ToF
    "ultrasonic": "#4dd6df",  # HC-SR04
    "marker": "#a1d9ff",      # other placed sensors
}


def number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


# Two measured forward runs: 1805 + 1830 mm, with encoder 0 on the left
# and encoder 1 on the right. These are provisional odometry scales.
DEFAULT_LEFT_MM_PER_COUNT = 3635.0 / 41153.0
DEFAULT_RIGHT_MM_PER_COUNT = 3635.0 / 42224.0


class ArenaModel:
    def __init__(self):
        # Provisional chassis-distance calibration from two measured forward
        # runs (1805 mm / 20462,-20966 and 1830 mm / 20691,-21258).
        # These are odometry defaults, not a motor-speed correction.
        self.left_mm_per_count = DEFAULT_LEFT_MM_PER_COUNT
        self.right_mm_per_count = DEFAULT_RIGHT_MM_PER_COUNT
        self.track_width_mm = 300.0
        self.invert_left = False
        self.invert_right = True
        self.matrix_fov_deg = 60.0
        self.matrix_mirrored = False
        self.sensor_specs = []
        self.known_walls = []
        self.known_obstacles = []
        self.known_weights = []
        self.detected_weights = []
        self.weight_votes = {}
        self.goal = None
        self.route = []
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
        self.detected_weights.clear()
        self.weight_votes.clear()

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
            if not spec.get("enabled", True) or spec.get("kind") not in ("point", "ultrasonic"):
                continue
            distance = number(frame.get(spec["signal"]))
            if distance is not None and 20 <= distance < 4000:
                self._observe(distance, spec)
        self._observe_matrix(frame)
        self._detect_weights(frame)

    def _detect_weights(self, frame):
        # Group 7's top/bottom depth-gap idea, mapped to Group 23's wiring.
        # Require three consistent frames before putting a candidate on-map.
        for top_key, bottom_key in (("xshut6", "xshut5"), ("xshut3", "xshut4")):
            top = number(frame.get(f"tof.{top_key}"))
            bottom = number(frame.get(f"tof.{bottom_key}"))
            if top is None or bottom is None or not (50 <= bottom <= 2000) or not (50 <= top < 4000):
                self.weight_votes[top_key] = 0
                continue
            if top - bottom < 150:
                self.weight_votes[top_key] = 0
                continue
            self.weight_votes[top_key] = min(3, self.weight_votes.get(top_key, 0) + 1)
            if self.weight_votes[top_key] < 3:
                continue
            spec = next((s for s in self.sensor_specs if s["key"] == bottom_key), None)
            if spec is None:
                continue
            ox, oy, angle = self._origin_and_angle(spec)
            point = (ox + bottom * math.cos(angle), oy + bottom * math.sin(angle))
            if all(math.hypot(point[0] - x, point[1] - y) > 150 for x, y in self.detected_weights):
                self.detected_weights.append(point)

    def plan(self, radius_mm=150, margin_mm=70):
        self.route = [] if self.goal is None else plan_route(
            (self.x, self.y), self.goal, self.known_walls, self.known_obstacles,
            robot_radius_mm=radius_mm, margin_mm=margin_mm,
        )
        return self.route

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
        self.rays.append((ox, oy, ex, ey, source or spec.get("name", "sensor"),
                          spec["kind"]))
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
    map_clicked = pyqtSignal(float, float)
    sensor_moved = pyqtSignal(str, float, float)

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.zoom = 0.22
        self.pan = QPointF()
        self._drag = None
        self._sensor_drag = None
        self.selected_sensor_key = None
        self.edit_mode = "Pan / inspect"
        self.setMinimumSize(600, 500)

    def wheelEvent(self, event):
        self.zoom = max(0.03, min(2.0, self.zoom * (1.15 if event.angleDelta().y() > 0 else 1 / 1.15)))
        self.update()

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        if self.edit_mode == "Place sensor":
            candidates = []
            for spec in self.model.sensor_specs:
                if spec.get("enabled", True):
                    ox, oy, _ = self.model._origin_and_angle(spec)
                    candidates.append(((self._point(ox, oy) - event.position()).manhattanLength(), spec["key"]))
            if candidates:
                selected = next(
                    (candidate for candidate in candidates
                     if candidate[1] == self.selected_sensor_key and candidate[0] < 16),
                    None,
                )
                distance, key = selected or min(candidates)
                if distance < 16:
                    self._sensor_drag = key
            return
        if self.edit_mode == "Pan / inspect":
            self._drag = event.position()
        else:
            x, y = self._world(event.position())
            self.map_clicked.emit(x, y)

    def mouseMoveEvent(self, event):
        if self._sensor_drag is not None:
            x, y = self._world(event.position())
            dx, dy = x - self.model.x, y - self.model.y
            theta = self.model.theta
            forward = dx * math.cos(theta) + dy * math.sin(theta)
            right = dx * math.cos(theta - math.pi/2) + dy * math.sin(theta - math.pi/2)
            self.sensor_moved.emit(self._sensor_drag, right, forward)
            self.update()
            return
        if self._drag is not None:
            delta = event.position() - self._drag
            self.pan += delta
            self._drag = event.position()
            self.update()

    def mouseReleaseEvent(self, _event):
        self._drag = None
        self._sensor_drag = None

    def _world(self, point):
        return ((point.x() - self.width()/2 - self.pan.x()) / self.zoom,
                (self.height()/2 + self.pan.y() - point.y()) / self.zoom)

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

        for ox, oy, ex, ey, _source, kind in self.model.rays:
            colour = QColor(SENSOR_COLOURS[kind])
            colour.setAlpha(170)
            p.setPen(QPen(colour, 1))
            p.drawLine(self._point(ox, oy), self._point(ex, ey))

        p.setPen(QPen(QColor("#f2a84b"), max(2, int(self.zoom * 10))))
        for ax, ay, bx, by in self.model.known_walls:
            p.drawLine(self._point(ax, ay), self._point(bx, by))
        p.setPen(QPen(QColor("#f2544b"), 2)); p.setBrush(QColor(242, 84, 75, 75))
        for x, y, radius in self.model.known_obstacles:
            p.drawEllipse(self._point(x, y), radius*self.zoom, radius*self.zoom)
        p.setPen(QPen(QColor("#bb8bf4"), 2)); p.setBrush(QColor("#bb8bf4"))
        for x, y in self.model.known_weights + self.model.detected_weights:
            p.drawEllipse(self._point(x, y), 6, 6)
        if self.model.route:
            p.setPen(QPen(QColor("#4dd6df"), 3))
            for a, b in zip(self.model.route, self.model.route[1:]):
                p.drawLine(self._point(*a), self._point(*b))
        if self.model.goal is not None:
            p.setPen(QPen(QColor("#4dd6df"), 2)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(self._point(*self.model.goal), 9, 9)

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
        # Draw markers last so the robot silhouette cannot hide sensors in the
        # middle of its footprint (especially IMU and optical flow).
        for spec in self.model.sensor_specs:
            if not spec.get("enabled", True): continue
            ox, oy, sensor_angle = self.model._origin_and_angle(spec)
            origin = self._point(ox, oy)
            marker = spec.get("kind") == "marker"
            colour = QColor(SENSOR_COLOURS[spec["kind"]])
            p.setPen(QPen(colour, 2))
            p.setBrush(colour)
            p.drawEllipse(origin, 5, 5)
            # Height does not move a sensor in this top-down map. Rings make
            # co-located sensors distinguishable without falsifying x/y.
            height = float(spec.get("height_mm", 0))
            if height > 0:
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(origin, 7 + min(8, height / 100),
                              7 + min(8, height / 100))
            if not marker:
                p.drawLine(origin, self._point(ox + 55*math.cos(sensor_angle),
                                               oy + 55*math.sin(sensor_angle)))
            if self.edit_mode == "Place sensor":
                p.drawText(origin + QPointF(9, -9 - min(30, height / 50)),
                           f"{spec['name']}  z={height:g} mm")


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
        self._load_known_map()
        self._wall_start = None
        self._encoder_calibration_start = None

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
        self.canvas.map_clicked.connect(self._map_clicked)
        self.canvas.sensor_moved.connect(self._move_sensor)
        editor = QHBoxLayout()
        editor.addWidget(QLabel("Map tool"))
        self.map_tool = QComboBox()
        self.map_tool.addItems(("Pan / inspect", "Place sensor", "Place wall", "Place obstacle", "Place weight", "Set goal"))
        self.map_tool.currentTextChanged.connect(self._tool_changed)
        editor.addWidget(self.map_tool)
        plan_button = QPushButton("Plan route")
        plan_button.clicked.connect(self._plan_route)
        editor.addWidget(plan_button)
        undo_button = QPushButton("Undo mark")
        undo_button.clicked.connect(self._undo_mark)
        editor.addWidget(undo_button)
        editor.addStretch()
        left.addLayout(editor)
        hint = QLabel("Click to place marks; walls need two clicks. Sensor markers: gold = VL53, purple = 8×8 ToF, cyan = ultrasonic. Rings and z labels show height above ground. Select a sensor below before dragging co-located markers. Cyan route is preview only.")
        hint.setWordWrap(True)
        left.addWidget(hint)
        left.addWidget(self.canvas, 1)
        root.addLayout(left, 1)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setMaximumWidth(390)
        side = QWidget(); form = QFormLayout(side)
        note = QLabel("Local map only — not absolute arena localisation. Calibrate encoder scale, direction and wheel track before trusting pose or autonomous motion.")
        note.setWordWrap(True); form.addRow(note)
        self.left_scale = self._spin(form, "Left mm/count", "left_scale", self.model.left_mm_per_count, 0, 10, 5)
        self.right_scale = self._spin(form, "Right mm/count", "right_scale", self.model.right_mm_per_count, 0, 10, 5)
        self.track = self._spin(form, "Wheel track (mm)", "track", 300, 1, 2000, 1)
        self.invert_left = QCheckBox("Invert left encoder")
        self.invert_right = QCheckBox("Invert right encoder")
        self.invert_left.setChecked(settings.value("arena/invert_left", False, type=bool))
        self.invert_right.setChecked(settings.value("arena/invert_right", True, type=bool))
        self.invert_left.toggled.connect(self._calibration_changed)
        self.invert_right.toggled.connect(self._calibration_changed)
        form.addRow(self.invert_left); form.addRow(self.invert_right)
        self.matrix_fov = self._spin(form, "8x8 horizontal FOV (°)", "matrix_fov", 60, 20, 120, 1)
        self.matrix_mirror = QCheckBox("Mirror 8x8 horizontally")
        self.matrix_mirror.setChecked(settings.value("arena/matrix_mirror", False, type=bool))
        self.matrix_mirror.toggled.connect(self._calibration_changed); form.addRow(self.matrix_mirror)

        encoder_cal = QGroupBox("Encoder distance calibration")
        encoder_form = QFormLayout(encoder_cal)
        self.calibration_distance = QDoubleSpinBox()
        self.calibration_distance.setRange(50, 5000)
        self.calibration_distance.setValue(1000)
        self.calibration_distance.setSuffix(" mm")
        encoder_form.addRow("Measured forward travel", self.calibration_distance)
        use_runs = QPushButton("Use 2 measured runs (1805 + 1830 mm)")
        use_runs.setToolTip("Applies 0.08833 / 0.08609 mm per count and reverses encoder 1 for forward travel")
        use_runs.clicked.connect(self._apply_measured_encoder_runs)
        encoder_form.addRow(use_runs)
        mark_start = QPushButton("1 · Capture start counts")
        mark_start.clicked.connect(self._capture_encoder_start)
        encoder_form.addRow(mark_start)
        calculate = QPushButton("2 · Calculate mm/count")
        calculate.clicked.connect(self._calculate_encoder_scale)
        encoder_form.addRow(calculate)
        self.encoder_cal_status = QLabel("With motors OFF, mark start; roll straight forward the measured distance; calculate.")
        self.encoder_cal_status.setWordWrap(True)
        encoder_form.addRow(self.encoder_cal_status)
        form.addRow(encoder_cal)

        nav = QGroupBox("Navigation tuning"); nav_form = QFormLayout(nav)
        self.nav_speed = self._int_spin(nav_form, "Forward speed (%)", 30, 5, 60)
        self.nav_turn = self._int_spin(nav_form, "Turn speed (%)", 25, 5, 60)
        self.nav_stop = self._int_spin(nav_form, "Obstacle distance (mm)", 300, 100, 1500)
        apply_nav = QPushButton("Apply to robot")
        apply_nav.clicked.connect(self._apply_navigation)
        nav_form.addRow(apply_nav); form.addRow(nav)

        planner = QGroupBox("Arena route preview")
        planner_form = QFormLayout(planner)
        self.robot_radius = self._int_spin(planner_form, "Robot radius (mm)", 150, 50, 500)
        self.route_margin = self._int_spin(planner_form, "Extra clearance (mm)", 70, 0, 300)
        planner_form.addRow(QLabel("Orange walls / red obstacles are operator-placed; purple weights are manual or 3-frame TOF gap candidates. Preview does not command motors."))
        form.addRow(planner)

        sensor_group = QGroupBox("Sensor placement (robot coordinates)")
        sensor_form = QVBoxLayout(sensor_group)
        self.sensor_form = sensor_form
        self.sensor_controls = {}
        self.sensor_boxes = {}
        self.sensor_selector = QComboBox()
        self.sensor_selector.setToolTip("Choose which sensor to drag when markers overlap")
        self.sensor_selector.currentIndexChanged.connect(self._selected_sensor_changed)
        sensor_form.addWidget(QLabel("Drag this sensor on the map"))
        sensor_form.addWidget(self.sensor_selector)
        self._populate_sensor_controls()
        form.addRow(sensor_group)
        scroll.setWidget(side); root.addWidget(scroll)
        self._calibration_changed()

    def _load_known_map(self):
        try:
            data = json.loads(self.settings.value("arena/known_map", "{}"))
            self.model.known_walls = [tuple(map(float, row)) for row in data.get("walls", []) if len(row) == 4][:200]
            self.model.known_obstacles = [tuple(map(float, row)) for row in data.get("obstacles", []) if len(row) == 3][:200]
            self.model.known_weights = [tuple(map(float, row)) for row in data.get("weights", []) if len(row) == 2][:200]
            goal = data.get("goal")
            self.model.goal = tuple(map(float, goal)) if goal and len(goal) == 2 else None
        except (TypeError, ValueError, AttributeError):
            self.model.known_walls = []
            self.model.known_obstacles = []
            self.model.known_weights = []
            self.model.goal = None

    def _save_known_map(self):
        self.settings.setValue("arena/known_map", json.dumps({
            "walls": self.model.known_walls,
            "obstacles": self.model.known_obstacles,
            "weights": self.model.known_weights,
            "goal": self.model.goal,
        }))

    def _tool_changed(self, mode):
        self.canvas.edit_mode = mode
        self._wall_start = None
        self.canvas.update()

    def _map_clicked(self, x, y):
        mode = self.map_tool.currentText()
        x, y = round(x / 50) * 50, round(y / 50) * 50
        if mode == "Place wall":
            if self._wall_start is None:
                self._wall_start = (x, y)
                self.status.setText(f"Wall start ({x}, {y}) mm — click endpoint")
                return
            if (x, y) != self._wall_start:
                self.model.known_walls.append((*self._wall_start, x, y))
            self._wall_start = None
        elif mode == "Place obstacle":
            self.model.known_obstacles.append((x, y, 100))
        elif mode == "Place weight":
            self.model.known_weights.append((x, y))
        elif mode == "Set goal":
            self.model.goal = (x, y)
        else:
            return
        self.model.route = []
        self._save_known_map()
        self.canvas.update()

    def _undo_mark(self):
        self._wall_start = None
        mode = self.map_tool.currentText()
        collection = {"Place wall": self.model.known_walls,
                      "Place obstacle": self.model.known_obstacles,
                      "Place weight": self.model.known_weights}.get(mode)
        if collection:
            collection.pop()
        elif mode == "Set goal":
            self.model.goal = None
        self.model.route = []
        self._save_known_map()
        self.canvas.update()

    def _plan_route(self):
        if self.model.goal is None:
            self.status.setText("Set a goal on the map first")
            return
        try:
            route = self.model.plan(self.robot_radius.value(), self.route_margin.value())
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.status.setText(f"Route preview: {len(route)} waypoints" if route else "No collision-clear route found")
        self.canvas.update()

    def _move_sensor(self, key, right, forward):
        controls = self.sensor_controls.get(key)
        if not controls:
            return
        controls[0].setValue(max(-1000, min(1000, right)))
        controls[1].setValue(max(-1000, min(1000, forward)))

    def _spin(self, form, label, key, default, minimum, maximum, decimals):
        w = QDoubleSpinBox(); w.setRange(minimum, maximum); w.setDecimals(decimals)
        w.setValue(float(self.settings.value("arena/" + key, default))); w.valueChanged.connect(self._calibration_changed)
        form.addRow(label, w); return w

    @staticmethod
    def _int_spin(form, label, default, minimum, maximum):
        w = QSpinBox(); w.setRange(minimum, maximum); w.setValue(default); form.addRow(label, w); return w

    def _load_calibration(self):
        # Before these measured defaults, the UI saved 0.095 for both wheels
        # and a non-inverted right encoder. Upgrade only that exact untouched
        # legacy combination; preserve any user-calibrated values or polarity.
        if int(self.settings.value("arena/encoder_calibration_version", 0)) < 1:
            old_left = self.settings.value("arena/left_scale", None)
            old_right = self.settings.value("arena/right_scale", None)
            try:
                legacy_scales = (old_left is not None and old_right is not None
                                 and abs(float(old_left) - 0.095) < 1e-8
                                 and abs(float(old_right) - 0.095) < 1e-8)
            except (TypeError, ValueError):
                legacy_scales = False
            if (legacy_scales
                    and not self.settings.value("arena/invert_left", False, type=bool)
                    and not self.settings.value("arena/invert_right", False, type=bool)):
                self.settings.setValue("arena/left_scale", DEFAULT_LEFT_MM_PER_COUNT)
                self.settings.setValue("arena/right_scale", DEFAULT_RIGHT_MM_PER_COUNT)
                self.settings.setValue("arena/invert_right", True)
            self.settings.setValue("arena/encoder_calibration_version", 1)
        self.model.left_mm_per_count = float(self.settings.value("arena/left_scale", self.model.left_mm_per_count))
        self.model.right_mm_per_count = float(self.settings.value("arena/right_scale", self.model.right_mm_per_count))
        self.model.track_width_mm = float(self.settings.value("arena/track", 300))
        self.model.invert_left = self.settings.value("arena/invert_left", False, type=bool)
        self.model.invert_right = self.settings.value("arena/invert_right", True, type=bool)

    def _build_specs(self):
        specs = []
        point_devices = [d for d in self.hardware_map.devices
                         if d.port.startswith("xshut") and d.kind in ("vl53l0x", "vl53l1x")]
        for device in point_devices:
            signal = "tof." + device.port
            lower = device.name.lower()
            x_default = -90 if "left" in lower else (90 if "right" in lower else 0)
            y_default = 140 if "front" in lower or "top" in lower else 0
            angle_default = -45 if device.port == "xshut0" else (45 if device.port == "xshut1" else 0)
            key = device.port
            specs.append(self._placement_spec(key, device.name, signal, "point",
                                              x_default, y_default, angle_default,
                                              "not in use" not in lower))

        matrix_device = self.hardware_map.device_for_signal("tof.8x8")
        if matrix_device and matrix_device.kind == "tof_8x8":
            specs.append(self._placement_spec("matrix", matrix_device.name, "tof.array",
                                              "matrix", 0, 150, 0))

        for index, x_default in ((0, -110), (1, 110)):
            signal = f"ultrasonic.{index}"
            device = self.hardware_map.device_for_signal(signal)
            if device is None or device.kind != "ultrasonic":
                continue
            key = f"ultrasonic{index}"
            specs.append(self._placement_spec(key, device.name, signal,
                                              "ultrasonic", x_default, 120, 0))

        # The IMU and encoders feed pose integration above, but their board
        # locations are not useful range origins and should not clutter the
        # robot-placement editor. Likewise inactive colour/IR hardware.
        others = (
            ("flow", "flow.dx", "optical_flow", 0, -40, 0),
            ("inductive", "inductive.detected", "inductive", 0, 130, 0),
        )
        for key, signal, expected_kind, x_default, y_default, angle_default in others:
            device = self.hardware_map.device_for_signal(signal)
            if device and device.kind == expected_kind:
                specs.append(self._placement_spec(key, device.name, signal, "marker",
                                                  x_default, y_default, angle_default))
        self.model.sensor_specs = specs

    def _placement_spec(self, key, name, signal, kind, x, y, angle, enabled=True):
        prefix = f"arena/sensors/{key}/"
        return {
            "key": key, "name": name, "signal": signal, "kind": kind,
            "x": float(self.settings.value(prefix + "x", x)),
            "y": float(self.settings.value(prefix + "y", y)),
            "angle": float(self.settings.value(prefix + "angle", angle)),
            "height_mm": float(self.settings.value(prefix + "height_mm", 0)),
            "enabled": self.settings.value(prefix + "enabled", enabled, type=bool),
        }

    def _populate_sensor_controls(self):
        previous_key = self.sensor_selector.currentData()
        while self.sensor_form.count() > 2:
            item = self.sensor_form.takeAt(2)
            if item.widget() is not None:
                item.widget().deleteLater()
        self.sensor_controls.clear()
        self.sensor_boxes.clear()
        self.sensor_selector.blockSignals(True)
        self.sensor_selector.clear()
        for spec in self.model.sensor_specs:
            key = spec["key"]
            self.sensor_selector.addItem(spec["name"], key)
            box = QGroupBox(f"{spec['name']}  ·  {spec['signal']}")
            grid = QGridLayout(box)
            enabled = QCheckBox("Enabled"); enabled.setChecked(spec["enabled"])
            x = self._sensor_spin(spec, "x", -1000, 1000)
            y = self._sensor_spin(spec, "y", -1000, 1000)
            angle = self._sensor_spin(spec, "angle", -180, 180)
            height = self._sensor_spin(spec, "height_mm", 0, 1500)
            height.setSuffix(" mm")
            self.sensor_controls[key] = (x, y, angle, height)
            self.sensor_boxes[key] = box
            enabled.toggled.connect(lambda value, s=spec: self._sensor_changed(s, "enabled", value))
            grid.addWidget(enabled, 0, 0, 1, 2)
            for row, (label, widget) in enumerate((("Right x (mm)", x),
                                                    ("Forward y (mm)", y),
                                                    ("Angle (°)", angle),
                                                    ("Height above ground", height)), 1):
                grid.addWidget(QLabel(label), row, 0)
                grid.addWidget(widget, row, 1)
            self.sensor_form.addWidget(box)
        index = self.sensor_selector.findData(previous_key)
        self.sensor_selector.setCurrentIndex(index if index >= 0 else 0)
        self.sensor_selector.blockSignals(False)
        self._selected_sensor_changed()

    def _selected_sensor_changed(self, *_args):
        self.canvas.selected_sensor_key = self.sensor_selector.currentData()

    def refresh_from_wiring(self):
        """Update placement names/devices immediately after a Wiring tab edit."""
        self._build_specs()
        self.canvas._sensor_drag = None
        self._populate_sensor_controls()
        self.canvas.update()

    def _sensor_spin(self, spec, field, minimum, maximum):
        w = QDoubleSpinBox(); w.setRange(minimum, maximum); w.setDecimals(1); w.setValue(spec[field])
        w.valueChanged.connect(lambda value, s=spec, f=field: self._sensor_changed(s, f, value)); return w

    def _sensor_changed(self, spec, field, value):
        spec[field] = value
        self.settings.setValue(f"arena/sensors/{spec['key']}/{field}", value)
        self.canvas.update()

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

    def _capture_encoder_start(self):
        left = number(self.model.latest.get("encoder.0"))
        right = number(self.model.latest.get("encoder.1"))
        if left is None or right is None:
            self.encoder_cal_status.setText("No live encoder counts yet — connect the robot first.")
            return
        self._encoder_calibration_start = (left, right)
        self.encoder_cal_status.setText(f"Start L={left:.0f}, R={right:.0f}; roll forward, then calculate.")

    def _apply_measured_encoder_runs(self):
        self.left_scale.setValue(DEFAULT_LEFT_MM_PER_COUNT)
        self.right_scale.setValue(DEFAULT_RIGHT_MM_PER_COUNT)
        self.invert_left.setChecked(False)
        self.invert_right.setChecked(True)
        self._calibration_changed()
        self.reset_map()
        self.encoder_cal_status.setText(
            "Applied both runs: L=0.08833, R=0.08609 mm/count; encoder 1 inverted. "
            "These are provisional chassis-distance scales, not motor trim. "
            "Verify wheel travel separately before trusting turning odometry."
        )

    def _calculate_encoder_scale(self):
        if self._encoder_calibration_start is None:
            self.encoder_cal_status.setText("Capture start counts first.")
            return
        left = number(self.model.latest.get("encoder.0"))
        right = number(self.model.latest.get("encoder.1"))
        if left is None or right is None:
            self.encoder_cal_status.setText("Encoder counts unavailable.")
            return
        dl = left - self._encoder_calibration_start[0]
        dr = right - self._encoder_calibration_start[1]
        if abs(dl) < 10 or abs(dr) < 10:
            self.encoder_cal_status.setText("Too few counts; move farther with motors OFF.")
            return
        distance = self.calibration_distance.value()
        self.left_scale.setValue(distance / abs(dl))
        self.right_scale.setValue(distance / abs(dr))
        self.invert_left.setChecked(dl < 0)
        self.invert_right.setChecked(dr < 0)
        self._encoder_calibration_start = None
        self.reset_map()
        self.encoder_cal_status.setText(
            f"Saved L={self.left_scale.value():.5f}, R={self.right_scale.value():.5f} mm/count; "
            "verify with a second measured run. Wheel track still needs calibration."
        )

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
