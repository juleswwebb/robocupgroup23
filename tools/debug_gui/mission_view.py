"""Graphical arena editor and supervised route controls for Group 23."""

from __future__ import annotations

import json
import math

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from mission_layout import MissionLayout


class MissionCanvas(QWidget):
    changed = pyqtSignal()
    selection_changed = pyqtSignal()

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.tool = "Select / drag"
        self.selected = None
        self.dragging = False
        self.live_candidates = []
        self.robot_pose = None
        self.active_waypoint = None
        self.setMinimumSize(700, 400)

    def _geometry(self):
        scale = min((self.width()-50)/self.model.WIDTH_MM,
                    (self.height()-50)/self.model.HEIGHT_MM)
        return scale, (self.width()-self.model.WIDTH_MM*scale)/2, (self.height()-self.model.HEIGHT_MM*scale)/2

    def _point(self, x, y):
        scale, ox, oy = self._geometry()
        return QPointF(ox+x*scale, oy+y*scale)

    def _world(self, point):
        scale, ox, oy = self._geometry()
        return (max(0, min(self.model.WIDTH_MM, (point.x()-ox)/scale)),
                max(0, min(self.model.HEIGHT_MM, (point.y()-oy)/scale)))

    def _hit(self, x, y):
        if math.dist((x, y), self.model.start) < 100:
            return ("start", None)
        for item in reversed(self.model.weights):
            if math.hypot(x-item["x"], y-item["y"]) < 100:
                return ("weight", item)
        for item in reversed(self.model.obstacles):
            if item["kind"] == "tube":
                inside = math.hypot(x-item["x"], y-item["y"]) <= item["width"]/2
            else:
                x0, y0, x1, y1 = self.model.obstacle_rect(item)
                inside = x0 <= x <= x1 and y0 <= y <= y1
            if inside:
                return ("obstacle", item)
        return None

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        x, y = self._world(event.position())
        if self.tool == "Select / drag":
            self.selected = self._hit(x, y)
            self.dragging = self.selected is not None
            self.selection_changed.emit()
        elif self.tool == "Set start":
            self.model.start = (x, y)
            self.selected = ("start", None)
            self.changed.emit()
        elif self.tool in ("Real weight", "Dummy weight"):
            item = {"x": x, "y": y, "dummy": self.tool == "Dummy weight"}
            self.model.weights.append(item)
            self.selected = ("weight", item)
            self.changed.emit()
        elif self.tool in ("Wall", "Ramp", "Tube"):
            sizes = {"Wall": (700, 110), "Ramp": (700, 380), "Tube": (320, 320)}
            width, height = sizes[self.tool]
            item = {"kind": self.tool.lower(), "x": x, "y": y,
                    "width": width, "height": height, "rotation": 0}
            self.model.obstacles.append(item)
            self.selected = ("obstacle", item)
            self.changed.emit()
        self.selection_changed.emit()
        self.update()

    def mouseMoveEvent(self, event):
        if not self.dragging or self.selected is None:
            return
        x, y = self._world(event.position())
        kind, item = self.selected
        if kind == "start":
            self.model.start = (x, y)
        elif item is not None:
            item["x"], item["y"] = x, y
        self.model.route = []
        self.changed.emit()
        self.update()

    def mouseReleaseEvent(self, _event):
        self.dragging = False

    def delete_selected(self):
        if not self.selected:
            return
        kind, item = self.selected
        if kind == "weight" and item in self.model.weights:
            self.model.weights.remove(item)
        elif kind == "obstacle" and item in self.model.obstacles:
            self.model.obstacles.remove(item)
        self.selected = None
        self.changed.emit()
        self.selection_changed.emit()

    def rotate_selected(self):
        if self.selected and self.selected[0] == "obstacle":
            item = self.selected[1]
            item["rotation"] = (item.get("rotation", 0)+90) % 180
            self.changed.emit()

    def paintEvent(self, _event):
        m = self.model
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#111318"))
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        scale, ox, oy = self._geometry()
        p.setPen(QPen(QColor("#6e7785"), 2))
        p.setBrush(QColor("#1d2630"))
        p.drawRect(QRectF(ox, oy, m.WIDTH_MM*scale, m.HEIGHT_MM*scale))
        # Seven interior guides divide the arena into eight equal-width bays.
        p.setPen(QPen(QColor("#303944"), 1, Qt.PenStyle.DashLine))
        for index in range(1, 8):
            x = m.WIDTH_MM * index / 8
            p.drawLine(self._point(x, 0), self._point(x, m.HEIGHT_MM))
        for colour, y0 in (("green", 0), ("blue", m.HEIGHT_MM-m.HOME_MM)):
            p.setPen(QPen(QColor("#33bb76" if colour == "green" else "#3d8bfd"), 2))
            p.setBrush(QColor(40, 150, 95, 60) if colour == "green" else QColor(60, 130, 250, 60))
            p.drawRect(QRectF(self._point(0, y0), self._point(m.HOME_MM, y0+m.HOME_MM)))
        x0, y0, x1, y1 = m.opposite_home()
        p.setPen(QPen(QColor("#f2544b"), 2)); p.setBrush(QColor(242, 84, 75, 45))
        p.drawRect(QRectF(self._point(x0, y0), self._point(x1, y1)))
        for item in m.obstacles:
            selected = self.selected and self.selected[1] is item
            p.setPen(QPen(QColor("#ffffff" if selected else "#f2a84b"), 2))
            p.setBrush(QColor(210, 125, 55, 140))
            if item["kind"] == "tube":
                p.drawEllipse(self._point(item["x"], item["y"]), item["width"]*scale/2, item["width"]*scale/2)
            else:
                x0, y0, x1, y1 = m.obstacle_rect(item)
                p.drawRect(QRectF(self._point(x0, y0), self._point(x1, y1)))
        for item in m.weights:
            p.setPen(QPen(QColor("#ffffff" if self.selected and self.selected[1] is item else "#bc85ef"), 2))
            p.setBrush(QColor("#7d8090" if item["dummy"] else "#bc85ef"))
            p.drawEllipse(self._point(item["x"], item["y"]), 7, 7)
        p.setPen(QPen(QColor("#ea77dc"), 2)); p.setBrush(Qt.BrushStyle.NoBrush)
        for x, y in self.live_candidates:
            p.drawEllipse(self._point(x, y), 11, 11)
        if m.route:
            p.setPen(QPen(QColor("#4dd6df"), 3))
            before = self._point(*m.start)
            for waypoint in m.route:
                after = self._point(waypoint["x"], waypoint["y"])
                p.drawLine(before, after)
                if waypoint["target"]: p.drawEllipse(after, 5, 5)
                before = after
        if self.active_waypoint is not None and 0 <= self.active_waypoint < len(m.route):
            target = m.route[self.active_waypoint]
            p.setPen(QPen(QColor("#fff2a8"), 3)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(self._point(target["x"], target["y"]), 12, 12)
        if self.robot_pose is not None:
            x, y, heading_deg = self.robot_pose
            p.setPen(QPen(QColor("#ffffff"), 2)); p.setBrush(QColor("#3d8bfd"))
            p.drawEllipse(self._point(x, y), 9, 9)
            angle = math.radians(heading_deg)
            p.drawLine(self._point(x, y), self._point(x + 140*math.cos(angle), y + 140*math.sin(angle)))
        p.setPen(QPen(QColor("#ffffff"), 2)); p.setBrush(QColor("#3d8bfd"))
        origin = self._point(*m.start)
        p.drawEllipse(origin, 8, 8)
        theta = math.radians(m.heading_deg)
        p.drawLine(origin, self._point(m.start[0]+150*math.cos(theta), m.start[1]+150*math.sin(theta)))
        p.end()


class MissionPlannerView(QWidget):
    follow_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    route_changed = pyqtSignal()

    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.model = MissionLayout()
        try:
            saved = self.settings.value("arena/mission_layout", "")
            if saved: self.model.load_dict(json.loads(str(saved)))
        except (ValueError, TypeError, KeyError):
            pass
        root = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        self.tool = QComboBox()
        self.tool.addItems(("Select / drag", "Set start", "Real weight", "Dummy weight", "Wall", "Ramp", "Tube"))
        toolbar.addWidget(QLabel("Tool")); toolbar.addWidget(self.tool)
        self.home = QComboBox(); self.home.addItems(("Green home", "Blue home"))
        self.home.setCurrentIndex(0 if self.model.my_home == "green" else 1)
        self.home.currentIndexChanged.connect(self._home_changed)
        toolbar.addWidget(QLabel("Our base")); toolbar.addWidget(self.home)
        self.heading = QComboBox()
        self.heading.addItems(("East →", "South ↓", "West ←", "North ↑"))
        self.heading.setCurrentIndex(int(round(self.model.heading_deg/90)) % 4)
        self.heading.currentIndexChanged.connect(self._heading_changed)
        toolbar.addWidget(QLabel("Start heading")); toolbar.addWidget(self.heading)
        self.return_home = QCheckBox("Return home after weights")
        self.return_home.toggled.connect(lambda _checked: self._changed())
        toolbar.addWidget(self.return_home)
        toolbar.addStretch(); root.addLayout(toolbar)
        route_controls = QHBoxLayout()
        plan = QPushButton("PLAN WEIGHT ROUTE"); plan.setObjectName("primaryButton")
        plan.clicked.connect(self._plan)
        route_controls.addWidget(plan)
        self.follow_button = QPushButton("FOLLOW ROUTE")
        self.follow_button.setObjectName("primaryButton")
        self.follow_button.setEnabled(False)
        self.follow_button.clicked.connect(lambda: self.follow_requested.emit())
        route_controls.addWidget(self.follow_button)
        self.stop_button = QPushButton("STOP ROUTE")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(lambda: self.stop_requested.emit())
        route_controls.addWidget(self.stop_button)
        route_controls.addStretch(); root.addLayout(route_controls)
        body = QHBoxLayout()
        self.canvas = MissionCanvas(self.model)
        self.tool.currentTextChanged.connect(lambda mode: setattr(self.canvas, "tool", mode))
        self.canvas.changed.connect(self._changed)
        self.canvas.selection_changed.connect(self._selection_changed)
        body.addWidget(self.canvas, 1)
        panel = QGroupBox("Selected item / clearance")
        panel.setMaximumWidth(270)
        form = QFormLayout(panel)
        self.selected_label = QLabel("None")
        form.addRow("Selection", self.selected_label)
        self.width_spin = QDoubleSpinBox(); self.width_spin.setRange(50, 4000)
        self.height_spin = QDoubleSpinBox(); self.height_spin.setRange(50, 2400)
        self.width_spin.valueChanged.connect(self._dimension_changed)
        self.height_spin.valueChanged.connect(self._dimension_changed)
        form.addRow("Width / diameter mm", self.width_spin)
        form.addRow("Depth mm", self.height_spin)
        self.radius_spin = QDoubleSpinBox(); self.radius_spin.setRange(50, 600)
        self.radius_spin.setValue(self.model.robot_radius_mm)
        self.radius_spin.valueChanged.connect(self._clearance_changed)
        form.addRow("Robot radius mm", self.radius_spin)
        self.margin_spin = QDoubleSpinBox(); self.margin_spin.setRange(0, 400)
        self.margin_spin.setValue(self.model.margin_mm)
        self.margin_spin.valueChanged.connect(self._clearance_changed)
        form.addRow("Safety margin mm", self.margin_spin)
        rotate = QPushButton("Rotate selected 90°"); rotate.clicked.connect(self.canvas.rotate_selected)
        form.addRow(rotate)
        delete = QPushButton("Delete selected"); delete.clicked.connect(self.canvas.delete_selected)
        form.addRow(delete)
        body.addWidget(panel); root.addLayout(body, 1)
        self.status = QLabel("Plan a route, then FOLLOW to command the robot from this app. Marked start and heading must match the physical robot.")
        self.status.setWordWrap(True); root.addWidget(self.status)
        self.follow_status = QLabel("Route follower idle · robot stays stopped")
        self.follow_status.setWordWrap(True); root.addWidget(self.follow_status)
        self.live_status = QLabel("Live weight candidates: 0 (hollow pink; requires calibrated start and odometry)")
        root.addWidget(self.live_status)
        self._selection_changed()

    def update_live_candidates(self, arena_model):
        # ArenaModel starts at (0, 0) facing +Y; mission heading 0 faces +X.
        # This is an overlay only, never auto-added as a trusted target.
        theta = math.radians(self.model.heading_deg)
        sx, sy = self.model.start
        self.canvas.live_candidates = [
            (sx + ly*math.cos(theta) - lx*math.sin(theta),
             sy + ly*math.sin(theta) + lx*math.cos(theta))
            for lx, ly in arena_model.detected_weights
        ]
        self.live_status.setText(
            f"Live weight candidates: {len(self.canvas.live_candidates)} "
            "(hollow pink; verify start pose and odometry before use)"
        )
        self.canvas.update()

    def _home_changed(self, index):
        self.model.my_home = "green" if index == 0 else "blue"
        self._changed()

    def _heading_changed(self, index):
        self.model.heading_deg = index * 90.0
        self._changed()

    def _clearance_changed(self, _value):
        self.model.robot_radius_mm = self.radius_spin.value()
        self.model.margin_mm = self.margin_spin.value()
        self._changed()

    def _selection_changed(self):
        selected = self.canvas.selected
        item = selected[1] if selected else None
        self.selected_label.setText("None" if not selected else selected[0].upper())
        enabled = selected is not None and selected[0] == "obstacle"
        self.width_spin.setEnabled(enabled); self.height_spin.setEnabled(enabled)
        if enabled:
            self.width_spin.blockSignals(True); self.height_spin.blockSignals(True)
            self.width_spin.setValue(item["width"]); self.height_spin.setValue(item["height"])
            self.width_spin.blockSignals(False); self.height_spin.blockSignals(False)

    def _dimension_changed(self, _value):
        selected = self.canvas.selected
        if selected and selected[0] == "obstacle":
            item = selected[1]
            item["width"] = self.width_spin.value()
            item["height"] = self.width_spin.value() if item["kind"] == "tube" else self.height_spin.value()
            self._changed()

    def _changed(self):
        self.model.route = []
        self.follow_button.setEnabled(False)
        self.route_changed.emit()
        self.settings.setValue("arena/mission_layout", json.dumps(self.model.to_dict()))
        self.canvas.update()

    def _plan(self):
        self.route_changed.emit()
        if self.model.plan(self.return_home.isChecked()):
            real = sum(not item["dummy"] for item in self.model.weights)
            self.status.setText(f"Desktop route: {len(self.model.route)} waypoints through {real} real weights"
                                + (" and back home" if self.return_home.isChecked() else "")
                                + ". Ready for supervised desktop following.")
        else:
            self.status.setText("Route unavailable: " + self.model.error)
        self.follow_button.setEnabled(bool(self.model.route))
        self.canvas.update()

    def set_follow_status(self, text, *, active=False, waypoint=None):
        self.follow_status.setText(text)
        self.follow_button.setEnabled(bool(self.model.route) and not active)
        self.stop_button.setEnabled(active)
        self.canvas.active_waypoint = waypoint if active else None
        self.canvas.update()

    def set_robot_pose(self, right_mm, forward_mm, heading_rad):
        theta = math.radians(self.model.heading_deg)
        sx, sy = self.model.start
        self.canvas.robot_pose = (
            sx + forward_mm * math.cos(theta) - right_mm * math.sin(theta),
            sy + forward_mm * math.sin(theta) + right_mm * math.cos(theta),
            self.model.heading_deg + 90 - math.degrees(heading_rad),
        )
        self.canvas.update()
