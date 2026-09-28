"""
DebugGUI.py

Live robot debug console with:
- COM port selection
- dynamic telemetry
- dashboard commands
- live plots
- dynamic parameters and commands
- logs/raw serial
- full session recording to .rdbg files
- manual "Log Fault" event markers

Dependencies:
    pip install PyQt6 pyserial pyqtgraph

Required beside this file:
    BluetoothSerial.py
    DataRecorder.py
"""

from __future__ import annotations

import html
import json
import math
import sys
import time
import subprocess
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

from PyQt6.QtCore import QEvent, Qt, QSettings, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

import theme
from BluetoothSerial import BluetoothSerial
from DataRecorder import DataRecorder
from tof_view import TofView
from wiring import HardwareMap, WiringPanel
from arena_view import ArenaView, number
from mission_view import MissionPlannerView
from route_follower import (
    EMERGENCY_STOP_MM, MAP_REQUIRED_MM, RouteFollower, local_to_mission,
    matrix_obstacle_points, mission_to_local, prepare_route,
)
from weight_targeting import WeightTracker



# Log levels the firmware emits, mapped to theme colours. Anything not
# listed falls back to plain body text.
LOG_LEVEL_COLOURS = {
    "ERROR": theme.DANGER,
    "FATAL": theme.DANGER,
    "WARN": theme.WARNING,
    "WARNING": theme.WARNING,
    "SYSTEM": theme.ACCENT,
    "DEBUG": theme.TEXT_MUTED,
}


class ValueEditor(QWidget):
    def __init__(self, definition: dict, parent=None):
        super().__init__(parent)

        datatype = str(
            definition.get(
                "datatype",
                definition.get("type", "float"),
            )
        ).lower()

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if datatype in ("bool", "boolean"):
            self.editor = QCheckBox()
            self.editor.setChecked(
                bool(
                    definition.get(
                        "value",
                        definition.get("default", False),
                    )
                )
            )

        elif datatype in ("int", "integer"):
            self.editor = QSpinBox()
            self.editor.setRange(
                int(definition.get("min", -1_000_000_000)),
                int(definition.get("max", 1_000_000_000)),
            )
            self.editor.setSingleStep(
                max(int(definition.get("step", 1)), 1)
            )
            self.editor.setValue(
                int(
                    definition.get(
                        "value",
                        definition.get("default", 0),
                    )
                )
            )

        elif datatype in ("enum", "choice", "select"):
            self.editor = QComboBox()

            for option in definition.get("options", []):
                if isinstance(option, dict):
                    label = str(
                        option.get(
                            "label",
                            option.get("value", ""),
                        )
                    )
                    value = option.get("value", label)
                    self.editor.addItem(label, value)
                else:
                    self.editor.addItem(str(option), option)

            current = definition.get(
                "value",
                definition.get("default"),
            )

            for index in range(self.editor.count()):
                if self.editor.itemData(index) == current:
                    self.editor.setCurrentIndex(index)
                    break

        elif datatype in ("str", "string", "text"):
            self.editor = QLineEdit()
            self.editor.setText(
                str(
                    definition.get(
                        "value",
                        definition.get("default", ""),
                    )
                )
            )

        else:
            self.editor = QDoubleSpinBox()
            self.editor.setDecimals(
                int(definition.get("decimals", 4))
            )
            self.editor.setRange(
                float(definition.get("min", -1e12)),
                float(definition.get("max", 1e12)),
            )
            self.editor.setSingleStep(
                float(definition.get("step", 0.01))
            )
            self.editor.setValue(
                float(
                    definition.get(
                        "value",
                        definition.get("default", 0.0),
                    )
                )
            )

        layout.addWidget(self.editor)

    def value(self):
        if isinstance(self.editor, QCheckBox):
            return self.editor.isChecked()

        if isinstance(self.editor, QComboBox):
            return self.editor.currentData()

        if isinstance(self.editor, QLineEdit):
            return self.editor.text()

        return self.editor.value()

    def set_value(self, value: Any):
        try:
            if isinstance(self.editor, QCheckBox):
                self.editor.setChecked(bool(value))

            elif isinstance(self.editor, QComboBox):
                for index in range(self.editor.count()):
                    if self.editor.itemData(index) == value:
                        self.editor.setCurrentIndex(index)
                        break

            elif isinstance(self.editor, QLineEdit):
                self.editor.setText(str(value))

            else:
                self.editor.setValue(value)

        except Exception:
            pass


class CommandWidget(QGroupBox):
    def __init__(self, definition: dict, send_callback, parent=None):
        name = str(definition.get("name", "command"))
        label = str(definition.get("label", name))

        super().__init__(label, parent)

        self.name = name
        self.send_callback = send_callback
        self.argument_editors = {}

        layout = QFormLayout(self)

        description = definition.get("description")

        if description:
            description_label = QLabel(str(description))
            description_label.setWordWrap(True)
            layout.addRow(description_label)

        arguments = definition.get(
            "args",
            definition.get("arguments", []),
        )

        if isinstance(arguments, dict):
            converted = []

            for arg_name, arg_definition in arguments.items():
                if isinstance(arg_definition, dict):
                    item = dict(arg_definition)
                    item["name"] = arg_name
                else:
                    item = {
                        "name": arg_name,
                        "type": str(arg_definition),
                    }

                converted.append(item)

            arguments = converted

        for argument in arguments:
            arg_name = str(argument.get("name", "value"))
            arg_label = str(argument.get("label", arg_name))

            editor = ValueEditor(argument)

            self.argument_editors[arg_name] = editor

            layout.addRow(
                arg_label,
                editor,
            )

        run_button = QPushButton(f"Run {label}")
        run_button.clicked.connect(self._execute)

        layout.addRow(run_button)

    def _execute(self):
        arguments = {
            name: editor.value()
            for name, editor in self.argument_editors.items()
        }

        self.send_callback(
            self.name,
            arguments,
        )


class RobotDebugGUI(QMainWindow):
    MAX_HISTORY_POINTS = 10000

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot Debug Console")

        # Sized against the actual screen rather than a fixed 1500x900,
        # which overflowed a 1440x900 laptop display once the menu bar
        # and dock were accounted for.
        self.setMinimumSize(800, 560)
        theme.fit_to_screen(self, preferred_width=1400, preferred_height=880)

        self.settings = QSettings(
            "RobotProject",
            "DebugGUI",
        )

        self.bluetooth = BluetoothSerial()
        self.recorder = DataRecorder()

        self.telemetry = {}
        self.telemetry_definitions = {}
        self.telemetry_rows = {}
        self.telemetry_history = {}
        self.parameter_editors = {}
        self.command_widgets = {}
        self.dashboard_command_widgets = {}
        self.plot_curves = {}
        self.plot_colour_index = 0
        self.drive_keys: set[int] = set()
        self.drive_command_available = False
        self.drum_command_available = False
        self.drum_held = False
        self.drum_latched = False
        self.robot_debug_mode = False
        self.route_follower = None
        self.route_last_frame_monotonic = None
        self.route_started_at = None
        self.route_awaiting_pose = False
        self.route_awaiting_limit = False
        self.route_last_obstacle_frame = None
        self.route_pause_until = None
        self.route_resume_after_frame = None
        self.route_replans = 0

        # Recording metadata / parameter snapshot support.
        self.parameter_definitions = {}
        self.parameter_values = {}

        # Link-health counters.
        self.raw_line_times = deque(maxlen=5000)
        self.telemetry_event_times = deque(maxlen=10000)
        self.last_telemetry_monotonic = None
        self.connected_since_monotonic = None
        self.protocol_error_count = 0

        self.start_time = time.monotonic()

        # What's plugged in where, with human names. Loaded before the UI
        # because the dashboard and plots label signals from it.
        self.hardware_map, wiring_warning = HardwareMap.load(
            HardwareMap.default_path()
        )

        self._build_ui()
        self._connect_signals()

        if wiring_warning:
            self.add_log("WARN", wiring_warning)

        self.port_refresh_timer = QTimer(self)
        self.port_refresh_timer.timeout.connect(
            self.refresh_ports
        )
        self.port_refresh_timer.start(2000)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(
            self.update_plot
        )
        self.plot_timer.start(50)

        self.recording_timer = QTimer(self)
        self.recording_timer.timeout.connect(
            self.update_recording_clock
        )
        self.recording_timer.start(250)

        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(
            self.update_link_health
        )
        self.health_timer.start(500)

        # Fresh drive commands are sent while a key is held. The firmware has
        # its own 300 ms timeout as the final protection if this app stalls.
        self.drive_keepalive_timer = QTimer(self)
        self.drive_keepalive_timer.timeout.connect(self._send_keyboard_drive)
        self.drive_keepalive_timer.start(100)
        self.drum_keepalive_timer = QTimer(self)
        self.drum_keepalive_timer.timeout.connect(self._send_drum_command)
        self.drum_keepalive_timer.start(100)
        self.route_timer = QTimer(self)
        self.route_timer.timeout.connect(self._mission_route_step)
        self.route_timer.start(100)

        self.apply_device_names()

        self.refresh_ports()

    # =================================================================
    # Recording
    # =================================================================

    def default_data_directory(self) -> Path:
        # Same directory DataVisualiser.py reads from, so recordings made
        # here show up there without any copying about.
        data_dir = Path(__file__).resolve().parent / "Data"

        data_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        return data_dir

    @staticmethod
    def repository_root() -> Path:
        """Walk up until we find the repo, so this doesn't break if the
        file gets moved to a different depth in the tree."""
        here = Path(__file__).resolve()

        for candidate in here.parents:
            if (candidate / ".git").exists():
                return candidate

        return here.parents[-1]

    def git_metadata(self) -> dict:
        """Best-effort Git branch/commit capture for reproducible tests."""
        try:
            project_root = self.repository_root()
            branch = subprocess.check_output(
                ["git", "branch", "--show-current"],
                cwd=project_root,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            commit = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=project_root,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            dirty = bool(
                subprocess.check_output(
                    ["git", "status", "--porcelain"],
                    cwd=project_root,
                    text=True,
                    stderr=subprocess.DEVNULL,
                ).strip()
            )
            return {
                "git_branch": branch,
                "git_commit": commit,
                "git_dirty": str(dirty).lower(),
            }
        except Exception:
            return {
                "git_branch": "",
                "git_commit": "",
                "git_dirty": "",
            }

    def toggle_recording(self):
        if self.recorder.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if not self.bluetooth.is_connected():
            QMessageBox.warning(
                self,
                "Robot not connected",
                "Connect to the robot before starting a recording.",
            )
            return

        now = datetime.now()

        suggested = (
            f"RobotRun_"
            f"{now:%Y-%m-%d_%H-%M-%S}"
            f".rdbg"
        )

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Robot Recording As",
            str(
                self.default_data_directory()
                / suggested
            ),
            "Robot Debug Recording (*.rdbg)",
        )

        if not filename:
            return

        path = Path(filename)

        if path.suffix.lower() != ".rdbg":
            path = path.with_suffix(".rdbg")

        session_metadata = {
            "test_name": self.test_name_edit.text().strip(),
            "test_notes": self.test_notes_edit.text().strip(),
            # So a recording still says what "Top right ToF" was
            # plugged into, even after the wiring is later changed.
            "wiring": json.dumps(self.hardware_map.snapshot()),
            **self.git_metadata(),
        }

        self.recorder.start(
            path,
            session_name=path.stem,
            port=str(
                self.port_combo.currentData()
                or ""
            ),
            baudrate=int(
                self.baud_combo.currentData()
            ),
            metadata=session_metadata,
        )

        # Snapshot every known parameter at recording start.
        snapshot = dict(self.parameter_values)
        for name, editor in self.parameter_editors.items():
            try:
                snapshot[name] = editor.value()
            except Exception:
                pass
        self.recorder.record_parameter_snapshot(snapshot)

        self.record_button.setText(
            "Stop Recording"
        )

        self.recording_status.setText(
            f"● REC — {path.name}"
        )
        self.recording_status.setToolTip(str(path))
        theme.set_pill_state(self.recording_status, "busy")

        self.log_fault_button.setEnabled(
            True
        )

        self.statusBar().showMessage(
            f"Recording to {path}"
        )

        self.add_log(
            "SYSTEM",
            f"Recording started: {path.name}",
        )

    def stop_recording(self):
        if not self.recorder.is_recording:
            return

        path = self.recorder.path

        self.recorder.record_log(
            "SYSTEM",
            "Recording stopped",
        )

        self.recorder.stop()

        self.record_button.setText(
            "Start Recording"
        )

        self.recording_status.setText(
            "NOT RECORDING"
        )
        self.recording_status.setToolTip("")
        theme.set_pill_state(self.recording_status, "")

        self.recording_time_label.setText(
            "00:00:00"
        )

        self.log_fault_button.setEnabled(
            False
        )

        if path is not None:
            self.statusBar().showMessage(
                f"Recording saved: {path}"
            )

            self.add_log(
                "SYSTEM",
                f"Recording saved: {path.name}",
            )

    def update_recording_clock(self):
        if not self.recorder.is_recording:
            return

        seconds = int(
            self.recorder.elapsed
        )

        hours, remainder = divmod(
            seconds,
            3600,
        )

        minutes, seconds = divmod(
            remainder,
            60,
        )

        self.recording_time_label.setText(
            f"{hours:02d}:"
            f"{minutes:02d}:"
            f"{seconds:02d}"
        )

    def log_fault(self):
        """
        Immediately mark the current recording time as a fault.

        No dialog is shown because the point of this button is to mark
        something abnormal as quickly as possible while watching the robot.
        """
        if not self.recorder.is_recording:
            return

        elapsed = self.recorder.record_fault(
            "MANUAL FAULT MARKER"
        )

        if elapsed is None:
            return

        self.add_log(
            "FAULT",
            f"Manual fault marker logged at {elapsed:.3f} s",
        )

        self.statusBar().showMessage(
            f"FAULT MARKED at {elapsed:.3f} s"
        )

        # Give immediate visual acknowledgement without blocking the GUI.
        self.log_fault_button.setText(
            f"FAULT LOGGED @ {elapsed:.1f}s"
        )

        QTimer.singleShot(
            900,
            lambda: self.log_fault_button.setText(
                "LOG FAULT"
            ),
        )

    # =================================================================
    # UI
    # =================================================================

    def _build_ui(self):
        central = QWidget()

        self.setCentralWidget(
            central
        )

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 8)
        main_layout.setSpacing(10)

        # --------------------------------------------------------------
        # Header
        #
        # Two compact rows rather than one long one: the previous single
        # row needed roughly 1900px of width and simply ran off the side
        # of a 1440px-wide laptop screen.
        # --------------------------------------------------------------

        header = QGroupBox("Connection && Session")
        header_layout = QVBoxLayout(header)
        header_layout.setSpacing(8)

        # ---- Row 1: link ----

        link_row = QHBoxLayout()
        link_row.setSpacing(8)

        port_label = QLabel("Port")
        port_label.setObjectName("fieldLabel")
        link_row.addWidget(port_label)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(210)
        link_row.addWidget(self.port_combo)

        self.refresh_button = QPushButton("Refresh")
        link_row.addWidget(self.refresh_button)

        baud_label = QLabel("Baud")
        baud_label.setObjectName("fieldLabel")
        link_row.addWidget(baud_label)

        self.baud_combo = QComboBox()

        for baud in (
            9600,
            19200,
            38400,
            57600,
            115200,
            230400,
            460800,
            921600,
        ):
            self.baud_combo.addItem(str(baud), baud)

        saved_baud = int(self.settings.value("baud", 115200))
        baud_index = self.baud_combo.findData(saved_baud)

        if baud_index >= 0:
            self.baud_combo.setCurrentIndex(baud_index)

        link_row.addWidget(self.baud_combo)

        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("primary")
        self.connect_button.setMinimumWidth(100)
        link_row.addWidget(self.connect_button)

        self.connection_status = QLabel("DISCONNECTED")
        self.connection_status.setObjectName("statusPill")
        theme.set_pill_state(self.connection_status, "bad")
        link_row.addWidget(self.connection_status)

        link_row.addStretch()

        self.enter_debug_button = QPushButton("Enter Debug Mode")
        self.enter_debug_button.setEnabled(False)
        link_row.addWidget(self.enter_debug_button)

        self.exit_debug_button = QPushButton("Exit Debug Mode")
        self.exit_debug_button.setEnabled(False)
        link_row.addWidget(self.exit_debug_button)

        header_layout.addLayout(link_row)

        # ---- Row 2: recording ----

        session_row = QHBoxLayout()
        session_row.setSpacing(8)

        self.record_button = QPushButton("Start Recording")
        self.record_button.setMinimumWidth(130)
        self.record_button.setEnabled(False)
        session_row.addWidget(self.record_button)

        self.log_fault_button = QPushButton("LOG FAULT")
        self.log_fault_button.setObjectName("danger")
        self.log_fault_button.setMinimumWidth(115)
        # Deliberately disabled until a recording is running - there is
        # nothing to attach a marker to before then.
        self.log_fault_button.setEnabled(False)
        session_row.addWidget(self.log_fault_button)

        self.recording_status = QLabel("NOT RECORDING")
        self.recording_status.setObjectName("statusPill")
        session_row.addWidget(self.recording_status)

        self.recording_time_label = QLabel("00:00:00")
        self.recording_time_label.setObjectName("clock")
        session_row.addWidget(self.recording_time_label)

        session_row.addSpacing(6)

        test_label = QLabel("Test")
        test_label.setObjectName("fieldLabel")
        session_row.addWidget(test_label)

        self.test_name_edit = QLineEdit()
        self.test_name_edit.setPlaceholderText("Straight drive PID test")
        self.test_name_edit.setMinimumWidth(150)
        session_row.addWidget(self.test_name_edit, 1)

        notes_label = QLabel("Notes")
        notes_label.setObjectName("fieldLabel")
        session_row.addWidget(notes_label)

        self.test_notes_edit = QLineEdit()
        self.test_notes_edit.setPlaceholderText("Optional")
        self.test_notes_edit.setMinimumWidth(120)
        session_row.addWidget(self.test_notes_edit, 1)

        header_layout.addLayout(session_row)

        main_layout.addWidget(header)

        # --------------------------------------------------------------
        # Link health
        # --------------------------------------------------------------

        health_row = QHBoxLayout()
        health_row.setContentsMargins(4, 0, 4, 0)

        health_caption = QLabel("LINK HEALTH")
        health_caption.setObjectName("fieldLabel")
        health_row.addWidget(health_caption)

        self.link_health_label = QLabel(
            "Frames/s 0     Signals/s 0     Last telemetry —     Protocol errors 0"
        )
        self.link_health_label.setObjectName("metric")
        health_row.addWidget(self.link_health_label)
        health_row.addStretch()

        main_layout.addLayout(health_row)

        self.tabs = QTabWidget()

        main_layout.addWidget(
            self.tabs,
            1,
        )

        self._build_dashboard_tab()
        self._build_plot_tab()
        self._build_matrix_tab()
        self._build_arena_tab()
        self._build_wiring_tab()
        self._build_parameter_tab()
        self._build_command_tab()
        self._build_log_tab()
        self._build_raw_tab()

        self.statusBar().showMessage(
            "Ready"
        )

    # =================================================================
    # Dashboard
    # =================================================================

    def _build_dashboard_tab(self):
        page = QWidget()

        main_layout = QVBoxLayout(
            page
        )

        splitter = QSplitter(
            Qt.Orientation.Horizontal
        )

        main_layout.addWidget(
            splitter,
            1,
        )

        # --------------------------------------------------------------
        # Telemetry
        # --------------------------------------------------------------

        telemetry_panel = QWidget()

        telemetry_layout = QVBoxLayout(
            telemetry_panel
        )

        top_layout = QHBoxLayout()

        title = QLabel(
            "Live Telemetry"
        )
        title.setObjectName("sectionTitle")

        top_layout.addWidget(
            title
        )

        top_layout.addStretch()

        self.clear_telemetry_button = QPushButton(
            "Clear"
        )

        top_layout.addWidget(
            self.clear_telemetry_button
        )

        telemetry_layout.addLayout(
            top_layout
        )

        self.telemetry_table = QTableWidget(
            0,
            5,
        )

        self.telemetry_table.setHorizontalHeaderLabels(
            [
                "Group",
                "Signal",
                "Value",
                "Unit",
                "Last Update",
            ]
        )

        self.telemetry_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )

        self.telemetry_table.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )

        header = (
            self.telemetry_table.horizontalHeader()
        )

        header.setStretchLastSection(
            False
        )

        header.setSectionResizeMode(
            0,
            header.ResizeMode.ResizeToContents,
        )

        header.setSectionResizeMode(
            1,
            header.ResizeMode.Stretch,
        )

        header.setSectionResizeMode(
            2,
            header.ResizeMode.ResizeToContents,
        )

        header.setSectionResizeMode(
            3,
            header.ResizeMode.ResizeToContents,
        )

        header.setSectionResizeMode(
            4,
            header.ResizeMode.ResizeToContents,
        )

        telemetry_layout.addWidget(
            self.telemetry_table,
            1,
        )

        splitter.addWidget(
            telemetry_panel
        )

        # --------------------------------------------------------------
        # Commands
        # --------------------------------------------------------------

        # Scroll the entire command column. Previously only the dynamic
        # commands scrolled, so fixed cards were crushed on laptop screens.
        command_panel = QScrollArea()
        command_panel.setWidgetResizable(True)
        command_panel.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        command_content = QWidget()
        command_panel.setWidget(command_content)

        command_panel_layout = QVBoxLayout(
            command_content
        )
        command_panel_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        command_header = QHBoxLayout()

        command_title = QLabel(
            "Commands"
        )

        command_title.setObjectName("sectionTitle")

        command_header.addWidget(
            command_title
        )

        command_header.addStretch()

        command_panel_layout.addLayout(
            command_header
        )

        stop_group = QGroupBox(
            "Emergency / General"
        )

        stop_layout = QVBoxLayout(
            stop_group
        )

        self.dashboard_stop_button = QPushButton(
            "STOP ROBOT"
        )

        self.dashboard_stop_button.setObjectName(
            "danger"
        )

        self.dashboard_stop_button.setMinimumHeight(
            45
        )

        self.dashboard_stop_button.setEnabled(
            False
        )

        stop_layout.addWidget(
            self.dashboard_stop_button
        )

        command_panel_layout.addWidget(
            stop_group
        )

        drive_group = QGroupBox("Keyboard Drive")
        drive_layout = QVBoxLayout(drive_group)

        self.drive_arm_checkbox = QCheckBox("Arm keyboard drive")
        self.drive_arm_checkbox.setEnabled(False)
        drive_layout.addWidget(self.drive_arm_checkbox)

        drive_hint = QLabel(
            "Click this window, then use W/S or ↑/↓ to drive and A/D or ←/→ to turn. "
            "Release all keys or press Space to stop. Debug Mode is required."
        )
        drive_hint.setWordWrap(True)
        drive_hint.setObjectName("hint")
        drive_layout.addWidget(drive_hint)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed limit"))
        self.drive_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.drive_speed_slider.setRange(5, 100)
        self.drive_speed_slider.setSingleStep(5)
        self.drive_speed_slider.setValue(100)
        self.drive_speed_slider.setEnabled(False)
        speed_row.addWidget(self.drive_speed_slider, 1)
        self.drive_speed_label = QLabel("100%")
        self.drive_speed_label.setMinimumWidth(36)
        speed_row.addWidget(self.drive_speed_label)
        drive_layout.addLayout(speed_row)

        trim_row = QHBoxLayout()
        trim_row.addWidget(QLabel("Left scale"))
        self.drive_left_scale = QSpinBox()
        self.drive_left_scale.setRange(0, 100)
        self.drive_left_scale.setSuffix(" %")
        self.drive_left_scale.setValue(int(self.settings.value("drive/left_scale", 100)))
        self.drive_left_scale.setToolTip("Left command as a percentage of the speed limit; 100% is untrimmed")
        trim_row.addWidget(self.drive_left_scale)
        trim_row.addWidget(QLabel("Right scale"))
        self.drive_right_scale = QSpinBox()
        self.drive_right_scale.setRange(0, 100)
        self.drive_right_scale.setSuffix(" %")
        self.drive_right_scale.setValue(int(self.settings.value("drive/right_scale", 100)))
        self.drive_right_scale.setToolTip("Try 98% right with 100% left to test the measured left drift")
        trim_row.addWidget(self.drive_right_scale)
        drive_layout.addLayout(trim_row)

        self.drive_status_label = QLabel("DISARMED")
        self.drive_status_label.setObjectName("statusPill")
        theme.set_pill_state(self.drive_status_label, "")
        drive_layout.addWidget(self.drive_status_label)

        command_panel_layout.addWidget(drive_group)

        drum_group = QGroupBox("Drum motors · D28 / D29")
        drum_layout = QVBoxLayout(drum_group)
        drum_hint = QLabel("Drums can run alongside robot movement. Set each channel, then hold RUN for a momentary test or switch Continuous RUN on. Requires Debug Mode; firmware stops after 300 ms without a fresh command, and STOP / disconnect always stops the drums.")
        drum_hint.setWordWrap(True)
        drum_layout.addWidget(drum_hint)
        drum_row = QHBoxLayout()
        self.drum_left_spin = QSpinBox(); self.drum_left_spin.setRange(-100, 100)
        self.drum_left_spin.setValue(100); self.drum_left_spin.setSuffix(" %")
        self.drum_right_spin = QSpinBox(); self.drum_right_spin.setRange(-100, 100)
        self.drum_right_spin.setValue(100); self.drum_right_spin.setSuffix(" %")
        drum_row.addWidget(QLabel("Left")); drum_row.addWidget(self.drum_left_spin)
        drum_row.addWidget(QLabel("Right")); drum_row.addWidget(self.drum_right_spin)
        drum_layout.addLayout(drum_row)
        self.drum_hold_button = QPushButton("PRESS AND HOLD · RUN DRUMS")
        self.drum_hold_button.setObjectName("primaryButton")
        self.drum_hold_button.setEnabled(False)
        self.drum_hold_button.pressed.connect(self._start_drum_hold)
        self.drum_hold_button.released.connect(self._stop_drum_hold)
        drum_layout.addWidget(self.drum_hold_button)
        self.drum_latch_button = QPushButton("CONTINUOUS RUN · OFF")
        self.drum_latch_button.setCheckable(True)
        self.drum_latch_button.setObjectName("primaryButton")
        self.drum_latch_button.setEnabled(False)
        self.drum_latch_button.toggled.connect(self._set_drum_latched)
        drum_layout.addWidget(self.drum_latch_button)
        self.drum_status_label = QLabel("WAITING FOR DRUM CONTROL")
        self.drum_status_label.setObjectName("statusPill")
        drum_layout.addWidget(self.drum_status_label)
        command_panel_layout.addWidget(drum_group)

        self.dashboard_command_container = QWidget()

        self.dashboard_command_layout = QVBoxLayout(
            self.dashboard_command_container
        )

        self.dashboard_command_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )

        command_panel_layout.addWidget(
            self.dashboard_command_container,
        )

        splitter.addWidget(
            command_panel
        )

        splitter.setSizes(
            [
                950,
                500,
            ]
        )

        splitter.setStretchFactor(
            0,
            3,
        )

        splitter.setStretchFactor(
            1,
            2,
        )

        self.tabs.addTab(
            page,
            "Dashboard",
        )

    # =================================================================
    # Keyboard drive
    # =================================================================

    DRIVE_KEYS = {
        Qt.Key.Key_W, Qt.Key.Key_A, Qt.Key.Key_S, Qt.Key.Key_D,
        Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right,
    }

    def _drum_controls_available(self) -> bool:
        return (self.bluetooth.is_connected() and self.robot_debug_mode
                and self.drum_command_available)

    def _update_drum_controls(self):
        available = self._drum_controls_available()
        if not available and (self.drum_held or self.drum_latched):
            self._stop_drum_hold()
        self.drum_hold_button.setEnabled(available)
        self.drum_latch_button.setEnabled(available)
        if not available:
            self.drum_status_label.setText("ENABLE DEBUG MODE / CONNECT DRUM CONTROL")
            theme.set_pill_state(self.drum_status_label, "")

    def _start_drum_hold(self):
        if not self._drum_controls_available():
            return
        if self.drum_latched:
            self.drum_latched = False
            self.drum_latch_button.blockSignals(True)
            self.drum_latch_button.setChecked(False)
            self.drum_latch_button.blockSignals(False)
            self.drum_latch_button.setText("CONTINUOUS RUN · OFF")
        self.drum_held = True
        self._send_drum_command()

    def _set_drum_latched(self, enabled: bool):
        if enabled and not self._drum_controls_available():
            self.drum_latch_button.blockSignals(True)
            self.drum_latch_button.setChecked(False)
            self.drum_latch_button.blockSignals(False)
            return
        if enabled:
            self.drum_latched = True
            self.drum_held = False
            self.drum_latch_button.setText("STOP CONTINUOUS RUN")
            self._send_drum_command()
        else:
            self._stop_drum_hold()

    def _send_drum_command(self):
        if not (self.drum_held or self.drum_latched):
            return
        if not self._drum_controls_available():
            self._stop_drum_hold()
            return
        if self.drum_latched and (
            self.last_telemetry_monotonic is None
            or time.monotonic() - self.last_telemetry_monotonic > 2.0
        ):
            self._stop_drum_hold()
            self.drum_status_label.setText("STOPPED · TELEMETRY LOST")
            theme.set_pill_state(self.drum_status_label, "bad")
            return
        left, right = self.drum_left_spin.value(), self.drum_right_spin.value()
        self.bluetooth.send_command("drum_set", left=left, right=right)
        self.drum_status_label.setText(f"RUNNING · L {left:+d}% / R {right:+d}%")
        theme.set_pill_state(self.drum_status_label, "busy")

    def _stop_drum_hold(self):
        was_running = self.drum_held or self.drum_latched
        self.drum_held = False
        self.drum_latched = False
        self.drum_latch_button.blockSignals(True)
        self.drum_latch_button.setChecked(False)
        self.drum_latch_button.blockSignals(False)
        self.drum_latch_button.setText("CONTINUOUS RUN · OFF")
        if was_running and self._drum_controls_available():
            self.bluetooth.send_command("drum_set", left=0, right=0)
        if self._drum_controls_available():
            self.drum_status_label.setText("STOPPED · HOLD OR SWITCH RUN")
            theme.set_pill_state(self.drum_status_label, "ok")

    def focusOutEvent(self, event):
        if self.drum_held:
            self._stop_drum_hold()
        super().focusOutEvent(event)

    def _drive_controls_available(self) -> bool:
        return (
            self.bluetooth.is_connected()
            and self.drive_command_available
            and self.robot_debug_mode
        )

    def _update_drive_controls(self):
        available = self._drive_controls_available()
        self.drive_arm_checkbox.setEnabled(available)
        self.drive_speed_slider.setEnabled(available)
        self.drive_left_scale.setEnabled(available)
        self.drive_right_scale.setEnabled(available)

        if not available:
            self._set_drive_armed(False)
            if not self.drive_command_available:
                text = "WAITING FOR DRIVE CONTROL"
            elif not self.bluetooth.is_connected():
                text = "DISCONNECTED"
            else:
                text = "ENABLE DEBUG MODE"
            self.drive_status_label.setText(text)
            theme.set_pill_state(self.drive_status_label, "")

    def _set_drive_armed(self, armed: bool):
        if not armed and self.drive_keys:
            self.drive_keys.clear()
            self._send_drive_stop()
        self.drive_arm_checkbox.blockSignals(True)
        self.drive_arm_checkbox.setChecked(armed)
        self.drive_arm_checkbox.blockSignals(False)

    def _on_drive_armed_changed(self, armed: bool):
        if armed and not self._drive_controls_available():
            self._set_drive_armed(False)
            return
        if armed:
            self._stop_mission_route("Manual keyboard drive armed")
        if not armed:
            self.drive_keys.clear()
            self._send_drive_stop()
            self.drive_status_label.setText("DISARMED")
            theme.set_pill_state(self.drive_status_label, "")
        else:
            self.drive_status_label.setText("ARMED — RELEASE KEYS TO STOP")
            theme.set_pill_state(self.drive_status_label, "busy")
            self.setFocus(Qt.FocusReason.OtherFocusReason)

    def _on_drive_speed_changed(self, value: int):
        self.drive_speed_label.setText(f"{value}%")
        if value < 80:
            self._stop_mission_route("Drive speed limit set below 80%")
        if self._drive_controls_available():
            self.bluetooth.set_parameter("drive.max_percent", value)

    def _on_drive_scale_changed(self):
        self.settings.setValue("drive/left_scale", self.drive_left_scale.value())
        self.settings.setValue("drive/right_scale", self.drive_right_scale.value())
        if self.drive_keys and self.drive_arm_checkbox.isChecked():
            self._send_keyboard_drive()

    def _keyboard_drive_values(self) -> tuple[int, int]:
        speed = self.drive_speed_slider.value()
        forward = Qt.Key.Key_W in self.drive_keys or Qt.Key.Key_Up in self.drive_keys
        reverse = Qt.Key.Key_S in self.drive_keys or Qt.Key.Key_Down in self.drive_keys
        turn_right = Qt.Key.Key_D in self.drive_keys or Qt.Key.Key_Right in self.drive_keys
        turn_left = Qt.Key.Key_A in self.drive_keys or Qt.Key.Key_Left in self.drive_keys
        linear = int(forward) - int(reverse)
        turn = int(turn_right) - int(turn_left)
        left = max(-speed, min(speed, (linear + turn) * speed))
        right = max(-speed, min(speed, (linear - turn) * speed))
        # Apply per-side calibration only while translating. Pure turns stay
        # symmetric, so a straight-line trim does not add unwanted motion.
        if linear:
            left = round(left * self.drive_left_scale.value() / 100)
            right = round(right * self.drive_right_scale.value() / 100)
        return left, right

    def _send_keyboard_drive(self):
        if not self._drive_controls_available() or not self.drive_arm_checkbox.isChecked():
            return
        left, right = self._keyboard_drive_values()
        if (left or right) and self.route_follower is not None:
            self._stop_mission_route("Manual keyboard drive")
        self.bluetooth.send_command("drive_set", left=left, right=right)
        if left == 0 and right == 0:
            self.drive_status_label.setText("ARMED — STOPPED")
            theme.set_pill_state(self.drive_status_label, "ok")
        else:
            self.drive_status_label.setText(f"DRIVING  L {left:+d}%  R {right:+d}%")
            theme.set_pill_state(self.drive_status_label, "busy")

    def _send_drive_stop(self):
        if self.bluetooth.is_connected() and self.drive_command_available:
            self.bluetooth.send_command("drive_set", left=0, right=0)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key.Key_Space and self.drive_arm_checkbox.isChecked():
            self.drive_keys.clear()
            self._send_keyboard_drive()
            event.accept()
            return
        if key in self.DRIVE_KEYS and self.drive_arm_checkbox.isChecked():
            self.drive_keys.add(key)
            self._send_keyboard_drive()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key in self.DRIVE_KEYS and self.drive_arm_checkbox.isChecked():
            self.drive_keys.discard(key)
            self._send_keyboard_drive()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def changeEvent(self, event):
        # Never keep a drive key logically held after the user alt-tabs away.
        if event.type() == QEvent.Type.ActivationChange and not self.isActiveWindow():
            if self.drive_keys:
                self.drive_keys.clear()
                self._send_drive_stop()
            if self.drum_held:
                self._stop_drum_hold()
            self._stop_mission_route("App lost focus")
        super().changeEvent(event)

    # =================================================================
    # Plots
    # =================================================================

    def _build_plot_tab(self):
        page = QWidget()
        self.plot_page = page

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addWidget(
            QLabel("Signal:")
        )

        self.plot_signal_combo = QComboBox()

        self.plot_signal_combo.setMinimumWidth(
            300
        )

        controls.addWidget(
            self.plot_signal_combo
        )

        self.add_plot_button = QPushButton(
            "Add to Plot"
        )

        self.remove_plot_button = QPushButton(
            "Remove Selected"
        )

        self.clear_plot_button = QPushButton(
            "Clear Plot"
        )

        controls.addWidget(
            self.add_plot_button
        )

        controls.addWidget(
            self.remove_plot_button
        )

        controls.addWidget(
            self.clear_plot_button
        )

        controls.addSpacing(20)

        controls.addWidget(
            QLabel("Time Window:")
        )

        self.time_window_combo = QComboBox()

        for text, seconds in (
            ("5 seconds", 5),
            ("10 seconds", 10),
            ("30 seconds", 30),
            ("1 minute", 60),
            ("2 minutes", 120),
            ("5 minutes", 300),
        ):
            self.time_window_combo.addItem(
                text,
                seconds,
            )

        self.time_window_combo.setCurrentIndex(
            1
        )

        controls.addWidget(
            self.time_window_combo
        )

        controls.addStretch()

        layout.addLayout(
            controls
        )

        splitter = QSplitter(
            Qt.Orientation.Horizontal
        )

        signal_container = QWidget()

        signal_layout = QVBoxLayout(
            signal_container
        )

        signal_layout.addWidget(
            QLabel("Currently plotted:")
        )

        self.active_plot_list = QListWidget()

        signal_layout.addWidget(
            self.active_plot_list
        )

        signal_container.setMaximumWidth(
            280
        )

        splitter.addWidget(
            signal_container
        )

        self.plot_widget = pg.PlotWidget()

        theme.style_plot(
            self.plot_widget
        )

        self.plot_widget.setLabel(
            "bottom",
            "Time",
            units="s",
        )

        self.plot_widget.setLabel(
            "left",
            "Value",
        )

        self.plot_widget.addLegend()

        splitter.addWidget(
            self.plot_widget
        )

        splitter.setStretchFactor(
            1,
            1,
        )

        layout.addWidget(
            splitter,
            1,
        )

        self.tabs.addTab(
            page,
            "Plots",
        )

    # =================================================================
    # 8x8 TOF field
    # =================================================================

    def _build_matrix_tab(self):
        self.tof_view = TofView(
            self.settings,
            telemetry_source=lambda: self.telemetry,
        )
        self.tof_view.zone_plot_requested.connect(self.plot_matrix_zone)
        self.tabs.addTab(self.tof_view, "8×8 TOF")

    # =================================================================
    # Arena map / navigation
    # =================================================================

    def _build_arena_tab(self):
        self.arena_view = ArenaView(
            self.settings,
            self.hardware_map,
        )
        self.arena_view.command_requested.connect(self.execute_command)
        self.arena_view.parameter_requested.connect(self._set_arena_parameter)
        self.arena_view.pose_configuration_changed.connect(
            lambda: self._stop_mission_route("Arena pose or calibration changed")
        )
        self.tabs.addTab(self.arena_view, "Arena View")
        self.mission_view = MissionPlannerView(self.settings)
        self.mission_view.follow_requested.connect(self._start_mission_route)
        self.mission_view.stop_requested.connect(lambda: self._stop_mission_route("Stopped by operator"))
        self.mission_view.route_changed.connect(lambda: self._stop_mission_route("Route changed"))
        self.tabs.addTab(self.mission_view, "Mission Planner")

    def _set_arena_parameter(self, name: str, value: Any):
        self.parameter_values[name] = value
        self.recorder.record_parameter(name, value)
        self.bluetooth.set_parameter(name, value)
        self.add_log("TX", f"{name} = {value}")

    @staticmethod
    def _route_front_range(frame: dict) -> float | None:
        # Same central/top field used by firmware navigation. Bottom rows
        # often see the floor or chassis and are not a forward collision ray.
        values = [number(frame.get(f"tof.array.r{row}c{col}"))
                  for row in range(5) for col in range(2, 6)]
        valid = [value for value in values if value is not None and 0 < value < 4000]
        return min(valid) if valid else None

    def _start_mission_route(self):
        if self.route_follower is not None:
            return
        reason = None
        if not self._drive_controls_available():
            reason = "Connect the robot, enter Debug Mode, and wait for drive control."
        elif self.drive_arm_checkbox.isChecked():
            reason = "Disarm manual keyboard drive first. Drum operation may continue during the route."
        elif self.telemetry.get("navigation.active") or self.telemetry.get("drive.active"):
            reason = "Stop existing drive/navigation activity before starting the route."
        elif (self.route_last_frame_monotonic is None
              or time.monotonic() - self.route_last_frame_monotonic > 1.25):
            reason = "No fresh complete robot telemetry frame."
        frame = self.arena_view.model.latest
        if reason is None and (number(frame.get("encoder.0")) is None
                               or number(frame.get("encoder.1")) is None
                               or number(frame.get("imu.heading")) is None
                               or (number(frame.get("imu.cal_gyro")) or 0) < 2):
            reason = "Need both encoders and a valid, gyro-calibrated IMU."
        if reason is None and frame.get("tof.array_frame_ok") is not True:
            reason = "No healthy 8×8 ToF frame. Upload the updated firmware if this signal is missing."
        matrix_spec = next((spec for spec in self.arena_view.model.sensor_specs
                            if spec.get("kind") == "matrix" and spec.get("enabled", True)), None)
        if reason is None and matrix_spec is None:
            reason = "Enable and position the 8×8 sensor in Arena View first."
        front_mm = self._route_front_range(frame)
        if reason is None and front_mm is not None and front_mm < EMERGENCY_STOP_MM:
            reason = f"Forward obstacle is within {EMERGENCY_STOP_MM} mm."
        if reason is None:
            try:
                points = prepare_route(self.mission_view.model)
            except (ValueError, KeyError, TypeError) as exc:
                reason = str(exc)
        if reason:
            self.mission_view.set_follow_status(f"Cannot start: {reason}")
            return
        answer = QMessageBox.question(
            self, "Start supervised route following?",
            "Confirm the physical robot is stationary at the marked start, aligned "
            "with the start arrow, and the arena is clear. Moving wheel commands "
            "will be 85% left / 100% right when straight; the drive limit "
            "will be set to 100%. Any latched drum command will continue while "
            "the robot moves. An obstacle on the route pauses the robot "
            "while the app tries a detour. "
            "The app must stay "
            "connected; STOP or loss of pose/8×8 frame health halts the motors. "
            "Mapped weight returns can be approached only when both ToF heights "
            "confirm the target for three frames. The robot may align its two "
            "lower ToFs, but pickup and unloading are NOT verified or automatic. "
            "Keep a clear stop zone around the robot.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.arena_view.reset_map()
        self.mission_view.canvas.robot_pose = None
        self.route_last_frame_monotonic = None
        self.route_started_at = time.monotonic()
        self.route_awaiting_pose = True
        self.route_awaiting_limit = True
        self.route_last_obstacle_frame = None
        self.route_pause_until = None
        self.route_resume_after_frame = None
        self.route_replans = 0
        self.route_weight_candidate_count = 0
        self.mission_view.model.live_obstacles.clear()
        self.mission_view.refresh_live_obstacles()
        self.route_follower = RouteFollower(points)
        self.route_weight_tracker = WeightTracker()
        self.drive_speed_slider.blockSignals(True)
        self.drive_speed_slider.setValue(100)
        self.drive_speed_slider.blockSignals(False)
        self.drive_speed_label.setText("100%")
        self.parameter_values.pop("drive.max_percent", None)
        self.bluetooth.set_parameter("drive.max_percent", 100)
        self.recorder.record_parameter("drive.max_percent", 100)
        self.mission_view.set_follow_status("Waiting for drive-limit confirmation and fresh pose…", active=True, waypoint=0)
        self.add_log("INFO", f"Route armed: {len(points)} waypoints; requested 100% drive limit")
        self.recorder.record_log(
            "MISSION",
            f"Route armed: {len(points)} waypoints; waiting for drive-limit confirmation and fresh pose",
        )

    def _stop_mission_route(self, reason="Stopped", *, send_stop=True):
        if self.route_follower is None:
            return
        self.route_follower = None
        self.route_weight_tracker = None
        self.route_awaiting_pose = False
        self.route_awaiting_limit = False
        self.route_pause_until = None
        self.route_resume_after_frame = None
        self.route_started_at = None
        if send_stop and self.bluetooth.is_connected():
            self.bluetooth.send_command("stop")
            self.recorder.record_command("stop", {})
        self.mission_view.set_follow_status(f"Route stopped: {reason}")
        self.add_log("WARN", f"Route stopped: {reason}")
        self.recorder.record_log("MISSION_STOP", f"Route stopped: {reason}")

    def _mission_route_step(self):
        follower = self.route_follower
        if follower is None:
            return
        now = time.monotonic()
        if not self._drive_controls_available() or self.telemetry.get("navigation.active"):
            self._stop_mission_route("Drive control or Debug Mode lost")
            return
        if self.route_started_at is not None and now - self.route_started_at > 120:
            self._stop_mission_route("120 s route time limit")
            return
        if self.route_awaiting_limit:
            if number(self.parameter_values.get("drive.max_percent")) != 100:
                if now - self.route_started_at > 2.5:
                    self._stop_mission_route("Drive limit 100% was not confirmed")
                return
            self.route_awaiting_limit = False
        elif (number(self.parameter_values.get("drive.max_percent")) or 0) < 80:
            self._stop_mission_route("Drive limit fell below 80%")
            return
        if self.route_awaiting_pose:
            if self.route_last_frame_monotonic is None:
                if now - self.route_started_at > 2.5:
                    self._stop_mission_route("Start pose did not arrive")
                return
            self.route_awaiting_pose = False
        if (self.route_last_frame_monotonic is None
                or now - self.route_last_frame_monotonic > 1.25):
            self._stop_mission_route("Telemetry stale for more than 1.25 s")
            return
        frame = self.arena_view.model.latest
        if (number(frame.get("encoder.0")) is None
                or number(frame.get("encoder.1")) is None
                or number(frame.get("imu.heading")) is None
                or (number(frame.get("imu.cal_gyro")) or 0) < 2):
            self._stop_mission_route("Encoder or IMU pose invalid")
            return
        if frame.get("tof.array_frame_ok") is not True:
            self._stop_mission_route("8×8 ToF frame read failed or health signal missing")
            return
        model = self.arena_view.model
        layout = getattr(self.mission_view, "model", None)
        front_mm = self._route_front_range(frame)
        weight = None
        target_leg = False
        search_waypoint = False
        tracker = getattr(self, "route_weight_tracker", None)
        if tracker is not None and layout is not None and follower.index < len(layout.route):
            goal = layout.route[follower.index]
            if goal.get("target"):
                target_leg = True
                target = (float(goal.get("target_x", goal["x"])),
                          float(goal.get("target_y", goal["y"])))
                search_waypoint = bool(goal.get("site_search"))
                weight = tracker.observe(frame, model, layout, target, front_mm,
                                         self.route_last_frame_monotonic)
        # ArenaView continuously looks for the bottom-near/top-far signature,
        # including between planned sites. If a new candidate appears while
        # transiting, stop for review rather than driving past an unplanned
        # object. At a planned site it must also agree with that mapped target.
        candidates = getattr(model, "detected_weights", [])
        seen_candidates = getattr(self, "route_weight_candidate_count", 0)
        if len(candidates) > seen_candidates:
            self.route_weight_candidate_count = len(candidates)
            candidate = candidates[-1]
            if target_leg:
                planned_local = mission_to_local(
                    layout.start, layout.heading_deg,
                    (float(goal.get("target_x", goal["x"])),
                     float(goal.get("target_y", goal["y"]))))
                if math.dist(candidate, planned_local) > 250:
                    self._stop_mission_route(
                        "Unmapped weight-like object detected; stopped for review")
                    return
            else:
                self._stop_mission_route(
                    "Weight-like object detected between planned sites; stopped for review")
                return
        if weight is not None and weight.pending and front_mm is not None and front_mm < MAP_REQUIRED_MM:
            # Stop before collecting more evidence. Never leave the previous
            # motor command running while a close target is ambiguous.
            self.bluetooth.send_command("stop")
            self.recorder.record_command("stop", {})
            self.mission_view.set_follow_status(
                "Verifying mapped weight with upper/lower ToFs…",
                active=True, waypoint=follower.index,
            )
            self.recorder.record_log(
                "MISSION", "Paused to verify a mapped weight using upper/lower ToF readings",
            )
            return
        if self.route_pause_until is not None and now < self.route_pause_until:
            return
        if (self.route_resume_after_frame is not None
                and self.route_last_frame_monotonic == self.route_resume_after_frame):
            return  # do not resume on the same frame that caused the detour
        self.route_pause_until = None
        self.route_resume_after_frame = None
        if self.route_last_obstacle_frame != self.route_last_frame_monotonic:
            self.route_last_obstacle_frame = self.route_last_frame_monotonic
            points = matrix_obstacle_points(frame, model, layout)
            if weight is not None and weight.front_is_target:
                # The intended weight is not a wall: do not turn its own 8×8
                # return into an obstacle. Unmatched returns remain hazards.
                points = [point for point in points
                          if math.dist(point, weight.target) > 170]
                layout.live_obstacles = [item for item in layout.live_obstacles
                                         if math.dist((item["x"], item["y"]), weight.target) > 170]
            # A close return that cannot be confidently projected onto the
            # arena is not, by itself, grounds to abandon the route. Continue
            # following the planned path; coherent mapped clusters below still
            # trigger a stop-and-replan, and RouteFollower retains the very-close
            # emergency stop for an immediate hazard.
            if layout.add_live_obstacles(points):
                self.mission_view.refresh_live_obstacles()
            current = local_to_mission(layout.start, layout.heading_deg, (model.x, model.y))
            remaining = layout.route[follower.index:]
            if not layout.route_is_clear_from(current, remaining):
                # Stop BEFORE running A*: even a slow plan or Bluetooth delay
                # must never leave the previous motion command active.
                self.bluetooth.send_command("stop")
                self.recorder.record_command("stop", {})
                if self.route_replans >= 4:
                    self._stop_mission_route("More than four obstacle replans")
                    return
                targets = [item for item in remaining if item.get("target")]
                if remaining and (not targets or targets[-1] is not remaining[-1]):
                    targets.append(remaining[-1])
                try:
                    new_route = layout.replan_from(current, targets)
                    layout.route = new_route
                    new_points = prepare_route(layout, current_local=(model.x, model.y),
                                               current_mission=current)
                except (ValueError, KeyError, TypeError) as exc:
                    self._stop_mission_route(f"No safe detour: {exc}")
                    return
                self.route_replans += 1
                self.route_follower = RouteFollower(new_points, start_pose=(model.x, model.y))
                self.route_pause_until = now + 0.5
                self.route_resume_after_frame = self.route_last_frame_monotonic
                self.mission_view.canvas.update()
                self.mission_view.set_follow_status(
                    f"Replanned around 8×8 obstacle ({self.route_replans}/4)",
                    active=True, waypoint=0,
                )
                self.add_log("INFO", f"Route detour {self.route_replans}: {len(new_route)} waypoints")
                self.recorder.record_log(
                    "MISSION",
                    f"Obstacle detected; stopped and replanned detour {self.route_replans}/4 "
                    f"with {len(new_route)} waypoints",
                )
                return
        decision = follower.step(model.x, model.y, model.theta, front_mm, now,
                                 weight=weight, target_leg=target_leg,
                                 search_waypoint=search_waypoint)
        self.mission_view.set_robot_pose(model.x, model.y, model.theta)
        if decision.done or decision.fault:
            self._stop_mission_route(decision.detail)
            return
        self.bluetooth.send_command("drive_set", left=decision.left, right=decision.right)
        self.recorder.record_command("drive_set", {"left": decision.left, "right": decision.right})
        self.mission_view.set_follow_status(
            f"{decision.state} · {decision.detail}", active=True, waypoint=follower.index
        )

    # =================================================================
    # Wiring
    # =================================================================

    def _build_wiring_tab(self):
        self.wiring_panel = WiringPanel(
            self.hardware_map,
            live_source=self.wiring_live_values,
            format_value=self.wiring_format_value,
        )
        self.wiring_panel.changed.connect(self.on_wiring_changed)
        self.tabs.addTab(self.wiring_panel, "Wiring")

    def wiring_live_values(self) -> dict | None:
        """Latest telemetry, or None once the link has gone quiet.

        The telemetry dict keeps its last values after a disconnect, and
        showing those as "Live" would be a lie.
        """
        if (
            self.last_telemetry_monotonic is None
            or time.monotonic() - self.last_telemetry_monotonic > 2.0
        ):
            return None
        return self.telemetry

    def wiring_format_value(self, name: str, value: Any, unit: bool = True) -> str:
        text = self.format_value(value)
        if not unit or value is None:
            return text
        _, _, unit_text = self._telemetry_metadata(name)
        return f"{text} {unit_text}".strip()

    def signal_display_name(self, name: str) -> str:
        """Plot-picker text: the human label, with the raw key alongside."""
        _, label, _ = self._telemetry_metadata(name)
        return name if label == name else f"{label}  ({name})"

    def apply_device_names(self):
        tof_device = self.hardware_map.device_for_signal("tof.8x8")
        self.tof_view.set_device_name(
            tof_device.name if tof_device else "8×8 ToF array"
        )

    def on_wiring_changed(self):
        self.apply_device_names()
        self.arena_view.refresh_from_wiring()

        # Relabel everything already on screen; new signals pick the
        # names up as they arrive.
        for name in self.telemetry_rows:
            self._update_telemetry_identity(name)

        for index in range(self.plot_signal_combo.count()):
            name = self.plot_signal_combo.itemData(index)
            if name:
                self.plot_signal_combo.setItemText(
                    index, self.signal_display_name(name)
                )

        legend = self.plot_widget.getPlotItem().legend
        for index in range(self.active_plot_list.count()):
            item = self.active_plot_list.item(index)
            name = item.data(Qt.ItemDataRole.UserRole)
            if not name:
                continue
            _, label, _ = self._telemetry_metadata(name)
            item.setText(label)
            curve = self.plot_curves.get(name)
            if legend is not None and curve is not None:
                legend_label = legend.getLabel(curve)
                if legend_label is not None:
                    legend_label.setText(label)

    def _build_parameter_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        top = QHBoxLayout()

        title = QLabel(
            "Robot Parameters"
        )

        title.setObjectName("sectionTitle")

        top.addWidget(
            title
        )

        top.addStretch()

        self.refresh_definitions_button = QPushButton(
            "Refresh From Robot"
        )

        top.addWidget(
            self.refresh_definitions_button
        )

        layout.addLayout(
            top
        )

        info = QLabel(
            "Parameters advertised by the robot appear here automatically."
        )

        layout.addWidget(
            info
        )

        self.parameter_scroll = QScrollArea()

        self.parameter_scroll.setWidgetResizable(
            True
        )

        self.parameter_container = QWidget()

        self.parameter_layout = QVBoxLayout(
            self.parameter_container
        )

        self.parameter_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )

        self.parameter_scroll.setWidget(
            self.parameter_container
        )

        layout.addWidget(
            self.parameter_scroll
        )

        self.tabs.addTab(
            page,
            "Parameters",
        )

    # =================================================================
    # Commands
    # =================================================================

    def _build_command_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        top = QHBoxLayout()

        title = QLabel(
            "Robot Commands"
        )

        title.setObjectName("sectionTitle")

        top.addWidget(
            title
        )

        top.addStretch()

        self.stop_button = QPushButton(
            "STOP ROBOT"
        )

        self.stop_button.setObjectName(
            "danger"
        )

        self.stop_button.setEnabled(
            False
        )

        top.addWidget(
            self.stop_button
        )

        layout.addLayout(
            top
        )

        info = QLabel(
            "Commands advertised by the robot are generated here automatically."
        )
        info.setObjectName("hint")

        layout.addWidget(
            info
        )

        self.command_scroll = QScrollArea()

        self.command_scroll.setWidgetResizable(
            True
        )

        self.command_container = QWidget()

        self.command_layout = QVBoxLayout(
            self.command_container
        )

        self.command_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )

        self.command_scroll.setWidget(
            self.command_container
        )

        layout.addWidget(
            self.command_scroll
        )

        self.tabs.addTab(
            page,
            "Commands",
        )

    # =================================================================
    # Logs
    # =================================================================

    def _build_log_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addStretch()

        self.clear_log_button = QPushButton(
            "Clear"
        )

        controls.addWidget(
            self.clear_log_button
        )

        layout.addLayout(
            controls
        )

        self.log_console = QTextEdit()

        self.log_console.setReadOnly(
            True
        )

        self.log_console.setFont(
            theme.monospace_font(11)
        )

        layout.addWidget(
            self.log_console
        )

        self.tabs.addTab(
            page,
            "Logs",
        )

    # =================================================================
    # Raw Serial
    # =================================================================

    def _build_raw_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        self.raw_pause_checkbox = QCheckBox(
            "Pause display"
        )

        controls.addWidget(
            self.raw_pause_checkbox
        )

        controls.addStretch()

        self.clear_raw_button = QPushButton(
            "Clear"
        )

        controls.addWidget(
            self.clear_raw_button
        )

        layout.addLayout(
            controls
        )

        self.raw_console = QTextEdit()

        self.raw_console.setReadOnly(
            True
        )

        self.raw_console.setFont(
            theme.monospace_font(11)
        )

        layout.addWidget(
            self.raw_console
        )

        self.tabs.addTab(
            page,
            "Raw Serial",
        )

    # =================================================================
    # Signals
    # =================================================================

    def _connect_signals(self):
        self.refresh_button.clicked.connect(
            self.refresh_ports
        )

        self.connect_button.clicked.connect(
            self.toggle_connection
        )

        self.record_button.clicked.connect(
            self.toggle_recording
        )

        self.log_fault_button.clicked.connect(
            self.log_fault
        )

        self.enter_debug_button.clicked.connect(
            lambda: self.execute_command(
                "set_debug_mode",
                {
                    "enabled": True,
                },
            )
        )

        self.exit_debug_button.clicked.connect(
            lambda: self.execute_command(
                "set_debug_mode",
                {
                    "enabled": False,
                },
            )
        )

        self.stop_button.clicked.connect(
            lambda: self.execute_command(
                "stop",
                {},
            )
        )

        self.dashboard_stop_button.clicked.connect(
            lambda: self.execute_command(
                "stop",
                {},
            )
        )

        self.drive_arm_checkbox.toggled.connect(self._on_drive_armed_changed)
        self.drive_speed_slider.valueChanged.connect(self._on_drive_speed_changed)
        self.drive_left_scale.valueChanged.connect(lambda _value: self._on_drive_scale_changed())
        self.drive_right_scale.valueChanged.connect(lambda _value: self._on_drive_scale_changed())

        self.refresh_definitions_button.clicked.connect(
            self.bluetooth.request_definitions
        )

        self.clear_telemetry_button.clicked.connect(
            self.clear_telemetry
        )

        self.add_plot_button.clicked.connect(
            self.add_selected_plot
        )

        self.remove_plot_button.clicked.connect(
            self.remove_selected_plot
        )

        self.clear_plot_button.clicked.connect(
            self.clear_plots
        )


        self.clear_log_button.clicked.connect(
            self.log_console.clear
        )

        self.clear_raw_button.clicked.connect(
            self.raw_console.clear
        )

        self.bluetooth.connection_changed.connect(
            self.on_connection_changed
        )

        self.bluetooth.telemetry_received.connect(
            self.on_telemetry
        )

        self.bluetooth.telemetry_definition_received.connect(
            self.on_telemetry_definition
        )

        self.bluetooth.parameter_definition_received.connect(
            self.on_parameter_definition
        )

        self.bluetooth.parameter_value_received.connect(
            self.on_parameter_value
        )

        self.bluetooth.command_definition_received.connect(
            self.on_command_definition
        )

        self.bluetooth.log_received.connect(
            self.on_log
        )

        self.bluetooth.raw_received.connect(
            self.on_raw
        )

        self.bluetooth.error_received.connect(
            self.on_error
        )

        self.bluetooth.robot_state_received.connect(
            self.on_robot_state
        )

    def update_link_health(self):
        now = time.monotonic()
        cutoff = now - 1.0

        while self.raw_line_times and self.raw_line_times[0] < cutoff:
            self.raw_line_times.popleft()

        while (
            self.telemetry_event_times
            and self.telemetry_event_times[0] < cutoff
        ):
            self.telemetry_event_times.popleft()

        if self.last_telemetry_monotonic is None:
            age_text = "—"
        else:
            age_text = (
                f"{now - self.last_telemetry_monotonic:.2f} s"
            )

        # An open serial device is not proof that the radio is delivering
        # telemetry. Make this explicit while the backend retries its hello.
        if self.bluetooth.is_connected() and self.connected_since_monotonic is not None:
            reference = self.last_telemetry_monotonic or self.connected_since_monotonic
            if now - reference > 4.0:
                self.connection_status.setText("● NO TELEMETRY")
                self.connection_status.setToolTip(
                    "Port is open, but no robot telemetry has arrived. "
                    "The app will retry the handshake automatically."
                )
                theme.set_pill_state(self.connection_status, "bad")
            else:
                self.connection_status.setText(
                    f"● {Path(self.bluetooth.port or '').name}"
                )
                theme.set_pill_state(self.connection_status, "ok")

        # Errors are called out in red once there are any, so a link that
        # is quietly dropping messages doesn't blend into the readout.
        if self.protocol_error_count:
            errors = (
                f"<span style='color:{theme.DANGER};font-weight:600;'>"
                f"Protocol errors {self.protocol_error_count}</span>"
            )
        else:
            errors = "Protocol errors 0"

        self.link_health_label.setText(
            f"Frames/s <b>{len(self.raw_line_times)}</b>"
            f" &nbsp;&nbsp; Signals/s <b>{len(self.telemetry_event_times)}</b>"
            f" &nbsp;&nbsp; Last telemetry <b>{age_text}</b>"
            f" &nbsp;&nbsp; {errors}"
        )

    # =================================================================
    # Ports / connection
    # =================================================================

    def refresh_ports(self):
        ports = self.bluetooth.available_ports()

        current_device = (
            self.port_combo.currentData()
        )

        saved_device = self.settings.value(
            "port",
            "",
        )

        target_device = (
            current_device
            or saved_device
        )

        existing_devices = {
            self.port_combo.itemData(i)
            for i in range(
                self.port_combo.count()
            )
        }

        new_devices = {
            port["device"]
            for port in ports
        }

        if existing_devices == new_devices:
            return

        self.port_combo.blockSignals(
            True
        )

        self.port_combo.clear()

        for port in ports:
            device = port["device"]
            description = port["description"]

            label = (
                f"{device} — {description}"
                if description
                else device
            )

            self.port_combo.addItem(
                label,
                device,
            )

        if target_device:
            index = (
                self.port_combo.findData(
                    target_device
                )
            )

            if index >= 0:
                self.port_combo.setCurrentIndex(
                    index
                )

        self.port_combo.blockSignals(
            False
        )

    def toggle_connection(self):
        if self.bluetooth.is_connected():
            self.bluetooth.disconnect_port()
            return

        port = self.port_combo.currentData()

        if not port:
            QMessageBox.warning(
                self,
                "No Serial Port",
                "No serial port is available.\n\n"
                "Connect the adapter and press Refresh.",
            )
            return

        baudrate = int(
            self.baud_combo.currentData()
        )

        self.settings.setValue(
            "port",
            port,
        )

        self.settings.setValue(
            "baud",
            baudrate,
        )

        self.statusBar().showMessage(
            f"Connecting to {port}..."
        )

        self.connect_button.setEnabled(
            False
        )

        self.bluetooth.connect_port(
            port,
            baudrate,
        )

    def on_connection_changed(
        self,
        connected: bool,
        port: str,
    ):
        self.connect_button.setEnabled(
            True
        )

        if connected:
            self.connected_since_monotonic = time.monotonic()
            self.last_telemetry_monotonic = None
            self.connect_button.setText(
                "Disconnect"
            )

            # Just the device name, not the full path - "/dev/cu.usbmodem
            # 145902401" would blow the pill out to a silly width.
            self.connection_status.setText(
                f"● {Path(port).name or port}"
            )
            self.connection_status.setToolTip(port)
            theme.set_pill_state(self.connection_status, "ok")

            self.port_combo.setEnabled(
                False
            )

            self.baud_combo.setEnabled(
                False
            )

            self.refresh_button.setEnabled(
                False
            )

            self.enter_debug_button.setEnabled(
                True
            )

            self.exit_debug_button.setEnabled(
                True
            )

            self.stop_button.setEnabled(
                True
            )

            self.dashboard_stop_button.setEnabled(
                True
            )

            self.record_button.setEnabled(
                True
            )

            # Still disabled until the user starts recording.
            self.log_fault_button.setEnabled(
                False
            )

            self.statusBar().showMessage(
                f"Connected to {port}"
            )

            self.add_log(
                "SYSTEM",
                f"Connected to {port}",
            )
            self._update_drive_controls()
            self._update_drum_controls()

        else:
            self.connected_since_monotonic = None
            self.last_telemetry_monotonic = None
            self._stop_mission_route("Disconnected", send_stop=False)
            self.route_last_frame_monotonic = None
            self.robot_debug_mode = False
            self._stop_drum_hold()
            self.drum_command_available = False
            self._set_drive_armed(False)
            self._update_drive_controls()
            self._update_drum_controls()
            if self.recorder.is_recording:
                self.stop_recording()

            self.connect_button.setText(
                "Connect"
            )

            self.connection_status.setText(
                "DISCONNECTED"
            )
            self.connection_status.setToolTip("")
            theme.set_pill_state(self.connection_status, "bad")

            self.port_combo.setEnabled(
                True
            )

            self.baud_combo.setEnabled(
                True
            )

            self.refresh_button.setEnabled(
                True
            )

            self.enter_debug_button.setEnabled(
                False
            )

            self.exit_debug_button.setEnabled(
                False
            )

            self.stop_button.setEnabled(
                False
            )

            self.dashboard_stop_button.setEnabled(
                False
            )

            self.record_button.setEnabled(
                False
            )

            self.log_fault_button.setEnabled(
                False
            )

            self.statusBar().showMessage(
                "Disconnected"
            )

            self.add_log(
                "SYSTEM",
                "Disconnected",
            )

    # =================================================================
    # Telemetry
    # =================================================================

    def on_telemetry_definition(self, definition: dict):
        name = str(definition.get("name", ""))
        if not name or self._inactive_sensor_signal(name):
            return

        self.telemetry_definitions[name] = dict(definition)

        # A value may have arrived just before its definition. Refresh the
        # human-friendly columns without waiting for the next packet.
        if name in self.telemetry_rows:
            self._update_telemetry_identity(name)

    def _telemetry_metadata(self, name: str) -> tuple[str, str, str]:
        definition = self.telemetry_definitions.get(name, {})
        fallback_group = name.split(".", 1)[0].replace("_", " ").title()
        fallback_label = name.replace("_", " ").replace(".", " › ").title()
        label = str(definition.get("label", fallback_label))
        return (
            str(definition.get("group", fallback_group)),
            self.hardware_map.display_label(name, label),
            str(definition.get("unit", "")),
        )

    def _update_telemetry_identity(self, name: str):
        row = self.telemetry_rows[name]
        group, label, unit = self._telemetry_metadata(name)
        self.telemetry_table.setItem(row, 0, QTableWidgetItem(group))

        signal_item = QTableWidgetItem(label)
        signal_item.setToolTip(name)
        self.telemetry_table.setItem(row, 1, signal_item)
        self.telemetry_table.setItem(row, 3, QTableWidgetItem(unit))

    @staticmethod
    def _inactive_sensor_signal(name: str) -> bool:
        # These sensors are not installed on the current robot. Firmware may
        # still advertise them, but they should not fill the live UI/plots.
        return name.startswith(("ir.", "colour."))

    @staticmethod
    def _matrix_coordinates(name: str) -> tuple[int, int] | None:
        prefix = "tof.array.r"
        if not name.startswith(prefix):
            return None
        suffix = name[len(prefix):]
        row_text, separator, column_text = suffix.partition("c")
        if not separator or not row_text.isdigit() or not column_text.isdigit():
            return None
        row, column = int(row_text), int(column_text)
        if not (0 <= row < 8 and 0 <= column < 8):
            return None
        return row, column

    def on_telemetry(
        self,
        name: str,
        value: Any,
        robot_timestamp: Any,
    ):
        if self._inactive_sensor_signal(name):
            return
        now = time.monotonic()
        self.telemetry_event_times.append(now)
        self.last_telemetry_monotonic = now

        self.recorder.record_telemetry(
            name,
            value,
            robot_timestamp,
        )

        self.telemetry[
            name
        ] = value
        # ArenaView groups all signals sharing one robot timestamp into a
        # coherent pose/range frame. Feed it before matrix zones are hidden
        # from the large dashboard table below.
        self.arena_view.receive_telemetry(name, value, robot_timestamp)
        if name == "system.uptime_ms" and robot_timestamp is not None:
            self.arena_view.model.finish_frame()
            self.route_last_frame_monotonic = now
            self.arena_view._refresh()
        if name == "navigation.state":
            self.mission_view.update_live_candidates(self.arena_view.model)

        if (
            isinstance(
                value,
                (int, float),
            )
            and not isinstance(
                value,
                bool,
            )
        ):
            if (
                name
                not in self.telemetry_history
            ):
                self.telemetry_history[
                    name
                ] = deque(
                    maxlen=self.MAX_HISTORY_POINTS
                )

            elapsed = (
                now
                - self.start_time
            )

            self.telemetry_history[
                name
            ].append(
                (
                    elapsed,
                    float(value),
                )
            )

            if (
                self.plot_signal_combo.findData(
                    name
                )
                < 0
            ):
                self.plot_signal_combo.addItem(
                    self.signal_display_name(name),
                    name,
                )

        if self._matrix_coordinates(name) is not None:
            # The 64 zones feed the depth camera rather than 64 dashboard
            # rows. Their history above still makes every zone plottable.
            return

        if name == "tof.array_valid_zones":
            # The firmware sends this straight after the 64 zones, so it
            # marks a complete frame.
            self.tof_view.notify_frame()

        if name not in self.telemetry_rows:
            row = (
                self.telemetry_table.rowCount()
            )

            self.telemetry_table.insertRow(
                row
            )

            self.telemetry_rows[
                name
            ] = row

            self._update_telemetry_identity(name)

        row = self.telemetry_rows[
            name
        ]

        self.telemetry_table.setItem(
            row,
            2,
            QTableWidgetItem(
                self.format_value(
                    value
                )
            ),
        )

        self.telemetry_table.setItem(
            row,
            4,
            QTableWidgetItem(
                time.strftime(
                    "%H:%M:%S"
                )
            ),
        )

    @staticmethod
    def format_value(
        value: Any,
    ) -> str:
        if value is None:
            return "INVALID"

        if isinstance(
            value,
            float,
        ):
            return f"{value:.6g}"

        if isinstance(
            value,
            bool,
        ):
            return (
                "TRUE"
                if value
                else "FALSE"
            )

        return str(value)

    def clear_telemetry(self):
        self.telemetry.clear()
        self.telemetry_rows.clear()
        self.telemetry_history.clear()

        self.telemetry_table.setRowCount(
            0
        )

        self.plot_signal_combo.clear()

        self.clear_plots()

        self.tof_view.reset()

        self.start_time = time.monotonic()

    # =================================================================
    # Live plotting
    # =================================================================

    def add_selected_plot(self):
        name = (
            self.plot_signal_combo.currentData()
        )

        self.add_signal_to_plot(str(name or ""))

    def plot_matrix_zone(self, name: str):
        self.add_signal_to_plot(name)
        self.tabs.setCurrentWidget(self.plot_page)

    def add_signal_to_plot(self, name: str):

        if not name:
            return

        if name in self.plot_curves:
            return

        # Cycle through the theme's curve colours rather than letting
        # every trace default to the same white - with four or five
        # signals up at once they're otherwise impossible to tell apart.
        pen = pg.mkPen(
            theme.plot_colour(
                self.plot_colour_index
            ),
            width=2,
        )

        self.plot_colour_index += 1

        _, label, _ = self._telemetry_metadata(name)

        curve = self.plot_widget.plot(
            [],
            [],
            name=label,
            pen=pen,
        )

        self.plot_curves[
            name
        ] = curve

        # Show the human label, but keep the telemetry key on the item -
        # the label can change under it when the wiring map is edited.
        item = QListWidgetItem(label)
        item.setData(Qt.ItemDataRole.UserRole, name)
        item.setToolTip(name)
        self.active_plot_list.addItem(
            item
        )

    def remove_selected_plot(self):
        selected = (
            self.active_plot_list.selectedItems()
        )

        for item in selected:
            name = item.data(Qt.ItemDataRole.UserRole) or item.text()

            curve = self.plot_curves.pop(
                name,
                None,
            )

            if curve is not None:
                self.plot_widget.removeItem(
                    curve
                )

            self.active_plot_list.takeItem(
                self.active_plot_list.row(
                    item
                )
            )

    def clear_plots(self):
        for curve in (
            self.plot_curves.values()
        ):
            self.plot_widget.removeItem(
                curve
            )

        self.plot_curves.clear()

        self.plot_colour_index = 0

        self.active_plot_list.clear()

    def update_plot(self):
        if not self.plot_curves:
            return

        now = (
            time.monotonic()
            - self.start_time
        )

        window = float(
            self.time_window_combo.currentData()
        )

        minimum_time = (
            now
            - window
        )

        for name, curve in (
            self.plot_curves.items()
        ):
            history = (
                self.telemetry_history.get(
                    name
                )
            )

            if not history:
                continue

            x = []
            y = []

            for timestamp, value in history:
                if timestamp >= minimum_time:
                    x.append(
                        timestamp
                        - now
                    )

                    y.append(
                        value
                    )

            curve.setData(
                x,
                y,
            )

        self.plot_widget.setXRange(
            -window,
            0,
            padding=0,
        )

    # =================================================================
    # Parameters
    # =================================================================

    def on_parameter_definition(
        self,
        definition: dict,
    ):
        name = str(
            definition.get(
                "name",
                "",
            )
        )

        if not name:
            return

        self.parameter_definitions[name] = dict(definition)
        if "value" in definition:
            self.parameter_values[name] = definition["value"]
        elif (
            "default" in definition
            and name not in self.parameter_values
        ):
            self.parameter_values[name] = definition["default"]

        if name in self.parameter_editors:
            if "value" in definition:
                self.parameter_editors[
                    name
                ].set_value(
                    definition[
                        "value"
                    ]
                )
            return

        group = QGroupBox(
            str(
                definition.get(
                    "label",
                    name,
                )
            )
        )

        layout = QVBoxLayout(
            group
        )

        description = definition.get(
            "description"
        )

        if description:
            text = QLabel(
                str(description)
            )

            text.setWordWrap(
                True
            )

            layout.addWidget(
                text
            )

        editor = ValueEditor(
            definition
        )

        self.parameter_editors[
            name
        ] = editor

        layout.addWidget(
            editor
        )

        unit = definition.get(
            "unit"
        )

        if unit:
            layout.addWidget(
                QLabel(
                    f"Unit: {unit}"
                )
            )

        apply_button = QPushButton(
            "Apply"
        )

        apply_button.clicked.connect(
            lambda checked=False,
            parameter_name=name,
            parameter_editor=editor:
            self.apply_parameter(
                parameter_name,
                parameter_editor,
            )
        )

        layout.addWidget(
            apply_button
        )

        self.parameter_layout.addWidget(
            group
        )

    def apply_parameter(
        self,
        name: str,
        editor: ValueEditor,
    ):
        value = editor.value()
        self.parameter_values[name] = value

        self.recorder.record_parameter(
            name,
            value,
        )

        self.bluetooth.set_parameter(
            name,
            value,
        )

        self.add_log(
            "TX",
            f"{name} = {value}",
        )

    def on_parameter_value(
        self,
        name: str,
        value: Any,
    ):
        self.parameter_values[name] = value

        editor = (
            self.parameter_editors.get(
                name
            )
        )

        if editor is not None:
            editor.set_value(
                value
            )

    # =================================================================
    # Commands
    # =================================================================

    def on_command_definition(
        self,
        definition: dict,
    ):
        name = str(
            definition.get(
                "name",
                "",
            )
        )

        if not name:
            return

        if name == "drive_set":
            self.drive_command_available = True
            self._update_drive_controls()
        if name == "drum_set":
            self.drum_command_available = True
            self._update_drum_controls()

        if name not in self.command_widgets:
            command_widget = CommandWidget(
                definition,
                self.execute_command,
            )

            self.command_widgets[
                name
            ] = command_widget

            self.command_layout.addWidget(
                command_widget
            )

        if (
            name
            not in self.dashboard_command_widgets
        ):
            dashboard_widget = CommandWidget(
                definition,
                self.execute_command,
            )

            self.dashboard_command_widgets[
                name
            ] = dashboard_widget

            self.dashboard_command_layout.addWidget(
                dashboard_widget
            )

    def execute_command(
        self,
        name: str,
        arguments: dict,
    ):
        if name == "stop":
            self._stop_mission_route("STOP pressed", send_stop=False)
        elif name in ("drive_set", "navigation_set"):
            self._stop_mission_route("Another drive command took control")
        elif name == "set_debug_mode" and arguments.get("enabled") is False:
            self._stop_mission_route("Debug Mode exited")
        if name in ("stop", "set_debug_mode") and (
            name == "stop" or arguments.get("enabled") is False
        ):
            self._stop_drum_hold()
        self.recorder.record_command(
            name,
            arguments,
        )

        self.bluetooth.send_command(
            name,
            **arguments,
        )

        if arguments:
            argument_text = ", ".join(
                f"{key}={value}"
                for key, value
                in arguments.items()
            )

            text = (
                f"{name}("
                f"{argument_text}"
                f")"
            )
        else:
            text = (
                f"{name}()"
            )

        self.add_log(
            "TX",
            text,
        )

    # =================================================================
    # Logs / state / raw
    # =================================================================

    def on_log(
        self,
        level: str,
        message: str,
    ):
        self.recorder.record_log(
            level,
            message,
        )

        self.add_log(
            level,
            message,
        )

    def add_log(
        self,
        level: str,
        message: str,
    ):
        timestamp = time.strftime(
            "%H:%M:%S"
        )

        # Colour by level so a warning or an error stands out in a wall of
        # scrolling info lines. Everything is escaped first - log text
        # comes off the wire and may well contain '<' or '&'.
        colour = LOG_LEVEL_COLOURS.get(
            level.upper(),
            theme.TEXT,
        )

        self.log_console.append(
            f'<span style="color:{theme.TEXT_FAINT};">[{timestamp}]</span> '
            f'<span style="color:{colour};font-weight:600;">'
            f"[{html.escape(level)}]</span> "
            f'<span style="color:{colour};">{html.escape(message)}</span>'
        )

    def on_raw(
        self,
        text: str,
    ):
        self.raw_line_times.append(
            time.monotonic()
        )

        # Recording is independent of whether the Raw Serial tab is paused.
        self.recorder.record_raw(
            text
        )

        if self.raw_pause_checkbox.isChecked():
            return

        self.raw_console.append(
            text
        )

    def on_robot_state(
        self,
        state: dict,
    ):
        self.recorder.record_state(
            state
        )

        debug_enabled = state.get("debug_mode")
        if debug_enabled is not None:
            self.robot_debug_mode = bool(debug_enabled)
            if not self.robot_debug_mode:
                self._stop_mission_route("Debug Mode exited", send_stop=False)
            self._update_drive_controls()
            self._update_drum_controls()

        if debug_enabled is True:
            self.statusBar().showMessage(
                "Robot is in DEBUG mode"
            )

        elif debug_enabled is False:
            self.statusBar().showMessage(
                "Robot is in NORMAL mode"
            )

    def on_error(
        self,
        message: str,
    ):
        self.protocol_error_count += 1

        self.add_log(
            "ERROR",
            message,
        )

        self.statusBar().showMessage(
            message
        )

    # =================================================================
    # Shutdown
    # =================================================================

    def closeEvent(
        self,
        event,
    ):
        if self.recorder.is_recording:
            self.stop_recording()

        self._set_drive_armed(False)
        self._stop_mission_route("App closing")
        self._stop_drum_hold()
        self.bluetooth.disconnect_port()

        event.accept()


def main():
    app = QApplication(
        sys.argv
    )

    app.setApplicationName(
        "Robot Debug Console"
    )

    theme.apply(
        app
    )

    window = RobotDebugGUI()

    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()
