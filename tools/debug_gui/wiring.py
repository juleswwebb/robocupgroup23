"""
wiring.py

Hardware wiring map: which physical port every sensor and actuator is
plugged into, and a human name for each ("Top right ToF" -> XSHUT0).

The port catalogue below mirrors include/sensor_config.h. The firmware
already reads every port that lists signals and reports its telemetry
keyed by port (tof.xshut0, ir.2, ...), so after re-plugging a sensor only
its port needs changing here - the human name then follows the data. A
port without signals exists on the Teensy but the current firmware doesn't
read it, and the map says so rather than pretending.

The map saves to hardware_map.json beside this file on every edit. It is
deliberately not gitignored: the whole group works on one robot.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

import theme


MAP_VERSION = 1


# =====================================================================
# Port catalogue (mirror of include/sensor_config.h)
# =====================================================================

@dataclass(frozen=True)
class Port:
    id: str
    label: str
    port_type: str
    # (telemetry key, short label) pairs the firmware sends for this port.
    # Empty means the port exists but the current firmware doesn't read it.
    signals: tuple[tuple[str, str], ...] = ()
    # Device kind the firmware drives this port as, if it cares.
    expects: str | None = None
    # Teensy pins the port occupies, for catching two things on one pin.
    pins: tuple[str, ...] = ()
    # Ports on the same bus legitimately share pins (several I2C devices
    # on SDA/SCL), so pin clashes are only reported across buses.
    bus: str | None = None
    # Extra telemetry keys beginning with this prefix also belong here
    # (the 8x8 array's 64 per-zone signals).
    signal_prefix: str | None = None

    @property
    def is_read(self) -> bool:
        return bool(self.signals)


@dataclass(frozen=True)
class DeviceKind:
    key: str
    label: str
    # None means "any port, or free text" - for hardware the firmware
    # doesn't implement yet, like the drive motors.
    port_type: str | None


KINDS: dict[str, DeviceKind] = {
    kind.key: kind
    for kind in (
        DeviceKind("vl53l0x", "VL53L0X ToF", "xshut"),
        DeviceKind("vl53l1x", "VL53L1X ToF", "xshut"),
        DeviceKind("tof_8x8", "8×8 ToF array", "i2c"),
        DeviceKind("serial_tof", "Serial ToF", "uart"),
        DeviceKind("ir", "IR distance", "pin"),
        DeviceKind("ultrasonic", "Ultrasonic", "ultrasonic"),
        DeviceKind("colour", "Colour sensor", "i2c"),
        DeviceKind("imu", "IMU", "i2c"),
        DeviceKind("optical_flow", "Optical flow", "spi"),
        DeviceKind("inductive", "Inductive proximity", "pin"),
        DeviceKind("encoder", "Wheel encoder", "encoder"),
        DeviceKind("servo", "Servo", "pwm"),
        DeviceKind("motor", "Drive motor", None),
        DeviceKind("other", "Other", None),
    )
}

PORT_TYPE_LABELS = {
    "xshut": "XSHUT (VL53 via SX1509)",
    "i2c": "I2C",
    "uart": "Serial (UART)",
    "pin": "Analog / digital pins",
    "ultrasonic": "Ultrasonic pairs",
    "spi": "SPI",
    "encoder": "Encoder pairs",
    "pwm": "PWM",
}


def _build_ports() -> list[Port]:
    ports: list[Port] = []

    # Seven VL53s, reset lines on the SX1509 - not Teensy GPIO - and all
    # sharing I2C bus 0. Which type sits on which line is fixed in
    # DistanceSensors.cpp.
    vl53_types = ["vl53l0x", "vl53l0x", "vl53l1x", "vl53l1x", "vl53l1x", "vl53l1x", "vl53l0x"]
    for n, expects in enumerate(vl53_types):
        ports.append(Port(
            f"xshut{n}", f"XSHUT{n}  ·  SX1509 IO{n}", "xshut",
            signals=((f"tof.xshut{n}", "Distance"),),
            expects=expects, pins=("D18", "D19"), bus="wire",
        ))

    i2c_devices = {
        0x28: ("imu", (
            ("imu.heading", "Heading"), ("imu.roll", "Roll"), ("imu.pitch", "Pitch"),
            ("imu.cal_system", "System cal"), ("imu.cal_gyro", "Gyro cal"),
            ("imu.cal_accel", "Accel cal"), ("imu.cal_mag", "Mag cal"),
        )),
        0x29: ("colour", (
            ("colour.r", "Red"), ("colour.g", "Green"),
            ("colour.b", "Blue"), ("colour.c", "Clear"),
        )),
        0x33: ("tof_8x8", (
            ("tof.8x8", "Centre"), ("tof.array_min", "Nearest"),
            ("tof.array_valid_zones", "Valid zones"),
        )),
    }
    buses = (("wire", "Wire", ("D18", "D19")),
             ("wire1", "Wire1", ("D16", "D17")),
             ("wire2", "Wire2", ("D24", "D25")))
    # Only Wire1 is read for these devices today.
    read_bus = "wire1"
    for bus_id, bus_name, pins in buses:
        for address, (kind, signals) in i2c_devices.items():
            is_read = bus_id == read_bus
            ports.append(Port(
                f"{bus_id}_0x{address:02x}",
                f"{bus_name} @ 0x{address:02X}  ·  pins {pins[0][1:]}/{pins[1][1:]}",
                "i2c",
                signals=signals if is_read else (),
                expects=kind if is_read else None,
                pins=pins, bus=bus_id,
                signal_prefix="tof.array." if (is_read and kind == "tof_8x8") else None,
            ))

    # Teensy 4.0 hardware serial ports (RX, TX).
    uarts = {1: (0, 1), 2: (7, 8), 3: (15, 14), 4: (16, 17),
             5: (21, 20), 6: (25, 24), 7: (28, 29), 8: (34, 35)}
    for n, (rx, tx) in uarts.items():
        is_read = n == 2
        ports.append(Port(
            f"serial{n}", f"Serial{n}  ·  RX {rx} / TX {tx}", "uart",
            signals=(("tof.serial", "Distance"),) if is_read else (),
            expects="serial_tof" if is_read else None,
            pins=(f"D{rx}", f"D{tx}"),
        ))

    # A0-A13 are D14-D27 on the Teensy 4.0.
    for n in range(14):
        digital = 14 + n
        signals: tuple[tuple[str, str], ...] = ()
        expects = None
        if n == 0:
            signals = (("inductive.detected", "Detected"), ("inductive.count", "Count"))
            expects = "inductive"
        elif 6 <= n <= 9:
            signals = ((f"ir.{n - 6}", "Distance"),)
            expects = "ir"
        ports.append(Port(
            f"a{n}", f"A{n}  ·  D{digital}", "pin",
            signals=signals, expects=expects, pins=(f"D{digital}",),
        ))

    for index, (trig, echo) in enumerate(((30, 31), (32, 33))):
        ports.append(Port(
            f"ultrasonic_{trig}_{echo}", f"D{trig} trig / D{echo} echo", "ultrasonic",
            signals=((f"ultrasonic.{index}", "Distance"),),
            expects="ultrasonic", pins=(f"D{trig}", f"D{echo}"),
        ))

    ports.append(Port(
        "spi_cs10", "SPI  ·  CS 10 (MOSI 11 / MISO 12 / SCK 13)", "spi",
        signals=(("flow.dx", "ΔX"), ("flow.dy", "ΔY"),
                 ("flow.total_x", "Total X"), ("flow.total_y", "Total Y")),
        expects="optical_flow", pins=("D10", "D11", "D12", "D13"), bus="spi",
    ))

    for index, (a, b) in enumerate(((2, 3), (4, 5))):
        ports.append(Port(
            f"encoder_{a}_{b}", f"D{a} A / D{b} B", "encoder",
            signals=((f"encoder.{index}", "Position"),),
            expects="encoder", pins=(f"D{a}", f"D{b}"),
        ))

    # The firmware currently drives the same servo pulse on both pins,
    # because nobody knows yet which conductor is the signal.
    for pin in (28, 29):
        ports.append(Port(
            f"d{pin}", f"D{pin}", "pwm",
            signals=(("servo.us", "Pulse"),),
            expects="servo", pins=(f"D{pin}",),
        ))

    return ports


PORTS: list[Port] = _build_ports()
PORTS_BY_ID: dict[str, Port] = {port.id: port for port in PORTS}


def ports_for_kind(kind_key: str) -> list[Port]:
    kind = KINDS.get(kind_key)
    if kind is None or kind.port_type is None:
        return list(PORTS)
    return [port for port in PORTS if port.port_type == kind.port_type]


# =====================================================================
# Model
# =====================================================================

@dataclass
class Device:
    name: str
    kind: str
    # A catalogue port id, or free text for kinds without a port type.
    port: str

    def to_json(self) -> dict:
        return {"name": self.name, "kind": self.kind, "port": self.port}


def default_devices() -> list[Device]:
    """The wiring sensor_config.h currently assumes, with plain names."""
    devices = [
        Device(f"ToF XSHUT{n}", "vl53l0x" if n in (0, 1, 6) else "vl53l1x", f"xshut{n}")
        for n in range(7)
    ]
    devices += [
        Device("8×8 ToF array", "tof_8x8", "wire1_0x33"),
        Device("Serial ToF", "serial_tof", "serial2"),
    ]
    devices += [Device(f"IR {n}", "ir", f"a{n + 6}") for n in range(4)]
    devices += [
        Device("Ultrasonic 0", "ultrasonic", "ultrasonic_30_31"),
        Device("Ultrasonic 1", "ultrasonic", "ultrasonic_32_33"),
        Device("Colour sensor", "colour", "wire1_0x29"),
        Device("IMU", "imu", "wire1_0x28"),
        Device("Optical flow", "optical_flow", "spi_cs10"),
        Device("Inductive sensor", "inductive", "a0"),
        Device("Encoder 0", "encoder", "encoder_2_3"),
        Device("Encoder 1", "encoder", "encoder_4_5"),
        Device("Servo", "servo", "d28"),
    ]
    return devices


# Status states match theme.set_pill_state: "ok", "bad", "busy", "".
Status = tuple[str, str]


class HardwareMap:
    def __init__(self, path: Path, devices: list[Device]):
        self.path = path
        self.devices = devices
        self._signal_index: dict[str, tuple[Device, Port]] = {}
        self._prefix_index: list[tuple[str, Device, Port]] = []
        self.reindex()

    @staticmethod
    def default_path() -> Path:
        return Path(__file__).resolve().parent / "hardware_map.json"

    # ---------------- persistence ----------------

    @classmethod
    def load(cls, path: Path) -> tuple["HardwareMap", str | None]:
        """
        Load the map, seeding firmware defaults on first run.

        Returns the map and a warning to show the user, if any. A file
        that can't be parsed is set aside rather than overwritten, so a
        bad merge never silently throws away everyone's names.
        """
        if not path.exists():
            hardware_map = cls(path, default_devices())
            hardware_map.save()
            return hardware_map, None

        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
            devices = [
                Device(
                    name=str(item.get("name", "")).strip() or "Unnamed device",
                    kind=str(item.get("kind", "other"))
                    if str(item.get("kind")) in KINDS else "other",
                    port=str(item.get("port", "")),
                )
                for item in raw.get("devices", [])
                if isinstance(item, dict)
            ]
            return cls(path, devices), None
        except (OSError, ValueError, AttributeError) as error:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            backup = path.with_name(f"{path.stem}.unreadable-{stamp}.json")
            try:
                path.replace(backup)
                where = f"moved to {backup.name}"
            except OSError:
                where = "left in place"
            hardware_map = cls(path, default_devices())
            hardware_map.save()
            return hardware_map, (
                f"Couldn't read {path.name} ({error}); it was {where} "
                f"and the firmware defaults were loaded instead."
            )

    def save(self):
        """Write atomically, so a crash mid-save can't truncate the file."""
        payload = {
            "version": MAP_VERSION,
            "devices": [device.to_json() for device in self.devices],
        }
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def snapshot(self) -> list[dict]:
        """Plain-data copy for stamping into recordings."""
        snapshot = []
        for device in self.devices:
            port = self.resolve_port(device)
            snapshot.append({
                **device.to_json(),
                "port_label": port.label if port else device.port,
                "signals": [key for key, _ in port.signals] if port else [],
            })
        return snapshot

    # ---------------- lookups ----------------

    @staticmethod
    def resolve_port(device: Device) -> Port | None:
        port = PORTS_BY_ID.get(device.port)
        if port is not None:
            return port
        # Free-text ports may still name a catalogue port by its label.
        text = device.port.strip().lower()
        for candidate in PORTS:
            if candidate.label.lower() == text:
                return candidate
        return None

    def reindex(self):
        self._signal_index.clear()
        self._prefix_index.clear()
        for device in self.devices:
            port = self.resolve_port(device)
            if port is None:
                continue
            for key, _ in port.signals:
                # First device on a port wins; the clash is reported in
                # its status rather than resolved by guessing.
                self._signal_index.setdefault(key, (device, port))
            if port.signal_prefix:
                self._prefix_index.append((port.signal_prefix, device, port))

    def device_for_signal(self, key: str) -> Device | None:
        match = self._signal_index.get(key)
        return match[0] if match else None

    def display_label(self, key: str, fallback: str) -> str:
        """Human label for a telemetry key, or the fallback if unmapped."""
        match = self._signal_index.get(key)
        if match is not None:
            device, port = match
            if len(port.signals) == 1:
                return device.name
            short = dict(port.signals)[key]
            return f"{device.name} · {short}"

        for prefix, device, _ in self._prefix_index:
            if key.startswith(prefix):
                return f"{device.name} · {fallback}"

        return fallback

    def default_port_for(self, kind_key: str, exclude: Device | None = None) -> str:
        """First port suited to this kind that nothing else is using."""
        used = {d.port for d in self.devices if d is not exclude}
        candidates = ports_for_kind(kind_key)
        preferred = [p for p in candidates if p.expects == kind_key] or candidates
        for port in preferred:
            if port.id not in used:
                return port.id
        return preferred[0].id if preferred else ""

    # ---------------- validation ----------------

    def config_status(self, device: Device) -> Status | None:
        """A wiring problem with this device, or None if the wiring is sound."""
        port = self.resolve_port(device)
        kind = KINDS.get(device.kind, KINDS["other"])

        if port is None:
            if not device.port.strip():
                return ("busy", "No port chosen")
            return None

        for other in self.devices:
            if other is device:
                continue
            other_port = self.resolve_port(other)
            if other_port is None:
                continue
            if other_port.id == port.id:
                return ("bad", f"Port also used by “{other.name}”")
            same_bus = port.bus is not None and port.bus == other_port.bus
            if not same_bus:
                clash = sorted(set(port.pins) & set(other_port.pins))
                if clash:
                    return ("bad", f"Pin {clash[0]} also used by “{other.name}”")

        if kind.port_type is not None:
            if not port.is_read:
                return ("busy", "Firmware doesn't read this port — see sensor_config.h")
            if port.expects and port.expects != device.kind:
                return ("busy", f"Firmware drives this port as a {KINDS[port.expects].label}")
        elif port.expects:
            return ("busy", f"Firmware reads this port as a {KINDS[port.expects].label}")

        return None

    def status(self, device: Device, live: dict | None) -> Status:
        problem = self.config_status(device)
        if problem is not None:
            return problem

        port = self.resolve_port(device)
        if port is None or not port.is_read:
            return ("", "Not in firmware telemetry")
        if live is None:
            return ("", "Waiting for data")

        values = [live.get(key) for key, _ in port.signals if key in live]
        if not values:
            return ("", "Waiting for data")
        if all(value is None for value in values):
            return ("bad", "No valid reading")
        return ("ok", "Live")


# =====================================================================
# Wiring tab
# =====================================================================

STATE_COLOURS = {
    "ok": theme.SUCCESS,
    "bad": theme.DANGER,
    "busy": theme.WARNING,
    "": theme.TEXT_MUTED,
}

COL_NAME, COL_KIND, COL_PORT, COL_LIVE, COL_STATUS = range(5)


class WiringPanel(QWidget):
    """Editable table of devices, their ports and their live state."""

    changed = pyqtSignal()

    def __init__(
        self,
        hardware_map: HardwareMap,
        live_source: Callable[[], dict | None],
        format_value: Callable[[str, Any], str],
        parent: QWidget | None = None,
    ):
        super().__init__(parent)

        self.map = hardware_map
        self.live_source = live_source
        self.format_value = format_value
        self._populating = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        top = QHBoxLayout()
        title = QLabel("Wiring")
        title.setObjectName("sectionTitle")
        top.addWidget(title)

        self.issue_pill = QLabel()
        self.issue_pill.setObjectName("statusPill")
        top.addWidget(self.issue_pill)
        top.addStretch()

        self.add_button = QPushButton("Add Device")
        self.add_button.setObjectName("primary")
        self.remove_button = QPushButton("Remove")
        self.reset_button = QPushButton("Reset to Firmware Defaults")
        for button in (self.add_button, self.remove_button, self.reset_button):
            top.addWidget(button)
        layout.addLayout(top)

        hint_row = QHBoxLayout()
        hint = QLabel(
            "Name each device and pick the port it's plugged into. Names "
            "replace port names across the dashboard and plots."
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        hint_row.addWidget(hint, 1)

        self.save_label = QLabel(f"Saves automatically to {self.map.path.name}")
        self.save_label.setObjectName("metric")
        self.save_label.setToolTip(str(self.map.path))
        hint_row.addWidget(self.save_label)
        layout.addLayout(hint_row)

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Name", "Type", "Port", "Live", "Status"])
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked
            | QAbstractItemView.EditTrigger.EditKeyPressed
            | QAbstractItemView.EditTrigger.AnyKeyPressed
        )

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(COL_NAME, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(COL_KIND, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(COL_PORT, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(COL_LIVE, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(COL_STATUS, QHeaderView.ResizeMode.Interactive)
        self.table.setColumnWidth(COL_NAME, 210)
        self.table.setColumnWidth(COL_KIND, 175)
        self.table.setColumnWidth(COL_PORT, 285)
        self.table.setColumnWidth(COL_STATUS, 300)

        layout.addWidget(self.table, 1)

        self.add_button.clicked.connect(self.add_device)
        self.remove_button.clicked.connect(self.remove_device)
        self.reset_button.clicked.connect(self.reset_to_defaults)
        self.table.itemChanged.connect(self._on_item_changed)

        self.rebuild()

        # Live values and statuses only; cheap enough at this rate, and
        # skipped entirely while the tab isn't on screen.
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_live)
        self.refresh_timer.start(250)

    # ---------------- building ----------------

    def rebuild(self):
        self._populating = True
        try:
            self.table.setRowCount(len(self.map.devices))
            for row, device in enumerate(self.map.devices):
                name_item = QTableWidgetItem(device.name)
                name_item.setToolTip("Double-click to rename")
                self.table.setItem(row, COL_NAME, name_item)

                kind_combo = QComboBox()
                for kind in KINDS.values():
                    kind_combo.addItem(kind.label, kind.key)
                kind_combo.setCurrentIndex(max(kind_combo.findData(device.kind), 0))
                kind_combo.currentIndexChanged.connect(
                    lambda _i, d=device, c=kind_combo: self._on_kind_changed(d, c)
                )
                self.table.setCellWidget(row, COL_KIND, kind_combo)

                self.table.setCellWidget(row, COL_PORT, self._make_port_combo(device))

                for column in (COL_LIVE, COL_STATUS):
                    item = QTableWidgetItem()
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    self.table.setItem(row, column, item)
        finally:
            self._populating = False

        self.refresh_live(force=True)

    def _make_port_combo(self, device: Device) -> QComboBox:
        combo = QComboBox()
        kind = KINDS.get(device.kind, KINDS["other"])
        ports = ports_for_kind(device.kind)

        last_type = None
        for port in ports:
            if kind.port_type is None and last_type not in (None, port.port_type):
                combo.insertSeparator(combo.count())
            last_type = port.port_type
            label = port.label if port.is_read else f"{port.label}   (not read)"
            combo.addItem(label, port.id)

        if kind.port_type is None:
            # Motors and the like: pick a known port or type anything.
            combo.setEditable(True)
            combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
            index = combo.findData(device.port)
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setCurrentIndex(-1)
                combo.setEditText(device.port)
            combo.lineEdit().setPlaceholderText("Pick or type a port")
            combo.activated.connect(
                lambda _i, d=device, c=combo: self._on_port_changed(d, c)
            )
            combo.lineEdit().editingFinished.connect(
                lambda d=device, c=combo: self._on_port_changed(d, c)
            )
        else:
            combo.setCurrentIndex(max(combo.findData(device.port), 0))
            combo.currentIndexChanged.connect(
                lambda _i, d=device, c=combo: self._on_port_changed(d, c)
            )

        port = HardwareMap.resolve_port(device)
        combo.setToolTip(
            PORT_TYPE_LABELS.get(port.port_type, "") if port else "Free-text port"
        )
        return combo

    # ---------------- editing ----------------

    def _row_of(self, device: Device) -> int:
        for row, candidate in enumerate(self.map.devices):
            if candidate is device:
                return row
        return -1

    def _on_item_changed(self, item: QTableWidgetItem):
        if self._populating or item.column() != COL_NAME:
            return
        device = self.map.devices[item.row()]
        name = item.text().strip()
        if not name:
            # Refuse blank names rather than leave an unlabelled row.
            self._populating = True
            item.setText(device.name)
            self._populating = False
            return
        if name != device.name:
            device.name = name
            self._commit()

    def _on_kind_changed(self, device: Device, combo: QComboBox):
        if self._populating:
            return
        device.kind = str(combo.currentData())
        valid_ids = {port.id for port in ports_for_kind(device.kind)}
        if KINDS[device.kind].port_type is not None and device.port not in valid_ids:
            device.port = self.map.default_port_for(device.kind, exclude=device)

        row = self._row_of(device)
        self._populating = True
        try:
            self.table.setCellWidget(row, COL_PORT, self._make_port_combo(device))
        finally:
            self._populating = False
        self._commit()

    def _on_port_changed(self, device: Device, combo: QComboBox):
        if self._populating:
            return
        if combo.isEditable():
            text = combo.currentText().strip()
            index = combo.findText(text)
            port = str(combo.itemData(index)) if index >= 0 else text
        else:
            port = str(combo.currentData())
        if port != device.port:
            device.port = port
            self._commit()

    def add_device(self):
        kind = "other"
        row = self.table.currentRow()
        if 0 <= row < len(self.map.devices):
            kind = self.map.devices[row].kind
        device = Device("New device", kind, "")
        device.port = self.map.default_port_for(kind, exclude=device)
        self.map.devices.append(device)
        self.rebuild()
        self._commit()

        new_row = len(self.map.devices) - 1
        self.table.setCurrentCell(new_row, COL_NAME)
        self.table.scrollToItem(self.table.item(new_row, COL_NAME))
        self.table.editItem(self.table.item(new_row, COL_NAME))

    def remove_device(self):
        row = self.table.currentRow()
        if not 0 <= row < len(self.map.devices):
            return
        del self.map.devices[row]
        self.rebuild()
        self._commit()
        if self.map.devices:
            self.table.setCurrentCell(min(row, len(self.map.devices) - 1), COL_NAME)

    def reset_to_defaults(self):
        answer = QMessageBox.question(
            self,
            "Reset wiring?",
            "Replace every device, name and port with the wiring in "
            "sensor_config.h? Your names will be lost.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.map.devices = default_devices()
        self.rebuild()
        self._commit()

    def _commit(self):
        self.map.reindex()
        try:
            self.map.save()
        except OSError as error:
            self.save_label.setText(f"Couldn't save: {error}")
            self.save_label.setStyleSheet(f"color:{theme.DANGER};")
        else:
            self.save_label.setText(f"Saved {datetime.now():%H:%M:%S}")
            self.save_label.setStyleSheet("")
        self.refresh_live(force=True)
        self.changed.emit()

    # ---------------- live refresh ----------------

    def refresh_live(self, force: bool = False):
        if not force and not self.isVisible():
            return

        live = self.live_source()
        issues = 0

        self._populating = True
        try:
            for row, device in enumerate(self.map.devices):
                port = self.map.resolve_port(device)
                self.table.item(row, COL_LIVE).setText(self._live_text(port, live))

                state, message = self.map.status(device, live)
                if self.map.config_status(device) is not None:
                    issues += 1
                status_item = self.table.item(row, COL_STATUS)
                status_item.setText(f"●  {message}")
                status_item.setForeground(QColor(STATE_COLOURS[state]))
                status_item.setToolTip(message)
        finally:
            self._populating = False

        if issues:
            self.issue_pill.setText(f"{issues} WIRING ISSUE{'S' if issues != 1 else ''}")
            theme.set_pill_state(self.issue_pill, "bad")
        else:
            self.issue_pill.setText(f"{len(self.map.devices)} DEVICES  ·  NO ISSUES")
            theme.set_pill_state(self.issue_pill, "ok")

    def _live_text(self, port: Port | None, live: dict | None) -> str:
        if port is None or not port.is_read or live is None:
            return "—"

        present = [(key, short) for key, short in port.signals if key in live]
        if not present:
            return "—"
        if len(port.signals) == 1:
            key = present[0][0]
            return self.format_value(key, live[key])
        # Compact multi-signal summary; the dashboard has the full set.
        return "   ".join(
            f"{short} {self.format_value(key, live[key], unit=False)}"
            for key, short in present[:4]
        )
