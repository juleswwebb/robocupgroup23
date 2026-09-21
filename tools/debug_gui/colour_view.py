"""
colour_view.py

Dashboard card that turns the TCS34725's raw RGBC counts into an actual
colour you can look at, plus a best-guess colour name and a short history.

Raw counts aren't a displayable colour on their own: the channels have
different sensitivities, the scale depends on gain and integration time,
and the sensor is linear while screens expect gamma-encoded values. So:

- Uncalibrated, each channel is scaled against the brightest one. That
  shows the hue faithfully but says nothing about brightness.
- After "Calibrate White" (sensor held over something white), each channel
  is divided by the white reference. White then shows as white, and a dark
  surface actually shows dark - which is what base-colour detection needs.
"""

from __future__ import annotations

import colorsys
import json
import time
from collections import deque
from typing import Any

from PyQt6.QtCore import QRectF, QSettings, Qt
from PyQt6.QtGui import QColor, QPainter, QPainterPath
from PyQt6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import theme


CHANNEL_KEYS = ("colour.r", "colour.g", "colour.b", "colour.c")
GAMMA = 1 / 2.2
SETTINGS_KEY = "colour/white_reference"


def _channels(telemetry: dict) -> tuple[float, float, float, float] | None:
    values = [telemetry.get(key) for key in CHANNEL_KEYS]
    if any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in values):
        return None
    return tuple(float(v) for v in values)  # type: ignore[return-value]


def linear_channels(
    r: float, g: float, b: float, white: tuple[float, float, float] | None
) -> tuple[float, float, float]:
    """Raw counts -> linear 0-1 channels, against white or the brightest channel."""
    if white is not None:
        scaled = (r / max(white[0], 1.0), g / max(white[1], 1.0), b / max(white[2], 1.0))
    else:
        peak = max(r, g, b, 1.0)
        scaled = (r / peak, g / peak, b / peak)
    return tuple(min(max(channel, 0.0), 1.0) for channel in scaled)  # type: ignore[return-value]


def to_display_rgb(linear: tuple[float, float, float]) -> tuple[int, int, int]:
    """Linear sensor channels -> gamma-encoded 0-255 for the screen."""
    return tuple(int(round(255 * channel ** GAMMA)) for channel in linear)  # type: ignore[return-value]


def colour_name(linear: tuple[float, float, float], calibrated: bool) -> str:
    # Classify on linear values, not the gamma-encoded display colour:
    # gamma lifts dark tones, so a 5% black would otherwise read as grey.
    h, s, v = colorsys.rgb_to_hsv(*linear)
    # Brightness only means something once calibrated; before that every
    # reading is normalised to full brightness.
    if calibrated and v < 0.08:
        return "Black"
    if s < 0.25:
        if not calibrated:
            return "White / grey"
        return "White" if v > 0.6 else "Grey"

    hue = h * 360
    for limit, name in (
        (15, "Red"), (40, "Orange"), (70, "Yellow"), (160, "Green"),
        (195, "Cyan"), (255, "Blue"), (290, "Purple"), (340, "Pink"), (360, "Red"),
    ):
        if hue < limit:
            return name
    return "Red"


class _Swatch(QWidget):
    """Rounded colour block. Painted directly: stylesheet backgrounds
    on plain widgets are unreliable across platforms."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.colour: QColor | None = None
        self.setMinimumSize(96, 96)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def set_colour(self, colour: QColor | None):
        self.colour = colour
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        path = QPainterPath()
        path.addRoundedRect(rect, 10, 10)

        if self.colour is None:
            painter.fillPath(path, QColor(theme.SURFACE_RAISED))
            painter.setPen(QColor(theme.TEXT_FAINT))
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "No\nreading")
        else:
            painter.fillPath(path, self.colour)
        painter.setPen(QColor(theme.BORDER_STRONG))
        painter.drawPath(path)


class _HistoryStrip(QWidget):
    """The last few seconds of colours, oldest on the left."""

    def __init__(self, length: int = 48, parent: QWidget | None = None):
        super().__init__(parent)
        self.colours: deque[QColor | None] = deque(maxlen=length)
        self.setFixedHeight(14)
        self.setToolTip("Recent colours, newest on the right")

    def push(self, colour: QColor | None):
        self.colours.append(colour)
        self.update()

    def clear(self):
        self.colours.clear()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = QRectF(self.rect())
        clip = QPainterPath()
        clip.addRoundedRect(rect, 4, 4)
        painter.setClipPath(clip)
        painter.fillRect(rect, QColor(theme.SURFACE_RAISED))

        maximum = self.colours.maxlen or 1
        width = rect.width() / maximum
        offset = maximum - len(self.colours)
        for index, colour in enumerate(self.colours):
            if colour is not None:
                painter.fillRect(
                    QRectF((offset + index) * width, 0, width + 0.5, rect.height()),
                    colour,
                )


class ColourCard(QGroupBox):
    HISTORY_INTERVAL_S = 0.25

    def __init__(self, settings: QSettings, parent: QWidget | None = None):
        super().__init__("Colour sensor", parent)
        self.settings = settings
        self.white = self._load_white()
        self.latest: tuple[float, float, float, float] | None = None
        self._last_history = 0.0

        outer = QVBoxLayout(self)
        outer.setSpacing(8)

        row = QHBoxLayout()
        row.setSpacing(12)
        self.swatch = _Swatch()
        row.addWidget(self.swatch)

        details = QVBoxLayout()
        details.setSpacing(2)
        self.name_label = QLabel("—")
        self.name_label.setObjectName("sectionTitle")
        details.addWidget(self.name_label)

        self.hex_label = QLabel("")
        self.hex_label.setObjectName("clock")
        self.hex_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        details.addWidget(self.hex_label)

        self.raw_label = QLabel("")
        self.raw_label.setObjectName("metric")
        details.addWidget(self.raw_label)

        self.calibration_label = QLabel("")
        self.calibration_label.setObjectName("hint")
        self.calibration_label.setWordWrap(True)
        details.addWidget(self.calibration_label)
        details.addStretch()
        row.addLayout(details, 1)
        outer.addLayout(row)

        self.history = _HistoryStrip()
        outer.addWidget(self.history)

        buttons = QHBoxLayout()
        self.calibrate_button = QPushButton("Calibrate White")
        self.calibrate_button.setToolTip(
            "Hold the sensor over something white at its normal working "
            "distance, then click. Saved between runs."
        )
        self.clear_button = QPushButton("Clear")
        self.clear_button.setToolTip("Forget the white calibration")
        buttons.addWidget(self.calibrate_button, 1)
        buttons.addWidget(self.clear_button)
        outer.addLayout(buttons)

        self.calibrate_button.clicked.connect(self.calibrate_white)
        self.clear_button.clicked.connect(self.clear_calibration)

        self._show_calibration_state()
        self.refresh({})

    # ---------------- calibration ----------------

    def _load_white(self) -> tuple[float, float, float] | None:
        try:
            value = json.loads(str(self.settings.value(SETTINGS_KEY, "")))
            if isinstance(value, list) and len(value) == 3 and min(value) > 0:
                return tuple(float(v) for v in value)  # type: ignore[return-value]
        except (TypeError, ValueError):
            pass
        return None

    def calibrate_white(self):
        if self.latest is None:
            self.calibration_label.setText("No reading to calibrate from.")
            return
        r, g, b, _ = self.latest
        if min(r, g, b) <= 0:
            self.calibration_label.setText("A channel read zero — too dark to calibrate.")
            return
        self.white = (r, g, b)
        self.settings.setValue(SETTINGS_KEY, json.dumps(list(self.white)))
        self._show_calibration_state()
        self.refresh_from_latest()

    def clear_calibration(self):
        self.white = None
        self.settings.remove(SETTINGS_KEY)
        self._show_calibration_state()
        self.refresh_from_latest()

    def _show_calibration_state(self):
        if self.white is None:
            self.calibration_label.setText("Uncalibrated — showing hue only")
            self.clear_button.setEnabled(False)
        else:
            r, g, b = (int(v) for v in self.white)
            self.calibration_label.setText(f"White reference R {r} · G {g} · B {b}")
            self.clear_button.setEnabled(True)

    # ---------------- display ----------------

    def set_device_name(self, name: str):
        self.setTitle(name)

    def refresh(self, telemetry: dict | None):
        """Update from the latest telemetry; None means the link is stale."""
        self.latest = _channels(telemetry) if telemetry is not None else None
        self.refresh_from_latest()

    def refresh_from_latest(self):
        if self.latest is None:
            self.swatch.set_colour(None)
            self.name_label.setText("No reading")
            self.hex_label.setText("")
            self.raw_label.setText("")
            self.calibrate_button.setEnabled(False)
            self._push_history(None)
            return

        r, g, b, c = self.latest
        linear = linear_channels(r, g, b, self.white)
        colour = QColor(*to_display_rgb(linear))
        self.swatch.set_colour(colour)
        self.name_label.setText(colour_name(linear, self.white is not None))
        self.hex_label.setText(colour.name().upper())
        self.raw_label.setText(f"R {r:.0f}   G {g:.0f}   B {b:.0f}   C {c:.0f}")
        self.calibrate_button.setEnabled(True)
        self._push_history(colour)

    def _push_history(self, colour: QColor | None):
        now = time.monotonic()
        if now - self._last_history >= self.HISTORY_INTERVAL_S:
            self._last_history = now
            self.history.push(colour)
