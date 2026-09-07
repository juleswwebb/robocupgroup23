"""
theme.py

Shared dark theme for the Robot Debug Console and the offline Data
Visualiser: colour palette, Qt stylesheet, pyqtgraph styling, and window
sizing that adapts to the actual screen rather than assuming a big one.
"""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QApplication, QWidget


# =====================================================================
# Palette
# =====================================================================

BACKGROUND = "#14161a"
SURFACE = "#1c1f26"
SURFACE_RAISED = "#242832"
SURFACE_HOVER = "#2c313d"
BORDER = "#2e3440"
BORDER_STRONG = "#3b4252"

TEXT = "#e6e9ef"
TEXT_MUTED = "#98a2b3"
TEXT_FAINT = "#6b7480"

ACCENT = "#3d8bfd"
ACCENT_HOVER = "#5a9dff"
ACCENT_PRESSED = "#2f74dd"

SUCCESS = "#2fbf71"
WARNING = "#f5a524"
DANGER = "#f2544b"
DANGER_HOVER = "#ff6961"
DANGER_PRESSED = "#d43f37"

# Curve colours for plots - picked to stay distinguishable on a dark
# background and to remain separable for the most common colour-vision
# deficiencies (no red/green-only pairings adjacent in the cycle).
PLOT_COLOURS = [
    "#4c9aff",  # blue
    "#f5a524",  # amber
    "#2fbf71",  # green
    "#e879f9",  # magenta
    "#22d3ee",  # cyan
    "#fb7185",  # rose
    "#a78bfa",  # violet
    "#a3e635",  # lime
    "#fbbf24",  # gold
    "#60a5fa",  # light blue
]


def plot_colour(index: int) -> str:
    return PLOT_COLOURS[index % len(PLOT_COLOURS)]


# =====================================================================
# Stylesheet
# =====================================================================

CHEVRON_PATH = (Path(__file__).parent / "assets" / "chevron-down.svg").as_posix()

STYLESHEET = f"""
QWidget {{
    background-color: {BACKGROUND};
    color: {TEXT};
    font-size: 12px;
}}

QMainWindow, QDialog {{
    background-color: {BACKGROUND};
}}

QLabel {{
    background: transparent;
}}

/* ---------------- Group boxes ---------------- */

QGroupBox {{
    background-color: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 8px;
    margin-top: 14px;
    padding: 10px 10px 8px 10px;
    font-weight: 600;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: {TEXT_MUTED};
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 1px;
}}

/* ---------------- Buttons ---------------- */

QPushButton {{
    background-color: {SURFACE_RAISED};
    border: 1px solid {BORDER_STRONG};
    border-radius: 6px;
    padding: 6px 12px;
    color: {TEXT};
    font-weight: 500;
}}

QPushButton:hover {{
    background-color: {SURFACE_HOVER};
    border-color: {ACCENT};
}}

QPushButton:pressed {{
    background-color: {BORDER};
}}

QPushButton:disabled {{
    background-color: {SURFACE};
    color: {TEXT_FAINT};
    border-color: {BORDER};
}}

QPushButton#primary {{
    background-color: {ACCENT};
    border-color: {ACCENT};
    color: #ffffff;
    font-weight: 600;
}}

QPushButton#primary:hover {{
    background-color: {ACCENT_HOVER};
    border-color: {ACCENT_HOVER};
}}

QPushButton#primary:pressed {{
    background-color: {ACCENT_PRESSED};
}}

QPushButton#primary:disabled {{
    background-color: {SURFACE};
    color: {TEXT_FAINT};
    border-color: {BORDER};
}}

QPushButton#danger {{
    background-color: {DANGER};
    border-color: {DANGER};
    color: #ffffff;
    font-weight: 700;
    letter-spacing: 0.5px;
}}

QPushButton#danger:hover {{
    background-color: {DANGER_HOVER};
    border-color: {DANGER_HOVER};
}}

QPushButton#danger:pressed {{
    background-color: {DANGER_PRESSED};
}}

QPushButton#danger:disabled {{
    background-color: {SURFACE};
    color: {TEXT_FAINT};
    border-color: {BORDER};
}}

/* ---------------- Inputs ---------------- */

QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{
    background-color: {SURFACE_RAISED};
    border: 1px solid {BORDER_STRONG};
    border-radius: 6px;
    padding: 5px 8px;
    color: {TEXT};
    selection-background-color: {ACCENT};
    selection-color: #ffffff;
}}

QComboBox:hover, QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover {{
    border-color: {ACCENT};
}}

QComboBox:focus, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {ACCENT};
}}

QComboBox:disabled, QLineEdit:disabled {{
    color: {TEXT_FAINT};
    background-color: {SURFACE};
}}

QComboBox::drop-down {{
    border: none;
    width: 18px;
}}

QComboBox::down-arrow {{
    image: url("{CHEVRON_PATH}");
    width: 10px;
    height: 7px;
    margin-right: 7px;
}}

QComboBox QAbstractItemView {{
    background-color: {SURFACE_RAISED};
    border: 1px solid {BORDER_STRONG};
    border-radius: 6px;
    selection-background-color: {ACCENT};
    selection-color: #ffffff;
    outline: none;
    padding: 4px;
}}

QLineEdit[placeholderText] {{
    color: {TEXT};
}}

QCheckBox {{
    spacing: 7px;
    color: {TEXT};
}}

QCheckBox::indicator {{
    width: 15px;
    height: 15px;
    border: 1px solid {BORDER_STRONG};
    border-radius: 4px;
    background-color: {SURFACE_RAISED};
}}

QCheckBox::indicator:hover {{
    border-color: {ACCENT};
}}

QCheckBox::indicator:checked {{
    background-color: {ACCENT};
    border-color: {ACCENT};
}}

/* ---------------- Tabs ---------------- */

QTabWidget::pane {{
    border: 1px solid {BORDER};
    border-radius: 8px;
    background-color: {SURFACE};
    top: -1px;
}}

QTabBar::tab {{
    background: transparent;
    color: {TEXT_MUTED};
    padding: 8px 18px;
    margin-right: 2px;
    border: 1px solid transparent;
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
    font-weight: 500;
}}

QTabBar::tab:hover {{
    color: {TEXT};
    background: {SURFACE};
}}

QTabBar::tab:selected {{
    background: {SURFACE};
    color: {TEXT};
    border-color: {BORDER};
    border-bottom-color: {SURFACE};
    font-weight: 600;
}}

/* ---------------- Tables ---------------- */

QTableWidget {{
    background-color: {SURFACE};
    alternate-background-color: {SURFACE_RAISED};
    gridline-color: {BORDER};
    border: 1px solid {BORDER};
    border-radius: 8px;
    selection-background-color: {ACCENT};
    selection-color: #ffffff;
    outline: none;
}}

QTableWidget::item {{
    padding: 5px 8px;
    border: none;
}}

QHeaderView::section {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_MUTED};
    padding: 7px 8px;
    border: none;
    border-right: 1px solid {BORDER};
    border-bottom: 1px solid {BORDER};
    font-weight: 600;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}}

QHeaderView::section:last {{
    border-right: none;
}}

QTableCornerButton::section {{
    background-color: {SURFACE_RAISED};
    border: none;
}}

/* ---------------- Lists ---------------- */

QListWidget {{
    background-color: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 4px;
    outline: none;
}}

QListWidget::item {{
    padding: 5px 8px;
    border-radius: 5px;
}}

QListWidget::item:hover {{
    background-color: {SURFACE_RAISED};
}}

QListWidget::item:selected {{
    background-color: {ACCENT};
    color: #ffffff;
}}

/* ---------------- Text areas ---------------- */

QTextEdit {{
    background-color: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 8px;
    color: {TEXT};
    selection-background-color: {ACCENT};
    selection-color: #ffffff;
}}

/* ---------------- Scrollbars ---------------- */

QScrollBar:vertical {{
    background: transparent;
    width: 11px;
    margin: 0;
}}

QScrollBar::handle:vertical {{
    background: {BORDER_STRONG};
    border-radius: 5px;
    min-height: 28px;
}}

QScrollBar::handle:vertical:hover {{
    background: {TEXT_FAINT};
}}

QScrollBar:horizontal {{
    background: transparent;
    height: 11px;
    margin: 0;
}}

QScrollBar::handle:horizontal {{
    background: {BORDER_STRONG};
    border-radius: 5px;
    min-width: 28px;
}}

QScrollBar::handle:horizontal:hover {{
    background: {TEXT_FAINT};
}}

QScrollBar::add-line, QScrollBar::sub-line {{
    height: 0;
    width: 0;
}}

QScrollBar::add-page, QScrollBar::sub-page {{
    background: none;
}}

QScrollArea {{
    border: none;
    background-color: transparent;
}}

/* ---------------- Splitter ---------------- */

QSplitter::handle {{
    background-color: {BORDER};
}}

QSplitter::handle:horizontal {{
    width: 2px;
}}

QSplitter::handle:vertical {{
    height: 2px;
}}

QSplitter::handle:hover {{
    background-color: {ACCENT};
}}

/* ---------------- Status bar ---------------- */

QStatusBar {{
    background-color: {SURFACE};
    color: {TEXT_MUTED};
    border-top: 1px solid {BORDER};
}}

QStatusBar::item {{
    border: none;
}}

/* ---------------- Status pills ---------------- */

QLabel#statusPill {{
    background-color: {SURFACE_RAISED};
    border: 1px solid {BORDER_STRONG};
    border-radius: 11px;
    padding: 4px 12px;
    color: {TEXT_MUTED};
    font-weight: 600;
    font-size: 11px;
}}

QLabel#statusPill[state="ok"] {{
    background-color: rgba(47, 191, 113, 0.15);
    border-color: {SUCCESS};
    color: {SUCCESS};
}}

QLabel#statusPill[state="bad"] {{
    background-color: rgba(242, 84, 75, 0.15);
    border-color: {DANGER};
    color: {DANGER};
}}

QLabel#statusPill[state="busy"] {{
    background-color: rgba(245, 165, 36, 0.15);
    border-color: {WARNING};
    color: {WARNING};
}}

/* ---------------- Misc labels ---------------- */

QLabel#sectionTitle {{
    font-size: 15px;
    font-weight: 700;
    color: {TEXT};
    padding-bottom: 2px;
}}

QLabel#hint {{
    color: {TEXT_MUTED};
    font-size: 11px;
}}

QLabel#metric {{
    color: {TEXT_MUTED};
    font-size: 11px;
}}

QLabel#clock {{
    color: {TEXT};
    font-family: "Menlo", "Consolas", monospace;
    font-size: 13px;
    font-weight: 600;
}}

QLabel#fieldLabel {{
    color: {TEXT_MUTED};
    font-size: 11px;
    font-weight: 600;
}}
"""


# =====================================================================
# Helpers
# =====================================================================

def apply(app: QApplication):
    """Apply the dark theme to the whole application."""
    app.setStyle("Fusion")
    app.setStyleSheet(STYLESHEET)


def monospace_font(size: int = 12) -> QFont:
    font = QFont("Menlo")
    font.setStyleHint(QFont.StyleHint.Monospace)
    font.setPointSize(size)
    return font


def set_pill_state(label, state: str):
    """
    Set a status pill's colour: "ok", "bad", "busy", or "" for neutral.

    Qt only restyles on a property change if we re-polish the widget, so
    do that here rather than at every call site.
    """
    label.setProperty("state", state)
    label.style().unpolish(label)
    label.style().polish(label)


def fit_to_screen(
    window: QWidget,
    preferred_width: int = 1400,
    preferred_height: int = 880,
    margin: float = 0.94,
):
    """
    Size a window to fit the screen it's on, then centre it.

    The previous hard-coded 1500x900 was larger than a 1440x900 laptop
    display once the menu bar and dock were taken off, so the window ran
    off-screen. Clamp to what's actually available instead.
    """
    screen = window.screen() or QApplication.primaryScreen()

    if screen is None:
        window.resize(preferred_width, preferred_height)
        return

    available = screen.availableGeometry()

    width = min(preferred_width, int(available.width() * margin))
    height = min(preferred_height, int(available.height() * margin))

    window.resize(width, height)
    window.move(
        available.x() + (available.width() - width) // 2,
        available.y() + (available.height() - height) // 2,
    )


def style_plot(plot_widget):
    """Apply the dark theme to a pyqtgraph PlotWidget."""
    plot_widget.setBackground(SURFACE)

    plot_item = plot_widget.getPlotItem()
    plot_item.showGrid(x=True, y=True, alpha=0.15)

    for axis_name in ("left", "bottom"):
        axis = plot_item.getAxis(axis_name)
        axis.setPen(BORDER_STRONG)
        axis.setTextPen(TEXT_MUTED)
