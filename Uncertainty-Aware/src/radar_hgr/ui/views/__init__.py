"""Presentation adapters between application state and the layout's widgets.

Each view owns the widgets it writes to and the unit conversions or scaling its
display needs, so the main window holds no formatting logic.
"""

from .colormaps import micro_doppler_colormap
from .log_console import LogConsole
from .micro_doppler import MicroDopplerView, log_magnitude
from .telemetry import TelemetryView

__all__ = [
    "LogConsole",
    "MicroDopplerView",
    "TelemetryView",
    "log_magnitude",
    "micro_doppler_colormap",
]
