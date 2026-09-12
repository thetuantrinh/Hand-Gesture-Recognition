"""Micro-Doppler heatmap plotting wrapper for PyQtGraph."""

from typing import Any, List
import numpy as np

try:
    import pyqtgraph as pg
except ImportError:
    pg = None

from .color_map import get_radar_colormap


class MicroDopplerPlotter:
    """Renders real-time micro-Doppler radar energy heatmaps in PyQtGraph.

    Parameters
    ----------
    graph_widget : Any
        PyQtGraph ImageView / PlotWidget instance.
    window_data : np.ndarray
        Accumulated Doppler-range feature tensor of shape [time_steps, Nr, IQ, bins].
    """

    def __init__(self, graph_widget: Any, window_data: np.ndarray) -> None:
        self.graph = graph_widget
        self.window_data = window_data
        if pg is not None:
            color_lut = pg.ColorMap(pos=np.linspace(0.0, 1.0, 3), color=get_radar_colormap())
            self.graph.setColorMap(color_lut)

    def show_micro_doppler(self, levels: List[float] = [65.0, 80.0]) -> None:
        """Render micro-Doppler spectrum in log magnitude (dB).

        Parameters
        ----------
        levels : List[float], optional
            Display intensity range [min_dB, max_dB], by default [65.0, 80.0].
        """
        complex_signal = self.window_data[:, 0, 0, :] + 1j * self.window_data[:, 0, 1, :]
        log_magnitude = 20.0 * np.log10(np.abs(complex_signal) + 1e-9)
        self.graph.setImage(log_magnitude, levels=levels)

    # Legacy alias
    show_micro_Dopler = show_micro_doppler


# Backwards compatibility alias
PLOT = MicroDopplerPlotter