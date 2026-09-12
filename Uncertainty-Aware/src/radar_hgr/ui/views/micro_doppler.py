"""Live micro-Doppler spectrogram rendering."""

from typing import Any

import numpy as np

from .colormaps import micro_doppler_colormap

#: Display window in dB. The floor sits just above the noise level of the
#: bench setup, so idle frames render as background rather than as speckle.
DEFAULT_LEVELS: tuple[float, float] = (65.0, 80.0)

#: Guards the logarithm against an exactly-zero magnitude.
LOG_FLOOR = 1e-9


def log_magnitude(spectrum: np.ndarray) -> np.ndarray:
    """Magnitude of a complex spectrum in decibels."""
    return 20.0 * np.log10(np.abs(spectrum) + LOG_FLOOR)


class MicroDopplerView:
    """Draws the rolling micro-Doppler buffer into a ``pyqtgraph.ImageView``.

    The colour map is installed once at construction rather than on every
    frame, which the previous implementation did.

    Parameters
    ----------
    image_view:
        The ``pyqtgraph.ImageView`` to draw into.
    levels:
        Display intensity window ``(min_dB, max_dB)``.
    """

    def __init__(self, image_view: Any, levels: tuple[float, float] = DEFAULT_LEVELS) -> None:
        self.image_view = image_view
        self.levels = levels
        self.image_view.setColorMap(micro_doppler_colormap())

    def update(self, history: np.ndarray) -> None:
        """Render the current micro-Doppler history.

        Parameters
        ----------
        history:
            Rolling buffer shaped ``[time, rx, iq, doppler]``. The first
            antenna's real and imaginary channels are recombined and displayed.
        """
        complex_signal = history[:, 0, 0, :] + 1j * history[:, 0, 1, :]
        self.image_view.setImage(log_magnitude(complex_signal), levels=list(self.levels))
