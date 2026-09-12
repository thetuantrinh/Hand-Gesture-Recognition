"""Colour maps for radar intensity displays."""

import numpy as np

#: Blue (low) through green to yellow (peak) — the return-energy ramp used for
#: micro-Doppler spectrograms throughout the paper's figures.
MICRO_DOPPLER_STOPS = np.array(
    [
        (0, 0, 255),
        (0, 255, 0),
        (255, 255, 0),
    ],
    dtype=np.ubyte,
)


def micro_doppler_colormap():
    """Build a ``pyqtgraph.ColorMap`` over :data:`MICRO_DOPPLER_STOPS`."""
    import pyqtgraph as pg

    positions = np.linspace(0.0, 1.0, len(MICRO_DOPPLER_STOPS))
    return pg.ColorMap(pos=positions, color=MICRO_DOPPLER_STOPS)
