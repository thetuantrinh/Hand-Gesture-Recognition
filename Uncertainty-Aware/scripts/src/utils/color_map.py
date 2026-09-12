"""Color map utilities for radar Doppler-range heatmap visualization."""

from typing import List, Tuple
import numpy as np


def get_radar_colormap() -> np.ndarray:
    """Return an RGB color gradient for micro-Doppler heatmap display.

    Returns
    -------
    np.ndarray
        Array of RGB uint8 tuples (Blue -> Green -> Yellow).
    """
    return np.array(
        [
            (0, 0, 255),    # Low energy (Blue)
            (0, 255, 0),    # Mid energy (Green)
            (255, 255, 0),  # Peak energy (Yellow)
        ],
        dtype=np.ubyte,
    )


# Backwards compatibility alias
cmap = get_radar_colormap
