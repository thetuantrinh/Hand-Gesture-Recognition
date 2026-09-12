"""Taper functions applied before each FFT stage.

Windows are pure functions of their length and are cached, because the shapes
are fixed for the lifetime of an acquisition and the processor re-applies them
to every frame at the radar frame rate.
"""

from functools import lru_cache

import numpy as np


@lru_cache(maxsize=8)
def range_window(num_samples: int) -> np.ndarray:
    """Blackman taper over fast-time ADC samples.

    Blackman's high sidelobe attenuation suppresses range leakage from the
    strong static reflections that dominate an indoor scene.
    """
    return np.blackman(num_samples).astype(np.float32)


@lru_cache(maxsize=8)
def doppler_window(num_chirps: int) -> np.ndarray:
    """Hanning taper over slow-time chirps.

    Hanning trades sidelobe depth for a narrower main lobe, preserving the
    closely spaced Doppler components that distinguish one gesture's
    micro-motion from another's.
    """
    return np.hanning(num_chirps).astype(np.float32)
