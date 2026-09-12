"""Clutter rejection filters."""

import numpy as np


def remove_static_clutter(range_profile: np.ndarray, chirp_axis: int = 1) -> np.ndarray:
    """Suppress stationary reflectors — the Moving Target Indication (MTI) filter.

    Subtracts the per-antenna mean across chirps. Stationary reflectors (walls,
    the bench, the robot's own base) contribute an identical return to every
    chirp of a frame and are cancelled, leaving only the moving hand.

    Parameters
    ----------
    range_profile:
        Range FFT output with a chirp axis, typically ``[rx, chirp, sample]``.
    chirp_axis:
        Axis along which chirps are stacked.

    Returns
    -------
    np.ndarray
        Clutter-suppressed profile with the input's shape and dtype.
    """
    static_component = range_profile.mean(axis=chirp_axis, keepdims=True)
    return range_profile - static_component
