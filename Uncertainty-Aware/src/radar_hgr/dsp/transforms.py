"""Fourier transform stages of the range-Doppler chain."""

import numpy as np
from numpy.fft import fft, fftshift

from .windows import doppler_window, range_window

#: Added to the spectrum so downstream log-magnitude conversion never sees zero.
SPECTRUM_FLOOR: float = 1e-9


def range_fft(adc_frame: np.ndarray, apply_window: bool = True) -> np.ndarray:
    """Fast-time FFT resolving target range.

    Parameters
    ----------
    adc_frame:
        Complex IQ frame shaped ``[rx, chirp, sample]``.
    apply_window:
        Apply the Blackman range taper before transforming.

    Returns
    -------
    np.ndarray
        Range spectrum, same shape as the input.
    """
    if apply_window:
        adc_frame = adc_frame * range_window(adc_frame.shape[-1])
    return fft(adc_frame, axis=-1)


def doppler_fft(range_profile: np.ndarray, apply_window: bool = True) -> np.ndarray:
    """Slow-time FFT resolving radial velocity.

    The result is *not* frequency-shifted; see
    :func:`center_zero_velocity` for the centring step.

    Parameters
    ----------
    range_profile:
        Clutter-suppressed range spectrum shaped ``[rx, sample, chirp]``.
    apply_window:
        Apply the Hanning Doppler taper before transforming.

    Returns
    -------
    np.ndarray
        Doppler spectrum, same shape as the input.
    """
    if apply_window:
        range_profile = range_profile * doppler_window(range_profile.shape[-1])
    return fft(range_profile, axis=-1)


def center_zero_velocity(spectrum: np.ndarray) -> np.ndarray:
    """Roll the Doppler spectrum so zero velocity sits at the centre bin.

    .. warning::

       This deliberately calls :func:`numpy.fft.fftshift` over **every** axis,
       not just the Doppler axis, which additionally rolls the antenna and
       range axes by half their length. That is not what a textbook
       range-Doppler chain does, but the shipped checkpoints in ``models/``
       were trained on features produced this way, so the behaviour is retained
       for compatibility. Restricting the shift to the Doppler axis changes the
       tensor the network sees and requires retraining.
    """
    return fftshift(spectrum)


def split_iq_channels(spectrum: np.ndarray) -> np.ndarray:
    """Interleave real and imaginary parts into an explicit channel axis.

    The complex-valued network consumes real tensors, so each antenna's
    spectrum is expanded into an adjacent (real, imaginary) pair.

    Parameters
    ----------
    spectrum:
        Complex array shaped ``[rx, ...]``.

    Returns
    -------
    np.ndarray
        ``float32`` array shaped ``[rx, 2, ...]``.
    """
    return np.stack((spectrum.real, spectrum.imag), axis=1).astype(np.float32)
