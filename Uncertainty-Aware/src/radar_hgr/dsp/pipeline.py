"""The range-Doppler processing chain, from raw ADC frame to network features."""

from typing import Optional

import numpy as np

from ..config.radar import RadarConfig
from .filters import remove_static_clutter
from .transforms import (
    SPECTRUM_FLOOR,
    center_zero_velocity,
    doppler_fft,
    range_fft,
    split_iq_channels,
)


class RangeDopplerProcessor:
    """Converts one raw ADC frame into a real-valued range-Doppler tensor.

    The chain is: de-interleave IQ, range FFT, MTI clutter rejection, Doppler
    FFT, zero-velocity centring, and finally a split of the complex spectrum
    into adjacent real/imaginary channels for the complex-valued network.

    Parameters
    ----------
    config:
        Acquisition geometry. Defaults to :class:`RadarConfig`.
    """

    def __init__(self, config: Optional[RadarConfig] = None) -> None:
        self.config = config or RadarConfig()

    @property
    def samples_per_frame(self) -> int:
        """Scalar samples the processor expects in one frame."""
        return self.config.samples_per_frame

    def deinterleave(self, frame: np.ndarray) -> np.ndarray:
        """Reassemble an interleaved int16 frame into complex IQ per antenna.

        The capture card emits the four receive antennas' in-phase samples
        followed by their quadrature samples, column-major within a frame.

        Parameters
        ----------
        frame:
            Flat int16 frame of length :attr:`samples_per_frame`.

        Returns
        -------
        np.ndarray
            Complex array shaped ``[rx, chirp, sample]``.
        """
        cfg = self.config
        num_lanes = cfg.num_rx_antennas * cfg.iq_channels
        lanes = np.reshape(frame, (num_lanes, cfg.samples_per_channel), order="F")

        in_phase = lanes[: cfg.num_rx_antennas, :]
        quadrature = lanes[cfg.num_rx_antennas :, :]
        complex_lanes = in_phase + 1j * quadrature

        return complex_lanes.reshape(cfg.num_rx_antennas, cfg.num_chirps, cfg.num_adc_samples)

    def range_doppler_map(self, adc_frame: np.ndarray) -> np.ndarray:
        """Complex range-Doppler map for a de-interleaved frame.

        Parameters
        ----------
        adc_frame:
            Complex array shaped ``[rx, chirp, sample]``.

        Returns
        -------
        np.ndarray
            Complex array shaped ``[rx, sample, chirp]``.
        """
        range_profile = range_fft(adc_frame)
        moving_only = remove_static_clutter(range_profile, chirp_axis=1)

        # Transpose to [rx, sample, chirp] so the Doppler FFT runs on the last axis.
        doppler_spectrum = doppler_fft(moving_only.transpose(0, 2, 1))
        return center_zero_velocity(doppler_spectrum) + SPECTRUM_FLOOR

    def process(self, frame: np.ndarray) -> np.ndarray:
        """Run the full chain on one raw frame.

        Parameters
        ----------
        frame:
            Flat int16 frame of length :attr:`samples_per_frame`.

        Returns
        -------
        np.ndarray
            ``float32`` tensor shaped ``[rx, 2, sample, chirp]``, where axis 1
            holds the real and imaginary components.
        """
        return split_iq_channels(self.range_doppler_map(self.deinterleave(frame)))
