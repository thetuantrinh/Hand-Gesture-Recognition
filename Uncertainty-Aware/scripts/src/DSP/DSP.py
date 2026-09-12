"""Radar Digital Signal Processing (DSP) Module.

Provides Range FFT, Doppler FFT, Moving Target Indication (MTI) clutter removal,
and IQ complex sample reconstruction for FMCW radar systems.
"""

from typing import Optional, Union
import numpy as np
from numpy.fft import fft, fftshift


class RadarDSP:
    """Digital Signal Processor for FMCW Radar data frames.

    Performs fast-time (range) and slow-time (Doppler) discrete Fourier transforms,
    windowing, and static background clutter rejection.

    Parameters
    ----------
    Nc : int
        Number of chirps per frame (slow-time dimension).
    Ns : int
        Number of ADC samples per chirp (fast-time dimension).
    """

    def __init__(self, Nc: int = 128, Ns: int = 64) -> None:
        self.adc_bit: int = 16
        self.Ns: int = Ns
        self.Nc: int = Nc
        self.Nt: int = 3  # Number of transmit antennas
        self.Nr: int = 4  # Number of receive antennas
        self.virtual_antennas: int = self.Nt * self.Nr
        self.num_angle_bins: int = Nc

        # Pre-computed windowing functions
        self.win_range: np.ndarray = np.blackman(self.Ns).astype(np.float32)
        self.win_doppler: np.ndarray = np.hanning(self.Nc).astype(np.float32)

    def fft_1d(self, adc_data: np.ndarray) -> np.ndarray:
        """Perform 1D Range FFT across fast-time ADC samples.

        Parameters
        ----------
        adc_data : np.ndarray
            Input complex ADC matrix of shape [Nr, Nc, Ns].

        Returns
        -------
        np.ndarray
            1D Range FFT spectrum of shape [Nr, Nc, Ns].
        """
        windowed = np.multiply(adc_data, self.win_range)
        return fft(windowed, axis=2)

    # Legacy alias
    fft_1D = fft_1d

    def mti_filter(self, range_data: np.ndarray) -> np.ndarray:
        """Moving Target Indication (MTI) filter for static clutter removal.

        Subtracts the average across all chirps for each antenna,
        removing stationary reflections (walls, desks) and isolating moving targets.

        Parameters
        ----------
        range_data : np.ndarray
            Range FFT output of shape [Nr, Nc, Ns].

        Returns
        -------
        np.ndarray
            Clutter-removed range profile with identical shape.
        """
        avg_chirp = (1.0 / self.Nc) * np.sum(range_data, axis=1, keepdims=True)
        return range_data - avg_chirp

    # Legacy alias
    MTI = mti_filter

    def fft_2d(
        self,
        adc_data: np.ndarray,
        apply_range_window: bool = True,
        apply_doppler_window: bool = True,
    ) -> np.ndarray:
        """Compute 2D Range-Doppler FFT matrix from raw frame.

        Parameters
        ----------
        adc_data : np.ndarray
            Complex ADC matrix of shape [Nr, Nc, Ns].
        apply_range_window : bool, optional
            Whether to apply Blackman window on fast-time samples, by default True.
        apply_doppler_window : bool, optional
            Whether to apply Hanning window on slow-time chirps, by default True.

        Returns
        -------
        np.ndarray
            Range-Doppler matrix formatted as [Nr, 2 (Real/Imag), Ns, Nc].
        """
        # Fast-time (Range) processing
        if apply_range_window:
            windowed_1d = np.multiply(adc_data, self.win_range)
            out_fft1d = fft(windowed_1d, axis=2)
        else:
            out_fft1d = fft(adc_data, axis=2)

        # Static clutter removal
        filtered_1d = self.mti_filter(out_fft1d)

        # Slow-time (Doppler) processing
        transposed = filtered_1d.transpose(0, 2, 1)  # [Nr, Ns, Nc]
        if apply_doppler_window:
            windowed_2d = np.multiply(transposed, self.win_doppler)
            out_fft2d = fft(windowed_2d, axis=2)
        else:
            out_fft2d = fft(filtered_1d, axis=1).transpose(0, 2, 1)

        fft_shifted = fftshift(out_fft2d) + 1e-9

        # Separate real and imaginary components for CNN consumption
        return np.array(
            [
                [fft_shifted[0].real, fft_shifted[0].imag],
                [fft_shifted[1].real, fft_shifted[1].imag],
                [fft_shifted[2].real, fft_shifted[2].imag],
                [fft_shifted[3].real, fft_shifted[3].imag],
            ],
            dtype=np.float32,
        )

    # Legacy alias
    fft_2D = fft_2d

    def pre_processing(
        self,
        adc_data: np.ndarray,
        fft: bool = False,
    ) -> np.ndarray:
        """Reshape interleaved raw ADC stream into multi-channel complex format.

        Parameters
        ----------
        adc_data : np.ndarray
            Raw ADC linear sample buffer.
        fft : bool, optional
            If True, immediately computes 2D Range-Doppler FFT, by default False.

        Returns
        -------
        np.ndarray
            Complex matrix [Nr, Nc, Ns] or 2D FFT tensor [Nr, 2, Ns, Nc].
        """
        reshaped = np.reshape(adc_data, (8, self.Nc * self.Ns), order="F")
        complex_iq = reshaped[:4, :] + 1j * reshaped[4:, :]

        formatted = np.array(
            [
                np.reshape(complex_iq[0], (self.Nc, self.Ns)),
                np.reshape(complex_iq[1], (self.Nc, self.Ns)),
                np.reshape(complex_iq[2], (self.Nc, self.Ns)),
                np.reshape(complex_iq[3], (self.Nc, self.Ns)),
            ]
        )

        if fft:
            return self.fft_2d(formatted)

        return formatted


# Backwards compatibility alias
_FFT_ = RadarDSP