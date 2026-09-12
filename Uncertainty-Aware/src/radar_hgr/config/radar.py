"""FMCW radar front-end and ADC sampling geometry."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RadarConfig:
    """AWR1243BOOST + DCA1000EVM acquisition geometry.

    Attributes
    ----------
    num_chirps:
        Chirps per frame (slow-time / Doppler dimension).
    num_adc_samples:
        ADC samples per chirp (fast-time / range dimension).
    num_rx_antennas:
        Physical receive antennas.
    num_tx_antennas:
        Physical transmit antennas.
    iq_channels:
        Interleaved components per sample: in-phase (I) and quadrature (Q).
    adc_bits:
        ADC resolution, in bits.
    """

    num_chirps: int = 128
    num_adc_samples: int = 64
    num_rx_antennas: int = 4
    num_tx_antennas: int = 3
    iq_channels: int = 2
    adc_bits: int = 16

    @property
    def num_virtual_antennas(self) -> int:
        """Virtual array size realised by TDM-MIMO multiplexing."""
        return self.num_tx_antennas * self.num_rx_antennas

    @property
    def samples_per_channel(self) -> int:
        """Scalar samples per interleaved RX/IQ channel in one frame."""
        return self.num_chirps * self.num_adc_samples

    @property
    def samples_per_frame(self) -> int:
        """Total scalar int16 samples in one complete frame."""
        return self.samples_per_channel * self.num_rx_antennas * self.iq_channels
