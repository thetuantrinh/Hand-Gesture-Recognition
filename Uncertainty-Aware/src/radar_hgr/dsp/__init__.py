"""Digital signal processing for FMCW radar.

The chain is exposed as small composable stages (:mod:`windows`,
:mod:`filters`, :mod:`transforms`), assembled by
:class:`~radar_hgr.dsp.pipeline.RangeDopplerProcessor`, and fed by
:class:`~radar_hgr.dsp.framing.FrameAssembler`.
"""

from .filters import remove_static_clutter
from .framing import FrameAssembler
from .pipeline import RangeDopplerProcessor
from .transforms import center_zero_velocity, doppler_fft, range_fft, split_iq_channels
from .windows import doppler_window, range_window

__all__ = [
    "FrameAssembler",
    "RangeDopplerProcessor",
    "center_zero_velocity",
    "doppler_fft",
    "doppler_window",
    "range_fft",
    "range_window",
    "remove_static_clutter",
    "split_iq_channels",
]
