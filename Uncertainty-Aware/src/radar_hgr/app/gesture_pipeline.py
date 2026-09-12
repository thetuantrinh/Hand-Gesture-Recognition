"""Assembly of range-Doppler frames into classifier input.

This is the stage between the DSP chain and the network: it maintains the
rolling micro-Doppler buffer that backs both the live spectrogram and the
inference window. It holds no Qt state, so the framing and windowing can be
tested without a GUI or a radar.
"""

from typing import Optional

import numpy as np

from ..config.pipeline import PipelineConfig
from ..config.radar import RadarConfig
from ..dsp.pipeline import RangeDopplerProcessor


class MicroDopplerPipeline:
    """Turns raw radar frames into a rolling micro-Doppler history.

    Each frame is reduced to one range bin's Doppler slice and pushed into a
    fixed-depth buffer; the trailing frames of that buffer form the classifier's
    temporal window.

    Parameters
    ----------
    radar_config:
        Acquisition geometry. Defaults to :class:`RadarConfig`.
    pipeline_config:
        Feature windowing. Defaults to :class:`PipelineConfig`.
    processor:
        Range-Doppler processor. Defaults to one built from ``radar_config``.

    Attributes
    ----------
    history:
        Buffer shaped ``[history_frames, rx, iq, doppler]``. Pre-filled with
        ones so the spectrogram and the classifier both have a full window from
        the first frame onwards.
    """

    def __init__(
        self,
        radar_config: Optional[RadarConfig] = None,
        pipeline_config: Optional[PipelineConfig] = None,
        processor: Optional[RangeDopplerProcessor] = None,
    ) -> None:
        self.radar_config = radar_config or RadarConfig()
        self.config = pipeline_config or PipelineConfig()
        self.processor = processor or RangeDopplerProcessor(self.radar_config)
        self.reset()

    @property
    def history_shape(self) -> tuple:
        """Shape of :attr:`history`."""
        return (
            self.config.history_frames,
            self.radar_config.num_rx_antennas,
            self.radar_config.iq_channels,
            self.config.doppler_bin_count,
        )

    def reset(self) -> None:
        """Clear the rolling history back to its primed state."""
        self.history = np.ones(self.history_shape, dtype=np.float32)

    def push(self, frame: np.ndarray) -> np.ndarray:
        """Process one raw frame and fold it into the history.

        Parameters
        ----------
        frame:
            Flat int16 frame, as emitted by
            :class:`~radar_hgr.dsp.framing.FrameAssembler`.

        Returns
        -------
        np.ndarray
            The updated :attr:`history`.
        """
        range_doppler = self.processor.process(frame)

        # Reduce [rx, iq, sample, chirp] to the hand's range bin and the
        # Doppler band around zero velocity, then prepend a time axis.
        slice_ = range_doppler[:, :, self.config.range_bin, self.config.doppler_slice]
        self.history = np.concatenate(
            (self.history, slice_[np.newaxis, ...]),
            axis=0,
        )[-self.config.history_frames :]

        return self.history

    def inference_window(self) -> np.ndarray:
        """Batched classifier input for the current history.

        Returns
        -------
        np.ndarray
            Tensor shaped ``(1, inference_frames, doppler, rx * iq)``, matching
            :meth:`PipelineConfig.input_shape`.
        """
        recent = self.history[-self.config.inference_frames :]

        # [time, rx, iq, doppler] -> [time, doppler, rx, iq], then flatten the
        # antenna and IQ axes into the single channel axis the network expects.
        channels_last = recent.transpose(0, 3, 1, 2)
        return channels_last.reshape(
            self.config.input_shape(
                self.radar_config.num_rx_antennas,
                self.radar_config.iq_channels,
            )
        )
