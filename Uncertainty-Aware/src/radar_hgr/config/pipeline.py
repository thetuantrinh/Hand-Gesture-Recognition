"""Feature-extraction geometry linking the DSP output to the classifier input.

These constants were previously inlined as literals in the GUI control loop.
They are grouped here because they must agree with the shape the trained
network was fitted on: changing one without retraining invalidates the model.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PipelineConfig:
    """Micro-Doppler windowing applied between the 2D FFT and the classifier.

    Attributes
    ----------
    range_bin:
        Range bin the hand occupies at the nominal standoff distance. A single
        bin is extracted from the range-Doppler map per frame.
    doppler_bin_start, doppler_bin_count:
        Half-open Doppler slice retained around zero velocity.
    history_frames:
        Depth of the rolling micro-Doppler buffer that backs the live plot.
    inference_frames:
        Trailing frames of that buffer fed to the network, i.e. the temporal
        receptive field the model was trained with.
    """

    range_bin: int = 33
    doppler_bin_start: int = 44
    doppler_bin_count: int = 40
    history_frames: int = 40
    inference_frames: int = 20

    @property
    def doppler_slice(self) -> slice:
        """Doppler bins retained from each range-Doppler map."""
        return slice(self.doppler_bin_start, self.doppler_bin_start + self.doppler_bin_count)

    def input_shape(self, num_rx_antennas: int, iq_channels: int) -> tuple:
        """Batch-shaped classifier input ``(1, time, doppler, rx * iq)``."""
        return (1, self.inference_frames, self.doppler_bin_count, num_rx_antennas * iq_channels)
