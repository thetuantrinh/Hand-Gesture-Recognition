"""Reassembly of the DCA1000EVM packet stream into whole radar frames.

The capture card streams fixed-size UDP payloads that are not aligned to frame
boundaries, so samples must be buffered until a full frame is available. This
concern is kept separate from both the socket layer and the FFT chain so it can
be exercised without hardware.
"""


import numpy as np


class FrameAssembler:
    """Buffers interleaved int16 ADC samples and emits complete frames.

    Parameters
    ----------
    samples_per_frame:
        Scalar samples in one frame, from
        :attr:`~radar_hgr.config.radar.RadarConfig.samples_per_frame`.

    Examples
    --------
    >>> assembler = FrameAssembler(samples_per_frame=4)
    >>> [frame.tolist() for frame in assembler.push(np.arange(5, dtype=np.int16))]
    [[0, 1, 2, 3]]
    >>> assembler.pending
    1
    """

    def __init__(self, samples_per_frame: int) -> None:
        if samples_per_frame <= 0:
            raise ValueError("samples_per_frame must be positive")
        self.samples_per_frame = samples_per_frame
        self._buffer = np.empty(0, dtype=np.int16)

    @property
    def pending(self) -> int:
        """Samples buffered so far that do not yet form a complete frame."""
        return int(self._buffer.size)

    def push(self, samples: np.ndarray) -> list[np.ndarray]:
        """Append a packet payload and return every frame it completes.

        A single packet may complete no frames, or more than one if the
        consumer has fallen behind, so the result is always a list.

        The list is built eagerly rather than produced lazily: buffering has to
        happen when the packet arrives, not when the caller gets round to
        iterating, or a caller that discarded the result would silently drop
        samples.

        Parameters
        ----------
        samples:
            1-D array of int16 ADC samples from one UDP payload.

        Returns
        -------
        list of np.ndarray
            Complete frames, each of length ``samples_per_frame``.
        """
        if samples is None or len(samples) == 0:
            return []
        self._buffer = np.concatenate((self._buffer, samples))

        frames = []
        while self._buffer.size >= self.samples_per_frame:
            frames.append(self._buffer[: self.samples_per_frame])
            self._buffer = self._buffer[self.samples_per_frame :]
        return frames

    def reset(self) -> None:
        """Discard buffered samples, e.g. after restarting the capture card."""
        self._buffer = np.empty(0, dtype=np.int16)
