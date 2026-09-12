"""Temporal stabilisation of frame-wise gesture predictions.

A per-frame argmax is too jittery to drive a manipulator: a single misclassified
frame would command a motion. The smoother turns the prediction stream into a
stream of *state changes*, which is what the control layer acts on.
"""

from dataclasses import dataclass

import numpy as np

from ..config.gestures import NUM_CLASSES, Gesture


@dataclass(frozen=True)
class GestureTransition:
    """A change in the smoothed gesture state.

    Attributes
    ----------
    previous:
        Smoothed gesture before this frame.
    current:
        Smoothed gesture after this frame.
    """

    previous: Gesture
    current: Gesture

    @property
    def changed(self) -> bool:
        """Whether the smoothed state actually moved."""
        return self.previous != self.current


class MajorityVoteSmoother:
    """Majority vote over a sliding window of frame-wise predictions.

    Parameters
    ----------
    window_size:
        Frames retained in the vote. Larger values reject more noise at the
        cost of latency before a gesture takes effect.
    initial:
        Gesture the buffer is primed with, so no motion is commanded before
        enough frames have been observed.

    Examples
    --------
    >>> smoother = MajorityVoteSmoother(window_size=3, initial=Gesture.UNKNOWN)
    >>> smoother.update(Gesture.MOVE_LEFT).changed
    False
    >>> smoother.update(Gesture.MOVE_LEFT).current is Gesture.MOVE_LEFT
    True
    """

    def __init__(
        self,
        window_size: int = 10,
        initial: Gesture = Gesture.UNKNOWN,
    ) -> None:
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        self.window_size = window_size
        self._initial = initial
        self.reset()

    def reset(self) -> None:
        """Prime the vote buffer and state with the initial gesture."""
        self._votes = np.full(self.window_size, int(self._initial), dtype=np.int8)
        self._state = self._initial

    @property
    def state(self) -> Gesture:
        """Current smoothed gesture."""
        return self._state

    def update(self, prediction: int) -> GestureTransition:
        """Fold one frame-wise prediction into the vote.

        Parameters
        ----------
        prediction:
            Raw class index for the current frame.

        Returns
        -------
        GestureTransition
            The previous and new smoothed gesture. Inspect
            :attr:`GestureTransition.changed` to decide whether to act.
        """
        self._votes = np.roll(self._votes, -1)
        self._votes[-1] = int(prediction)

        winner = Gesture(int(np.argmax(np.bincount(self._votes, minlength=NUM_CLASSES))))
        previous, self._state = self._state, winner
        return GestureTransition(previous=previous, current=winner)
