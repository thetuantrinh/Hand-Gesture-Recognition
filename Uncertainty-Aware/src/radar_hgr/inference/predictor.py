"""Gesture classification with temporal stabilisation."""

import logging
from pathlib import Path
from typing import Optional

import numpy as np

from ..config.gestures import Gesture
from ..config.paths import resolve_model_path
from .keras_model import Classifier, load_classifier
from .smoothing import GestureTransition, MajorityVoteSmoother

logger = logging.getLogger(__name__)

#: Frames of history the majority vote considers.
DEFAULT_VOTE_WINDOW = 10


class GesturePredictor:
    """Classifies micro-Doppler windows and reports smoothed state changes.

    Parameters
    ----------
    model_path:
        Checkpoint to load. When ``None``, :func:`resolve_model_path` picks one.
    vote_window:
        Frames considered by the majority vote.
    classifier:
        Pre-built classifier, which bypasses checkpoint loading entirely. Used
        by the tests to drive the predictor without TensorFlow.

    Notes
    -----
    :attr:`available` reports whether a model was actually loaded. When it is
    ``False``, :meth:`predict` is a no-op that holds the current state, so the
    caller needs no special case.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        vote_window: int = DEFAULT_VOTE_WINDOW,
        classifier: Optional[Classifier] = None,
    ) -> None:
        if classifier is not None:
            self.model_path: Optional[Path] = None
            self._classifier: Optional[Classifier] = classifier
        else:
            self.model_path = resolve_model_path(model_path)
            self._classifier = load_classifier(self.model_path)

        self._smoother = MajorityVoteSmoother(window_size=vote_window, initial=Gesture.UNKNOWN)

    @property
    def available(self) -> bool:
        """Whether a usable classifier is loaded."""
        return self._classifier is not None

    @property
    def model_name(self) -> str:
        """Checkpoint file name for GUI display."""
        return self.model_path.name if self.model_path else "None"

    @property
    def state(self) -> Gesture:
        """Current smoothed gesture."""
        return self._smoother.state

    def reset(self) -> None:
        """Clear the temporal vote, e.g. after restarting acquisition."""
        self._smoother.reset()

    def predict(self, features: np.ndarray) -> Optional[GestureTransition]:
        """Classify one feature window and fold it into the vote.

        Parameters
        ----------
        features:
            Batched window shaped ``(1, time, doppler, rx * iq)``, as produced
            by :class:`radar_hgr.app.gesture_pipeline.MicroDopplerPipeline`.

        Returns
        -------
        GestureTransition or None
            The smoothed transition, or ``None`` when no model is loaded.
        """
        if self._classifier is None:
            return None
        return self._smoother.update(self._classifier.predict_class(features))
