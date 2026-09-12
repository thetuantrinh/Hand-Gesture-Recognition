"""Deep Learning Hand Gesture Recognition Predictor.

Loads pre-trained Keras model weights and executes windowed inference
with sliding-window majority voting for temporal stabilization.
"""

import os
import logging
from typing import Optional, List, Tuple
import numpy as np

try:
    from tensorflow.keras.models import load_model  # type: ignore
    HAS_TF = True
except ImportError:
    HAS_TF = False

from ..config import GESTURE_LABELS, DEFAULT_MODEL_PATH, FALLBACK_MODEL_PATH


logger = logging.getLogger(__name__)


class GesturePredictor:
    """Sliding-window gesture classifier with temporal majority voting.

    Parameters
    ----------
    model_path : str, optional
        Path to Keras (.h5 / SavedModel) checkpoint. If None, resolves to default.
    history_len : int, optional
        Number of frames in the majority voting buffer, by default 10.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        history_len: int = 10,
    ) -> None:
        self.labels: Tuple[str, ...] = GESTURE_LABELS
        self.history_len: int = history_len

        # Resolve model path dynamically
        self.model_path = self._resolve_model_path(model_path)
        self.model = None

        if HAS_TF and self.model_path and os.path.exists(self.model_path):
            try:
                self.model = load_model(self.model_path)
                logger.info(f"Successfully loaded gesture recognition model from: {self.model_path}")
            except Exception as e:
                logger.error(f"Failed to load model from {self.model_path}: {e}")
        else:
            logger.warning(
                f"Model file not found at '{self.model_path}'. Running in placeholder/simulation mode."
            )

        # Sliding window vote buffers (initialized to class 9: Unknown / Idle)
        self.vote_history: np.ndarray = np.ones((self.history_len,), dtype=np.int8) * 9
        self.transition_buffer: np.ndarray = np.ones((2,), dtype=np.int8) * 9

    @staticmethod
    def _resolve_model_path(path: Optional[str]) -> str:
        """Resolve model checkpoint path with fallback options."""
        if path and os.path.exists(path):
            return path
        if os.path.exists(DEFAULT_MODEL_PATH):
            return DEFAULT_MODEL_PATH
        if os.path.exists(FALLBACK_MODEL_PATH):
            return FALLBACK_MODEL_PATH
        return path or DEFAULT_MODEL_PATH

    def prediction(self, window: np.ndarray) -> np.ndarray:
        """Execute inference on a temporal range-Doppler feature window.

        Parameters
        ----------
        window : np.ndarray
            Input tensor of shape (batch_size, time_steps, range_bins, doppler_bins).

        Returns
        -------
        np.ndarray
            Two-element array [previous_gesture_id, current_gesture_id]
            used by the control engine to detect new gesture state transitions.
        """
        if self.model is not None:
            preds = self.model.predict_on_batch(window)
            predicted_class = int(np.argmax(preds, axis=1)[0])
        else:
            # Fallback mock prediction when model is not available
            predicted_class = 0

        # Update sliding history
        self.vote_history = np.roll(self.vote_history, -1)
        self.vote_history[-1] = predicted_class

        # Majority vote across recent window
        majority_vote = int(np.argmax(np.bincount(self.vote_history, minlength=10)))

        # Update transition buffer [prev, current]
        self.transition_buffer = np.array([self.transition_buffer[1], majority_vote], dtype=np.int8)

        return self.transition_buffer


# Backwards compatibility alias
PREDICTOR = GesturePredictor