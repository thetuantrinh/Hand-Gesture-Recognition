"""Loading and invocation of the trained Keras classifier.

TensorFlow is imported lazily so that the DSP, protocol and control layers, and
their tests, do not pay its import cost or require it to be installed.
"""

import logging
from pathlib import Path
from typing import Optional, Protocol

import numpy as np

logger = logging.getLogger(__name__)


class Classifier(Protocol):
    """Minimal interface the gesture predictor needs from a model."""

    def predict_class(self, features: np.ndarray) -> int:
        """Return the argmax class index for one batched feature window."""


class KerasClassifier:
    """Wraps a loaded Keras model behind :class:`Classifier`.

    Parameters
    ----------
    model:
        A loaded Keras model accepting the batched feature window.
    """

    def __init__(self, model) -> None:
        self._model = model

    def predict_class(self, features: np.ndarray) -> int:
        """Argmax class index for one batched feature window.

        Uses ``predict_on_batch`` rather than ``predict``: the latter builds a
        dataset and logs progress on every call, which is unacceptable at the
        radar frame rate.
        """
        logits = self._model.predict_on_batch(features)
        return int(np.argmax(logits, axis=1)[0])


def load_classifier(model_path: Optional[Path]) -> Optional[KerasClassifier]:
    """Load a checkpoint, or return ``None`` if it cannot be used.

    Failure is not fatal by design: the GUI must still start so the operator can
    exercise radar acquisition and manual robot control, and pick a different
    checkpoint from the model browser.

    Parameters
    ----------
    model_path:
        Checkpoint to load, typically from
        :func:`radar_hgr.config.paths.resolve_model_path`.

    Returns
    -------
    KerasClassifier or None
        The wrapped model, or ``None`` when no checkpoint was found,
        TensorFlow is unavailable, or loading failed.
    """
    if model_path is None:
        logger.warning("No gesture model found; inference is disabled")
        return None

    try:
        from tensorflow.keras.models import load_model
    except ImportError:
        logger.warning("TensorFlow is not installed; inference is disabled")
        return None

    try:
        model = load_model(str(model_path))
    except Exception:
        logger.exception("Failed to load the gesture model from %s", model_path)
        return None

    logger.info("Loaded gesture model from %s", model_path)
    return KerasClassifier(model)
