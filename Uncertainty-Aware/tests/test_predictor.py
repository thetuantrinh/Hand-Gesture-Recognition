"""Tests for the predictor's composition of a classifier and the vote."""

import numpy as np

from radar_hgr.config.gestures import Gesture
from radar_hgr.inference.predictor import GesturePredictor

WINDOW = np.zeros((1, 20, 40, 8), dtype=np.float32)


class ConstantClassifier:
    """Always predicts one class, and counts how often it was called."""

    def __init__(self, gesture):
        self.gesture = gesture
        self.calls = 0

    def predict_class(self, features):
        self.calls += 1
        return int(self.gesture)


def test_a_predictor_without_a_model_reports_unavailable(tmp_path, monkeypatch):
    monkeypatch.setenv("RADAR_HGR_MODELS_DIR", str(tmp_path))
    monkeypatch.delenv("RADAR_HGR_MODEL", raising=False)

    predictor = GesturePredictor()

    assert not predictor.available
    assert predictor.model_name == "None"
    assert predictor.predict(WINDOW) is None


def test_an_injected_classifier_is_used_without_loading_a_checkpoint():
    classifier = ConstantClassifier(Gesture.MOVE_RIGHT)
    predictor = GesturePredictor(classifier=classifier, vote_window=3)

    for _ in range(3):
        predictor.predict(WINDOW)

    assert predictor.available
    assert classifier.calls == 3
    assert predictor.state is Gesture.MOVE_RIGHT


def test_the_state_changes_only_once_for_a_sustained_gesture():
    classifier = ConstantClassifier(Gesture.CLAMP_WORKPIECE)
    predictor = GesturePredictor(classifier=classifier, vote_window=3)
    transitions = [predictor.predict(WINDOW) for _ in range(8)]

    assert sum(t.changed for t in transitions) == 1


def test_reset_clears_the_vote():
    classifier = ConstantClassifier(Gesture.LIFT_WORKPIECE)
    predictor = GesturePredictor(classifier=classifier, vote_window=3)
    for _ in range(3):
        predictor.predict(WINDOW)

    predictor.reset()

    assert predictor.state is Gesture.UNKNOWN
