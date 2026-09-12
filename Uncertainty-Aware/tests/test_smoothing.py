"""Tests for temporal stabilisation of predictions."""

import pytest

from radar_hgr.config.gestures import Gesture
from radar_hgr.inference.smoothing import GestureTransition, MajorityVoteSmoother


def test_rejects_a_non_positive_window():
    with pytest.raises(ValueError):
        MajorityVoteSmoother(window_size=0)


def test_starts_in_the_initial_state():
    smoother = MajorityVoteSmoother(window_size=5, initial=Gesture.UNKNOWN)
    assert smoother.state is Gesture.UNKNOWN


def test_a_single_outlier_does_not_move_the_state():
    """One misclassified frame must not command a robot motion."""
    smoother = MajorityVoteSmoother(window_size=5, initial=Gesture.UNKNOWN)
    transition = smoother.update(Gesture.CLAMP_WORKPIECE)

    assert not transition.changed
    assert smoother.state is Gesture.UNKNOWN


def test_a_sustained_gesture_becomes_the_state():
    smoother = MajorityVoteSmoother(window_size=5, initial=Gesture.UNKNOWN)
    for _ in range(3):
        smoother.update(Gesture.MOVE_LEFT)

    assert smoother.state is Gesture.MOVE_LEFT


def test_the_transition_reports_the_previous_state():
    smoother = MajorityVoteSmoother(window_size=3, initial=Gesture.UNKNOWN)
    smoother.update(Gesture.MOVE_LEFT)
    transition = smoother.update(Gesture.MOVE_LEFT)

    assert transition.changed
    assert transition.previous is Gesture.UNKNOWN
    assert transition.current is Gesture.MOVE_LEFT


def test_the_state_holds_while_a_gesture_is_sustained():
    """Only changes command motion; a held gesture must not repeat it."""
    smoother = MajorityVoteSmoother(window_size=3, initial=Gesture.UNKNOWN)
    transitions = [smoother.update(Gesture.LIFT_WORKPIECE) for _ in range(6)]

    assert sum(t.changed for t in transitions) == 1


def test_reset_returns_to_the_initial_state():
    smoother = MajorityVoteSmoother(window_size=3, initial=Gesture.UNKNOWN)
    for _ in range(3):
        smoother.update(Gesture.CLOCKWISE)

    smoother.reset()

    assert smoother.state is Gesture.UNKNOWN


def test_an_unchanged_transition_is_not_actionable():
    held = GestureTransition(previous=Gesture.EMPTY, current=Gesture.EMPTY)
    assert not held.changed
