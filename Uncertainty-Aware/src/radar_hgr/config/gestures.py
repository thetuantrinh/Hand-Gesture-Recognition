"""Gesture class vocabulary shared by the classifier and the control layer."""

from enum import IntEnum


class Gesture(IntEnum):
    """The ten gesture classes produced by the uncertainty-aware classifier.

    Members double as the classifier's output indices, so a prediction can be
    converted with ``Gesture(index)`` and compared by name at the call site.
    """

    EMPTY = 0
    COUNTER_CLOCKWISE = 1
    CLOCKWISE = 2
    LOWER_WORKPIECE = 3
    LIFT_WORKPIECE = 4
    RELEASE_WORKPIECE = 5
    CLAMP_WORKPIECE = 6
    MOVE_LEFT = 7
    MOVE_RIGHT = 8
    UNKNOWN = 9

    @property
    def label(self) -> str:
        """Human-readable name for GUI display."""
        return GESTURE_LABELS[self.value]


GESTURE_LABELS: tuple[str, ...] = (
    "Empty",
    "Counter-Clockwise",
    "Clockwise",
    "Lowering Workpiece",
    "Lifting Workpiece",
    "Release Workpiece",
    "Clamp Workpiece",
    "Move Left",
    "Move Right",
    "Unknown / Clutter",
)

NUM_CLASSES: int = len(Gesture)


def label_for(index: int) -> str:
    """Display label for a raw classifier index, tolerating out-of-range values."""
    if 0 <= index < NUM_CLASSES:
        return GESTURE_LABELS[index]
    return "Unknown"
