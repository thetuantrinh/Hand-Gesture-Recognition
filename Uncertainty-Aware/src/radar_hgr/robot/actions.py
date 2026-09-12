"""The mapping from recognised gesture to robot action.

Expressing the mapping as data rather than as a chain of ``if`` branches keeps
the gesture vocabulary in one auditable place and lets it be asserted against
in tests without a robot attached.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from ..config.gestures import Gesture


class ToolState(str, Enum):
    """Gripper states understood by the URScript tool command."""

    CLAMP = "CLAMP"
    RELEASE = "RELEASE"


@dataclass(frozen=True)
class RobotAction:
    """One robot command derived from a gesture.

    Exactly one of :attr:`translation` and :attr:`tool_state` is set.

    Attributes
    ----------
    translation:
        Cartesian offset as multiples of the configured step distance, ordered
        ``(x, y, z)``.
    tool_state:
        Gripper state to actuate.
    """

    translation: Optional[tuple[int, int, int]] = None
    tool_state: Optional[ToolState] = None

    def offsets(self, step_distance: float) -> tuple[float, float, float]:
        """Scale :attr:`translation` into metres."""
        if self.translation is None:
            return (0.0, 0.0, 0.0)
        return tuple(axis * step_distance for axis in self.translation)


#: Gestures that command the manipulator. Gestures absent from this table
#: (``EMPTY``, the two rotations and ``UNKNOWN``) are recognised but inert.
GESTURE_ACTIONS: dict[Gesture, RobotAction] = {
    Gesture.LOWER_WORKPIECE: RobotAction(translation=(0, 0, -1)),
    Gesture.LIFT_WORKPIECE: RobotAction(translation=(0, 0, +1)),
    Gesture.RELEASE_WORKPIECE: RobotAction(tool_state=ToolState.RELEASE),
    Gesture.CLAMP_WORKPIECE: RobotAction(tool_state=ToolState.CLAMP),
    Gesture.MOVE_LEFT: RobotAction(translation=(+1, 0, 0)),
    Gesture.MOVE_RIGHT: RobotAction(translation=(-1, 0, 0)),
}


def action_for(gesture: Gesture) -> Optional[RobotAction]:
    """Return the action a gesture commands, or ``None`` when it commands nothing."""
    return GESTURE_ACTIONS.get(gesture)


def translate(pose: Sequence[float], offsets: tuple[float, float, float]) -> list:
    """Apply a Cartesian offset to a pose, leaving its orientation untouched.

    Parameters
    ----------
    pose:
        TCP pose ``[x, y, z, rx, ry, rz]``.
    offsets:
        ``(dx, dy, dz)`` in metres.

    Returns
    -------
    list
        A new pose; the input is not modified.

    Examples
    --------
    >>> translate([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], (0.0, 0.0, -0.03))
    [1.0, 2.0, 2.97, 0.1, 0.2, 0.3]
    """
    moved = list(pose)
    for axis, delta in enumerate(offsets):
        moved[axis] += delta
    return moved
