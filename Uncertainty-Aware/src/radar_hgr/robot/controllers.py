"""Translation of gestures and GUI jog input into UR3 motion commands."""

import logging
from collections.abc import Sequence
from typing import Optional

from ..config.gestures import Gesture
from ..config.robot import RobotConfig
from .actions import ToolState, action_for, translate
from .safety import SafetySupervisor

logger = logging.getLogger(__name__)


class _CartesianController:
    """Shared pose bookkeeping for the gesture and jog controllers.

    Parameters
    ----------
    robot:
        Connected :class:`~radar_hgr.robot.ur3.UR3Interface`.
    tcp_pose:
        Tool pose the command is computed relative to. This is a snapshot of
        the last telemetry reading, not a live reference.
    config:
        Motion parameters. Defaults to :class:`RobotConfig`.
    supervisor:
        Optional workspace supervisor. When supplied, commanded poses it
        rejects are dropped.
    """

    def __init__(
        self,
        robot,
        tcp_pose: Sequence[float],
        config: Optional[RobotConfig] = None,
        supervisor: Optional[SafetySupervisor] = None,
    ) -> None:
        self.robot = robot
        self.config = config or RobotConfig()
        self.supervisor = supervisor
        self.tcp_pose = list(tcp_pose)

    @property
    def step_distance(self) -> float:
        """Cartesian step applied per command, in metres."""
        return self.config.step_distance

    def _move_by(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> bool:
        """Command a relative Cartesian move.

        Returns
        -------
        bool
            Whether the command was sent. ``False`` means the supervisor
            vetoed the target pose.
        """
        target = translate(self.tcp_pose, (dx, dy, dz))
        if self.supervisor is not None and not self.supervisor.permits(target):
            return False
        self.robot.set_pose(target)
        return True

    def _actuate_tool(self, state: ToolState) -> None:
        """Command a gripper state."""
        self.robot.set_tool_state(state)


class GestureController(_CartesianController):
    """Actuates the manipulator from classified gestures.

    The gesture-to-motion vocabulary lives in
    :data:`~radar_hgr.robot.actions.GESTURE_ACTIONS`; this class only executes
    it.
    """

    def execute(self, gesture: Gesture) -> bool:
        """Perform the action a gesture commands.

        Parameters
        ----------
        gesture:
            Smoothed gesture to act on.

        Returns
        -------
        bool
            Whether a command was sent to the robot. ``False`` means the
            gesture commands nothing, or its target pose was vetoed.
        """
        action = action_for(gesture)
        if action is None:
            logger.debug("Gesture %s commands no motion", gesture.name)
            return False

        if action.tool_state is not None:
            self._actuate_tool(action.tool_state)
            return True

        return self._move_by(*action.offsets(self.step_distance))


class JogController(_CartesianController):
    """Manual Cartesian jog and gripper control driven by the GUI jog pad.

    Axis conventions follow the operator's view of the cell: "in" and "out"
    move along -Y and +Y, "left" and "right" along -X and +X.
    """

    def up(self) -> bool:
        """Jog +Z."""
        return self._move_by(dz=self.step_distance)

    def down(self) -> bool:
        """Jog -Z."""
        return self._move_by(dz=-self.step_distance)

    def inward(self) -> bool:
        """Jog -Y, away from the operator."""
        return self._move_by(dy=-self.step_distance)

    def outward(self) -> bool:
        """Jog +Y, towards the operator."""
        return self._move_by(dy=self.step_distance)

    def left(self) -> bool:
        """Jog -X."""
        return self._move_by(dx=-self.step_distance)

    def right(self) -> bool:
        """Jog +X."""
        return self._move_by(dx=self.step_distance)

    def clamp(self) -> None:
        """Close the gripper."""
        self._actuate_tool(ToolState.CLAMP)

    def release(self) -> None:
        """Open the gripper."""
        self._actuate_tool(ToolState.RELEASE)
