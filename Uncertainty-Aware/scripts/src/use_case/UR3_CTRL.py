"""Universal Robots UR3 Control Execution Module.

Maps classified radar gesture signals into real-time Cartesian TCP displacements
and end-effector tool commands (Clamp / Release) via Ethernet.
"""

from typing import Any, List, Sequence


class AutoController:
    """Automated robotic arm actuation driven by real-time gesture classification.

    Parameters
    ----------
    ur3 : Any
        UR3 interface wrapper containing robot communication client.
    tcp_position : Sequence[float]
        Current actual Tool Center Point (TCP) pose [X, Y, Z, Rx, Ry, Rz].
    step_distance : float, optional
        Cartesian translation step in meters, by default 0.030 (30 mm).
    """

    def __init__(
        self,
        ur3: Any,
        tcp_position: Sequence[float],
        step_distance: float = 0.030,
    ) -> None:
        self.ur3 = ur3
        self.dis: float = step_distance
        self.tcp_position: List[float] = list(tcp_position)

    def thread_control(self, gesture_signal: int) -> None:
        """Dispatch robot motion command based on recognized gesture ID.

        Parameters
        ----------
        gesture_signal : int
            Gesture class ID (0 to 9).
        """
        pose = list(self.tcp_position)

        if gesture_signal == 3:
            # Lowering the workpiece (Z - 30mm)
            pose[2] -= self.dis
            self._set_pose(pose)

        elif gesture_signal == 4:
            # Lifting the workpiece (Z + 30mm)
            pose[2] += self.dis
            self._set_pose(pose)

        elif gesture_signal == 5:
            # Release gripper
            self._set_tool_state("RELEASE")

        elif gesture_signal == 6:
            # Clamp gripper
            self._set_tool_state("CLAMP")

        elif gesture_signal == 7:
            # Move left (X + 30mm)
            pose[0] += self.dis
            self._set_pose(pose)

        elif gesture_signal == 8:
            # Move right (X - 30mm)
            pose[0] -= self.dis
            self._set_pose(pose)

    def _set_pose(self, target_pose: List[float]) -> None:
        """Send target Cartesian pose to robot controller."""
        try:
            self.ur3.control_robot.robot.set_realtime_pose(target_pose)
        except Exception as e:
            print(f"Error setting UR3 pose: {e}")

    def _set_tool_state(self, state: str) -> None:
        """Actuate gripper tool state."""
        try:
            self.ur3.control_robot.robot.set_tools(STATE=state)
        except Exception as e:
            print(f"Error setting tool state: {e}")


class ManualController:
    """Manual GUI-driven jog controls for UR3 Cartesian position and tools.

    Parameters
    ----------
    ur3 : Any
        UR3 interface wrapper.
    tcp_position : Sequence[float]
        Current TCP pose [X, Y, Z, Rx, Ry, Rz].
    step_distance : float, optional
        Cartesian jog step in meters, default 0.030.
    """

    def __init__(
        self,
        ur3: Any,
        tcp_position: Sequence[float],
        step_distance: float = 0.030,
    ) -> None:
        self.ur3 = ur3
        self.dis: float = step_distance
        self.tcp_position: List[float] = list(tcp_position)

    def _move_offset(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        """Apply Cartesian translation offset to current TCP pose."""
        pose = list(self.tcp_position)
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        try:
            self.ur3.control_robot.robot.set_realtime_pose(pose)
        except Exception as e:
            print(f"Error executing manual move: {e}")

    def up_state(self) -> None:
        """Jog Z positive (Up)."""
        self._move_offset(dz=self.dis)

    def down_state(self) -> None:
        """Jog Z negative (Down)."""
        self._move_offset(dz=-self.dis)

    def in_state(self) -> None:
        """Jog Y negative (Inward)."""
        self._move_offset(dy=-self.dis)

    def out_state(self) -> None:
        """Jog Y positive (Outward)."""
        self._move_offset(dy=self.dis)

    def left_state(self) -> None:
        """Jog X negative (Left)."""
        self._move_offset(dx=-self.dis)

    def right_state(self) -> None:
        """Jog X positive (Right)."""
        self._move_offset(dx=self.dis)

    def clamp_state(self) -> None:
        """Actuate tool gripper to CLAMP."""
        try:
            self.ur3.control_robot.robot.set_tools(STATE="CLAMP")
        except Exception as e:
            print(f"Error clamping gripper: {e}")

    def release_state(self) -> None:
        """Actuate tool gripper to RELEASE."""
        try:
            self.ur3.control_robot.robot.set_tools(STATE="RELEASE")
        except Exception as e:
            print(f"Error releasing gripper: {e}")


# Backwards compatibility aliases
AUTO = AutoController
MANUAL = ManualController