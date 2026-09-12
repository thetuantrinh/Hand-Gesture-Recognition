"""Connection to the Universal Robots UR3 controller."""

import logging
import time
from typing import Optional

import numpy as np

from ..config.network import RobotNetworkConfig
from ..config.robot import RobotConfig
from .actions import ToolState
from .sdk import RobotModel, UrScriptExt

logger = logging.getLogger(__name__)

#: Seconds to let the controller clear its error state after a reset.
ERROR_RESET_DELAY = 1.0

#: Seconds for the real-time servoj loop on the controller to come up.
REALTIME_STARTUP_DELAY = 1.5


class UR3Interface:
    """Owns the UR3 connection and exposes the commands this application needs.

    Wrapping the vendored SDK here keeps its naming conventions and its
    three-deep object graph out of the control and GUI layers, which address the
    robot through :meth:`set_pose`, :meth:`set_tool_state` and the two read
    methods only.

    Parameters
    ----------
    network:
        Controller endpoint. Defaults to :class:`RobotNetworkConfig`.
    config:
        Motion parameters used for homing. Defaults to :class:`RobotConfig`.

    Raises
    ------
    Exception
        Propagated from the SDK if the controller cannot be reached. Connecting
        is the caller's decision point, so failure is not swallowed here.
    """

    def __init__(
        self,
        network: Optional[RobotNetworkConfig] = None,
        config: Optional[RobotConfig] = None,
    ) -> None:
        self.network = network or RobotNetworkConfig()
        self.config = config or RobotConfig()

        logger.info("Connecting to UR3 at %s", self.network.ip)
        self._model = RobotModel()
        self._robot = UrScriptExt(host=self.network.ip, robotModel=self._model)
        self._robot.reset_error()
        time.sleep(ERROR_RESET_DELAY)
        logger.info("UR3 connected; error state cleared")

        self._home()

    def _home(self) -> None:
        """Open the gripper, move to the home configuration, then start servoing.

        The real-time control loop must be started after the homing move,
        because ``movej`` and the servo loop cannot both own the trajectory.
        """
        self.set_tool_state(ToolState.RELEASE)
        self._robot.movej(
            q=np.radians(self.config.home_joints),
            a=self.config.acceleration,
            v=self.config.velocity,
        )
        self._robot.init_realtime_control()
        time.sleep(REALTIME_STARTUP_DELAY)
        logger.info("UR3 homed and servoing")

    def set_pose(self, pose: list[float]) -> None:
        """Command a Cartesian TCP pose through the real-time interface."""
        self._robot.set_realtime_pose(list(pose))

    def set_tool_state(self, state: ToolState) -> None:
        """Command a gripper state."""
        self._robot.set_tools(STATE=str(state.value))

    def read_tcp_pose(self) -> list[float]:
        """Measured TCP pose ``[x, y, z, rx, ry, rz]``, in metres and radians."""
        return list(self._robot.get_actual_tcp_pose())

    def read_joint_positions(self) -> list[float]:
        """Measured joint angles ``[base, shoulder, elbow, w1, w2, w3]``, in radians."""
        return list(self._robot.get_actual_joint_positions())

    def close(self) -> None:
        """Terminate the controller connection. Safe to call more than once."""
        logger.info("Closing the UR3 connection")
        try:
            self._robot.close()
        except Exception:
            logger.exception("Error closing the UR3 connection")
