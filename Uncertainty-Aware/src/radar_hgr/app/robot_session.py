"""Lifecycle of one UR3 connection."""

import logging
from typing import Callable, Optional

from PyQt5.QtCore import QThreadPool

from ..config.network import RobotNetworkConfig
from ..config.robot import RobotConfig
from ..robot.controllers import GestureController, JogController
from ..robot.safety import SafetySupervisor
from ..robot.ur3 import UR3Interface
from ..workers.robot_worker import RobotTelemetry, RobotWorker

logger = logging.getLogger(__name__)


class RobotSession:
    """Connects to the UR3, polls its telemetry, and issues motion commands.

    The last telemetry reading is retained as :attr:`tcp_pose`, because motion
    commands are computed as offsets from the robot's measured pose rather than
    from a pose the application believes it should be at.

    Parameters
    ----------
    on_telemetry:
        Called on the GUI thread for every telemetry reading.
    network_config, robot_config:
        Configuration objects; defaults are used when omitted.
    supervisor:
        Optional workspace supervisor applied to every commanded pose.
    thread_pool:
        Pool to run the worker in. Defaults to the global instance.
    """

    def __init__(
        self,
        on_telemetry: Callable[[RobotTelemetry], None],
        network_config: Optional[RobotNetworkConfig] = None,
        robot_config: Optional[RobotConfig] = None,
        supervisor: Optional[SafetySupervisor] = None,
        thread_pool: Optional[QThreadPool] = None,
    ) -> None:
        self.on_telemetry = on_telemetry
        self.network_config = network_config or RobotNetworkConfig()
        self.robot_config = robot_config or RobotConfig()
        self.supervisor = supervisor
        self.thread_pool = thread_pool or QThreadPool.globalInstance()

        self.tcp_pose = list(self.robot_config.home_tcp_pose)
        self._robot: Optional[UR3Interface] = None
        self._worker: Optional[RobotWorker] = None

    @property
    def connected(self) -> bool:
        """Whether a robot connection is currently held."""
        return self._robot is not None

    def start(self) -> None:
        """Connect to the controller, home it, and begin polling telemetry.

        Raises
        ------
        Exception
            Propagated from the SDK when the controller is unreachable.
        """
        if self.connected:
            logger.debug("Robot session already running")
            return

        robot = UR3Interface(self.network_config, self.robot_config)

        worker = RobotWorker(robot)
        worker.setAutoDelete(False)
        worker.signals.telemetry_received.connect(self._consume_telemetry)

        self._robot, self._worker = robot, worker
        self.thread_pool.start(worker)
        logger.info("Robot session started")

    def stop(self) -> None:
        """Stop polling and close the controller connection."""
        if self._worker is not None:
            self._worker.stop()
            self._worker = None

        if self._robot is not None:
            self._robot.close()
            self._robot = None

        logger.info("Robot session stopped")

    def gesture_controller(self) -> Optional[GestureController]:
        """Build a controller bound to the latest measured pose, or ``None`` if not connected."""
        if self._robot is None:
            return None
        return GestureController(
            self._robot, self.tcp_pose, self.robot_config, self.supervisor
        )

    def jog_controller(self) -> Optional[JogController]:
        """Jog controller bound to the latest measured pose, or ``None`` if not connected."""
        if self._robot is None:
            return None
        return JogController(
            self._robot, self.tcp_pose, self.robot_config, self.supervisor
        )

    def _consume_telemetry(self, telemetry: RobotTelemetry) -> None:
        """Retain the measured pose and forward the reading to the GUI."""
        self.tcp_pose = list(telemetry.tcp_pose)
        self.on_telemetry(telemetry)
