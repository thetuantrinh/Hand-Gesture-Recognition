"""Background polling of UR3 telemetry."""

import logging
from dataclasses import dataclass

from PyQt5.QtCore import QRunnable, pyqtSlot

from ..robot.ur3 import UR3Interface
from .signals import RobotSignals

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotTelemetry:
    """One synchronised reading of the robot's measured state.

    Attributes
    ----------
    tcp_pose:
        ``[x, y, z, rx, ry, rz]``, in metres and radians.
    joint_positions:
        ``[base, shoulder, elbow, w1, w2, w3]``, in radians.
    """

    tcp_pose: list[float]
    joint_positions: list[float]


class RobotWorker(QRunnable):
    """Polls the robot's pose and joint angles and republishes them.

    Parameters
    ----------
    robot:
        Connected robot interface. As with the radar worker, the connection's
        lifecycle belongs to the session, not to the worker.
    """

    def __init__(self, robot: UR3Interface) -> None:
        super().__init__()
        self.robot = robot
        self.signals = RobotSignals()
        self._running = False

    def stop(self) -> None:
        """Ask the loop to finish after the reading in flight."""
        self._running = False

    @pyqtSlot()
    def run(self) -> None:
        """Read telemetry until stopped or until the connection fails."""
        self._running = True
        reason = "Robot telemetry stopped"

        while self._running:
            try:
                telemetry = RobotTelemetry(
                    tcp_pose=self.robot.read_tcp_pose(),
                    joint_positions=self.robot.read_joint_positions(),
                )
            except Exception:
                logger.exception("Robot telemetry failed")
                reason = "Robot telemetry failed; see the log for details"
                break

            self.signals.telemetry_received.emit(telemetry)

        self.signals.stopped.emit(reason)
