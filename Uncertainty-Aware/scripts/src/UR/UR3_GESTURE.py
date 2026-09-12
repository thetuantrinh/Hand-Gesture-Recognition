"""Universal Robots UR3 High-Level Gesture Actuation Interface.

Initializes RTDE / Real-time client connection to physical UR3 controller or URSim,
executes homing sequence, and streams current joint / TCP pose vectors.
"""

import time
import logging
from typing import List, Optional, Union
import numpy as np

from src import UR
from ..config import NetworkConfig, RobotConfig

logger = logging.getLogger(__name__)


class UR3GestureInterface:
    """High-level interface for UR3 manipulator control and state acquisition.

    Parameters
    ----------
    robot_ip : Optional[str], optional
        Target UR3 controller IP address. Defaults to NetworkConfig.ur3_ip.
    """

    def __init__(self, robot_ip: Optional[str] = None) -> None:
        self.net_cfg = NetworkConfig()
        self.robot_cfg = RobotConfig()
        self.robot_ip: str = robot_ip or self.net_cfg.ur3_ip

        logger.info(f"Connecting to Universal Robots UR3 at {self.robot_ip}...")
        self.robot_model = UR.robotModel.RobotModel()
        self.robot = UR.urScriptExt.UrScriptExt(
            host=self.robot_ip,
            robotModel=self.robot_model,
        )
        self.robot.reset_error()
        logger.info("UR3 connected and cleared of errors.")
        time.sleep(1.0)

        self.acceleration: float = self.robot_cfg.acceleration
        self.velocity: float = self.robot_cfg.velocity
        self.start_pos: List[float] = list(self.robot_cfg.default_home_joints)

        # Initialize tool state and home position
        self.robot.set_tools(STATE="RELEASE")
        self.robot.movej(
            q=np.radians(self.start_pos),
            a=self.acceleration,
            v=self.velocity,
        )

        # Start real-time control loop
        self.robot.init_realtime_control()
        time.sleep(1.5)

    def read_ur_data(
        self,
        fps: int = 20,
        read_data: str = "TCP Pos",
    ) -> List[float]:
        """Read actual Cartesian Tool Center Point (TCP) pose or joint angular vector.

        Parameters
        ----------
        fps : int, optional
            Data polling rate limit in frames per second, default 20.
        read_data : str, optional
            Either 'TCP Pos' ([X, Y, Z, Rx, Ry, Rz]) or
            'joint Pos' ([Base, Shoulder, Elbow, W1, W2, W3]), default 'TCP Pos'.

        Returns
        -------
        List[float]
            Telemetry pose vector.
        """
        if read_data == "TCP Pos":
            return list(self.robot.get_actual_tcp_pose())
        elif read_data == "joint Pos":
            return list(self.robot.get_actual_joint_positions())
        return []

    def close(self) -> None:
        """Safely terminate robot connection and free socket."""
        logger.info("Closing UR3 controller connection...")
        try:
            self.robot.close()
        except Exception as e:
            logger.error(f"Error closing UR3 connection: {e}")


# Backwards compatibility alias
GES_POS = UR3GestureInterface
