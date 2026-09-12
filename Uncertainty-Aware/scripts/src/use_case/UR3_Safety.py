"""Universal Robots UR3 Workspace Safety Monitoring Module.

Enforces Cartesian bounding-box limits to prevent collisions or out-of-bounds actuation.
"""

from typing import List, Sequence


class SafetySupervisor:
    """Monitors and enforces robotic workspace Cartesian limits.

    Parameters
    ----------
    tcp_position : Sequence[float]
        Current TCP pose [X, Y, Z, Rx, Ry, Rz].
    z_upper : float, optional
        Upper limit for Z coordinate in meters, default -0.360.
    z_lower : float, optional
        Lower limit for Z coordinate in meters, default -0.40325.
    """

    def __init__(
        self,
        tcp_position: Sequence[float],
        z_upper: float = -0.360,
        z_lower: float = -0.40325,
    ) -> None:
        self.tcp_position: List[float] = list(tcp_position)
        self.z_upper: float = z_upper
        self.z_lower: float = z_lower

    def is_within_limits(self) -> bool:
        """Check if current TCP position resides safely within defined boundaries.

        Returns
        -------
        bool
            True if within safe workspace, False otherwise.
        """
        z = self.tcp_position[2]
        return self.z_lower <= z <= self.z_upper

    def limit_space_work(self) -> bool:
        """Enforce Cartesian workspace limits.

        Returns
        -------
        bool
            True if within safe limits, False if safety boundary was exceeded.
        """
        z = self.tcp_position[2]
        if abs(z) <= abs(self.z_upper):
            return False
        elif abs(z) >= abs(self.z_lower):
            return False
        return True


# Backwards compatibility alias
SAFETY = SafetySupervisor
