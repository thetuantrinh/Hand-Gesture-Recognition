"""UR3 manipulator motion parameters and workspace limits."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotConfig:
    """Trajectory parameters for gesture-driven UR3 actuation.

    Attributes
    ----------
    step_distance:
        Cartesian translation applied per recognised gesture, in metres.
    acceleration:
        Joint acceleration limit, in rad/s^2.
    velocity:
        Joint velocity limit, in rad/s.
    home_joints:
        Homing joint vector ``[base, shoulder, elbow, w1, w2, w3]``, in degrees.
    home_tcp_pose:
        Startup Tool Center Point pose ``[x, y, z, rx, ry, rz]`` (m, rad).
    """

    step_distance: float = 0.030
    acceleration: float = 0.9
    velocity: float = 1.0
    home_joints: list[float] = field(
        default_factory=lambda: [55.84, -73.91, 139.98, -195.87, -66.93, -203.18]
    )
    home_tcp_pose: list[float] = field(
        default_factory=lambda: [-0.0366, -0.3664, 0.1467, 2.8804, -0.5615, -1.0891]
    )


@dataclass(frozen=True)
class WorkspaceLimits:
    """Cartesian bounding box the tool is allowed to occupy, in metres.

    Applied by :class:`radar_hgr.robot.safety.SafetySupervisor`. The limits are
    opt-in: the gesture controller only consults them when a supervisor is
    supplied, so an unconfigured deployment retains full range of motion.
    """

    z_min: float = -0.40325
    z_max: float = -0.360

    def contains_z(self, z: float) -> bool:
        """Whether height ``z`` lies inside the permitted band."""
        return self.z_min <= z <= self.z_max
