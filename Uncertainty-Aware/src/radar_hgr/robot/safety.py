"""Workspace supervision for commanded tool poses."""

import logging
from collections.abc import Sequence
from typing import Optional

from ..config.robot import WorkspaceLimits

logger = logging.getLogger(__name__)

#: Index of the vertical component in a ``[x, y, z, rx, ry, rz]`` pose.
Z_INDEX = 2


class SafetySupervisor:
    """Vetoes commanded poses that leave the permitted workspace.

    Parameters
    ----------
    limits:
        Permitted Cartesian bounds. Defaults to :class:`WorkspaceLimits`.

    Notes
    -----
    The supervisor is opt-in and currently constrains height only, which is the
    axis the gesture vocabulary drives into the bench. Construct one and pass it
    to :class:`~radar_hgr.robot.controllers.GestureController` to enforce it;
    the limits must match the cell the robot is actually installed in.

    Examples
    --------
    >>> supervisor = SafetySupervisor(WorkspaceLimits(z_min=-0.4, z_max=-0.3))
    >>> supervisor.permits([0.0, 0.0, -0.35, 0.0, 0.0, 0.0])
    True
    >>> supervisor.permits([0.0, 0.0, -0.2, 0.0, 0.0, 0.0])
    False
    """

    def __init__(self, limits: Optional[WorkspaceLimits] = None) -> None:
        self.limits = limits or WorkspaceLimits()

    def permits(self, pose: Sequence[float]) -> bool:
        """Whether ``pose`` lies inside the permitted workspace.

        Parameters
        ----------
        pose:
            Candidate TCP pose ``[x, y, z, rx, ry, rz]``, in metres and radians.
        """
        height = pose[Z_INDEX]
        if self.limits.contains_z(height):
            return True

        logger.warning(
            "Rejected pose: z=%.4f m outside the permitted band [%.4f, %.4f]",
            height,
            self.limits.z_min,
            self.limits.z_max,
        )
        return False
