"""Formatting of robot telemetry into the readout labels.

The GUI shows translations in millimetres and joint angles in degrees, while
the controller reports metres and radians, so conversion belongs here rather
than being repeated at each label.
"""

from collections.abc import Sequence

import numpy as np

#: Cartesian components of a TCP pose that are translations, not rotations.
TRANSLATION_AXES = 3


class TelemetryView:
    """Writes UR3 telemetry into the pose and joint readout labels.

    Parameters
    ----------
    tcp_labels:
        The six ``QLabel`` widgets for ``[x, y, z, rx, ry, rz]``.
    joint_labels:
        The six ``QLabel`` widgets for ``[base, shoulder, elbow, w1, w2, w3]``.
    """

    def __init__(self, tcp_labels: Sequence, joint_labels: Sequence) -> None:
        self.tcp_labels = list(tcp_labels)
        self.joint_labels = list(joint_labels)

    def update_tcp_pose(self, pose: Sequence[float]) -> None:
        """Show a TCP pose: translations in millimetres, rotations in radians."""
        for index, (label, value) in enumerate(zip(self.tcp_labels, pose)):
            millimetres = index < TRANSLATION_AXES
            label.setText(f"{value * 1000:.2f}" if millimetres else f"{value:.2f}")

    def update_joint_positions(self, joints: Sequence[float]) -> None:
        """Show joint angles, converting radians to degrees."""
        for label, radians in zip(self.joint_labels, joints):
            label.setText(f"{np.degrees(radians):.2f}")
