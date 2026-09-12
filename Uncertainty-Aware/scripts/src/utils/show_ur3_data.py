"""Telemetry display update helpers for Universal Robots UR3 GUI widgets."""

from typing import Any
import numpy as np


def update_tcp_display(
    x: float,
    y: float,
    z: float,
    rx: float,
    ry: float,
    rz: float,
    lbl_x: Any,
    lbl_y: Any,
    lbl_z: Any,
    lbl_rx: Any,
    lbl_ry: Any,
    lbl_rz: Any,
) -> None:
    """Update UI QLabel widgets displaying UR3 Tool Center Point (TCP) coordinates.

    Converts X, Y, Z meters to millimeters and formats rotation values.
    """
    lbl_x.setText(f"{x * 1000:.2f}")
    lbl_y.setText(f"{y * 1000:.2f}")
    lbl_z.setText(f"{z * 1000:.2f}")

    lbl_rx.setText(f"{rx:.2f}")
    lbl_ry.setText(f"{ry:.2f}")
    lbl_rz.setText(f"{rz:.2f}")


def update_joint_display(
    b: float,
    s: float,
    e: float,
    w1: float,
    w2: float,
    w3: float,
    lbl_b: Any,
    lbl_s: Any,
    lbl_e: Any,
    lbl_w1: Any,
    lbl_w2: Any,
    lbl_w3: Any,
) -> None:
    """Update UI QLabel widgets displaying UR3 joint angles in degrees."""
    lbl_b.setText(f"{np.degrees(b):.2f}")
    lbl_s.setText(f"{np.degrees(s):.2f}")
    lbl_e.setText(f"{np.degrees(e):.2f}")
    lbl_w1.setText(f"{np.degrees(w1):.2f}")
    lbl_w2.setText(f"{np.degrees(w2):.2f}")
    lbl_w3.setText(f"{np.degrees(w3):.2f}")


# Backwards compatibility aliases
SHOW_UR3_TCP_POS = update_tcp_display
SHOW_UR3_JOINT_POS = update_joint_display