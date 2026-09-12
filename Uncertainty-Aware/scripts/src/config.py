"""Central configuration for FMCW Radar Hand Gesture Recognition and Robotics Control.

Defines default radar parameters, network endpoints, gesture label mappings,
and robotic control constraints.
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import os


@dataclass(frozen=True)
class RadarConfig:
    """FMCW Radar hardware & ADC sampling configuration."""
    num_chirps: int = 128            # Nc: Number of chirps per frame
    num_adc_samples: int = 64        # Ns: Number of ADC samples per chirp
    num_rx_antennas: int = 4         # Nr: Number of physical RX antennas
    num_tx_antennas: int = 3         # Nt: Number of physical TX antennas
    iq_channels: int = 2             # Real (I) + Imaginary (Q)
    adc_bits: int = 16
    max_packet_size: int = 4096
    bytes_in_packet: int = 1456

    @property
    def frame_size_samples(self) -> int:
        """Total scalar samples in one complete data frame."""
        return self.num_adc_samples * self.num_rx_antennas * self.iq_channels * self.num_chirps


@dataclass(frozen=True)
class NetworkConfig:
    """UDP / TCP network endpoints for DCA1000EVM capture card and UR3 robot."""
    fpga_ip: str = "192.168.33.180"
    fpga_port: int = 4096
    host_ip: str = "192.168.33.30"
    host_cfg_port: int = 4096
    host_data_port: int = 4098
    ur3_ip: str = "169.254.200.239"
    ur3_rtde_port: int = 30004
    ur3_realtime_port: int = 30003


@dataclass
class RobotConfig:
    """Universal Robots UR3 manipulator trajectory parameters."""
    step_distance: float = 0.030     # 30 mm Cartesian step per recognized gesture
    acceleration: float = 0.9        # Joint acceleration (rad/s^2)
    velocity: float = 1.0            # Joint velocity (rad/s)
    default_home_joints: List[float] = field(
        default_factory=lambda: [55.84, -73.91, 139.98, -195.87, -66.93, -203.18]
    )
    default_tcp_pose: List[float] = field(
        default_factory=lambda: [-0.0366, -0.3664, 0.1467, 2.8804, -0.5615, -1.0891]
    )
    # Cartesian workspace safety limits (meters)
    z_min_limit: float = -0.403
    z_max_limit: float = -0.360


# 10 Gesture Class Labels
GESTURE_LABELS: Tuple[str, ...] = (
    "Empty",
    "Counter-Clockwise",
    "Clockwise",
    "Lowering Workpiece",
    "Lifting Workpiece",
    "Release Workpiece",
    "Clamp Workpiece",
    "Move Left",
    "Move Right",
    "Unknown / Clutter",
)

# Default Model Paths
SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(SCRIPTS_DIR, "models")
DEFAULT_MODEL_PATH = os.path.join(MODELS_DIR, "model_2.h5")
FALLBACK_MODEL_PATH = os.path.join(MODELS_DIR, "model_1.h5")
