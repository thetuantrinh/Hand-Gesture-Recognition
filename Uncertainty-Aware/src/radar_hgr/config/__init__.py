"""Declarative configuration for the real-time gesture recognition system.

Every tunable value lives in a frozen dataclass in this package rather than as
a literal at its point of use, so a deployment can be described by the set of
configuration objects it constructs.
"""

from .gestures import GESTURE_LABELS, NUM_CLASSES, Gesture, label_for
from .network import RadarNetworkConfig, RobotNetworkConfig
from .paths import models_dir, radar_profile_dir, resolve_model_path
from .pipeline import PipelineConfig
from .radar import RadarConfig
from .robot import RobotConfig, WorkspaceLimits

__all__ = [
    "GESTURE_LABELS",
    "NUM_CLASSES",
    "Gesture",
    "label_for",
    "PipelineConfig",
    "RadarConfig",
    "RadarNetworkConfig",
    "RobotConfig",
    "RobotNetworkConfig",
    "WorkspaceLimits",
    "models_dir",
    "radar_profile_dir",
    "resolve_model_path",
]
