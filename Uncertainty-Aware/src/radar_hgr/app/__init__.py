"""Application layer: window wiring and the acquisition / control sessions.

:mod:`main_window` holds only Qt wiring. The work it coordinates lives in
:mod:`radar_session`, :mod:`robot_session` and :mod:`gesture_pipeline`, none of
which touch widgets.
"""

from .application import run
from .gesture_pipeline import MicroDopplerPipeline
from .main_window import MainWindow
from .radar_session import RadarSession
from .robot_session import RobotSession

__all__ = ["MainWindow", "MicroDopplerPipeline", "RadarSession", "RobotSession", "run"]
