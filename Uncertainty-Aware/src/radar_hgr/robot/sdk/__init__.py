"""Vendored Universal Robots RTDE / URScript interface.

Third-party code, derived from the ``URBasic`` library, kept in-tree because the
project pins behaviour the upstream releases have since changed. It is
deliberately quarantined from the rest of ``radar_hgr``: the naming and style
are upstream's, and the only supported entry point is
:class:`~radar_hgr.robot.ur3.UR3Interface`.

Local modifications, all mechanical:

* absolute ``src.UR`` imports became package-relative imports;
* ``pkg_resources.resource_filename`` was replaced by :mod:`._resources`,
  since ``pkg_resources`` is deprecated;
* the default log directory no longer depends on the pre-rename ``URBasic``
  package name.
"""

from .connectionState import ConnectionState
from .dashboard import DashBoard
from .dataLog import DataLog
from .dataLogging import DataLogging
from .realTimeClient import RealTimeClient
from .robotConnector import RobotConnector
from .robotModel import RobotModel
from .rtde import RTDE
from .urScript import UrScript
from .urScriptExt import UrScriptExt

__all__ = [
    "ConnectionState",
    "DashBoard",
    "DataLog",
    "DataLogging",
    "RTDE",
    "RealTimeClient",
    "RobotConnector",
    "RobotModel",
    "UrScript",
    "UrScriptExt",
]
