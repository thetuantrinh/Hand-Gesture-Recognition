"""The composed main-window layout.

Widget construction began as ``pyuic5`` output from ``ui/designer/UI.ui`` and
has been hand-maintained since; it is split across the panel mixins in this
package, each contributing one region of the window.

.. important::

   The layout positions every widget with absolute geometry, so **creation
   order determines stacking order**. :data:`PANELS` therefore fixes the build
   sequence, and panels must not be reordered without checking the window for
   widgets that have disappeared behind their neighbours.
"""

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtGui import QIcon

from ..resources import asset
from .joint_panel import JointReadoutPanel
from .jog_panel import JogPanel
from .monitor_panel import MonitorPanel
from .radar_panel import RadarConfigPanel
from .robot_panel import RobotConfigPanel
from .shell import WindowShell
from .tcp_panel import TcpReadoutPanel
from .toolbar_panel import ToolbarPanel

#: Panels in build order; see the stacking-order note above.
PANELS = (
    "shell",
    "radar_panel",
    "robot_panel",
    "jog_panel",
    "monitor_panel",
    "tcp_panel",
    "toolbar_panel",
    "joint_panel",
)


class MainWindowLayout(
    WindowShell,
    RadarConfigPanel,
    RobotConfigPanel,
    JogPanel,
    MonitorPanel,
    TcpReadoutPanel,
    ToolbarPanel,
    JointReadoutPanel,
):
    """Builds every widget of the main window and exposes them as attributes.

    This class is layout only: it creates and positions widgets and sets their
    display strings. Behaviour — signal connections, acquisition, inference and
    robot control — belongs to
    :class:`~radar_hgr.app.main_window.MainWindow`, which derives from it.
    """

    def setupUi(self, MainWindow: QtWidgets.QMainWindow) -> None:
        """Construct the window's widget tree.

        Named for the Qt Designer convention so the class stays a drop-in
        replacement for generated layout code.
        """
        for panel in PANELS:
            getattr(self, f"build_{panel}")(MainWindow)

        self._build_exit_button(MainWindow)
        self._finalise(MainWindow)

    def _build_exit_button(self, MainWindow: QtWidgets.QMainWindow) -> None:
        """Create the quit button, which closes the window directly."""
        self.btn_EXIT = QtWidgets.QPushButton(self.tab2)
        self.btn_EXIT.setGeometry(QtCore.QRect(1260, 780, 101, 101))
        self.btn_EXIT.setIcon(QIcon(asset("icon/quit.png")))
        self.btn_EXIT.setIconSize(QtCore.QSize(100, 100))
        self.btn_EXIT.setObjectName("btn_EXIT")
        self.btn_EXIT.clicked.connect(MainWindow.close)

    def _finalise(self, MainWindow: QtWidgets.QMainWindow) -> None:
        """Install the tab, apply display strings and wire auto-connected slots."""
        self.A_tab.addTab(self.tab2, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.retranslateUi(MainWindow)
        self.A_tab.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow: QtWidgets.QMainWindow) -> None:
        """Apply every panel's display strings."""
        _translate = QtCore.QCoreApplication.translate
        for panel in PANELS:
            getattr(self, f"retranslate_{panel}")(MainWindow, _translate)
