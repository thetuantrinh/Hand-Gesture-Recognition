"""Top-level window chrome: tab host, plot surface and control panel frame."""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QIcon
import pyqtgraph as pg

from ..resources import asset


class WindowShell:
    """Top-level window chrome: tab host, plot surface and control panel frame.

    Mixin contributing this panel's widgets to :class:`~radar_hgr.ui.layout.main_window.Ui_MainWindow`.
    """

    def build_shell(self, MainWindow):
        """Instantiate and position this panel's widgets."""
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1410, 960)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.A_tab = QtWidgets.QTabWidget(self.centralwidget)
        self.A_tab.setGeometry(QtCore.QRect(0, 0, 1911, 960))

        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        self.A_tab.setFont(font)
        self.A_tab.setStyleSheet("background-color: qradialgradient(spread:reflect, cx:0.5, cy:0.522727, radius:0.5, fx:0.507, fy:0.516636, stop:0.244318 rgba(255, 255, 255, 255), stop:0.715909 rgba(255, 255, 255, 255), stop:0.960227 rgba(230, 255, 253, 255));")
        self.A_tab.setObjectName("A_tab")
        self.tab2 = QtWidgets.QWidget()
        self.tab2.setObjectName("tab2")

        self.graph = pg.ImageView(self.tab2)
        self.graph.setGeometry(QtCore.QRect(520, 135, 895, 615))
        self.graph.ui.histogram.hide()
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        self.graph.setFont(font)
        self.graph.setObjectName("graph")

        self.control_pannel = QtWidgets.QGroupBox(self.tab2)
        self.control_pannel.setGeometry(QtCore.QRect(10, 100, 501, 820))
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        self.control_pannel.setFont(font)
        self.control_pannel.setStyleSheet("background-color: qradialgradient(spread:reflect, cx:0.5, cy:0.522727, radius:0.5, fx:0.507, fy:0.516636, stop:0.244318 rgba(255, 255, 255, 255), stop:0.715909 rgba(255, 255, 255, 255), stop:0.960227 rgba(184, 254, 255, 255));")
        self.control_pannel.setObjectName("control_pannel")

    def retranslate_shell(self, MainWindow, _translate):
        """Apply display strings for this panel's widgets."""
        MainWindow.setWindowTitle(_translate("MainWindow", "DEMO"))
        MainWindow.setWindowIcon(QIcon(asset("icon/ICSLAB.ico")))
        self.control_pannel.setTitle(_translate("MainWindow", "Control Pannel"))
        self.A_tab.setTabText(self.A_tab.indexOf(self.tab2), _translate("MainWindow", "DEMO"))
