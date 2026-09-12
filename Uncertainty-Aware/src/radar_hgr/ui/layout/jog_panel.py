"""Manual Cartesian jog pad and gripper tool selector."""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QRadioButton, QVBoxLayout

from ..resources import asset


class JogPanel:
    """Manual Cartesian jog pad and gripper tool selector.

    Mixin contributing this panel's widgets to :class:`~radar_hgr.ui.layout.main_window.Ui_MainWindow`.
    """

    def build_jog_panel(self, MainWindow):
        """Instantiate and position this panel's widgets."""
        self.frame_5 = QtWidgets.QFrame(self.UR3_SYS_CFG)
        self.frame_5.setGeometry(QtCore.QRect(20, 170, 400, 220))
        self.frame_5.setStyleSheet("background-color: rgb(235, 250, 255);")
        self.frame_5.setObjectName("frame_5")

        self.label = QtWidgets.QLabel(self.frame_5)
        pixmap = QPixmap(asset("xyz.png"))
        resized_pixmap = pixmap.scaled(70, 110)
        self.label.setPixmap(resized_pixmap)
        self.label.setGeometry(0, 10, 70, 110)

        self.tools = QtWidgets.QGroupBox(self.frame_5)
        self.tools.setGeometry(QtCore.QRect(5, 125, 100, 90))
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        self.tools.setFont(font)
        self.tools.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.tools.setObjectName("tools")

        self.option_clamp = QRadioButton("Clamp", self.tools)
        self.option_release = QRadioButton("Release", self.tools)
        vbox = QVBoxLayout()
        vbox.addWidget(self.option_clamp)
        vbox.addWidget(self.option_release)
        self.tools.setLayout(vbox)

        self.UP = QtWidgets.QPushButton(self.frame_5)
        self.UP.setGeometry(QtCore.QRect(140, 10, 41, 91))
        self.UP.setIcon(QIcon(asset("icon/up.png")))
        self.UP.setIconSize(QtCore.QSize(41, 91))
        self.UP.setObjectName("UP")

        self.DOWN = QtWidgets.QPushButton(self.frame_5)
        self.DOWN.setGeometry(QtCore.QRect(330, 10, 41, 91))
        self.DOWN.setIcon(QIcon(asset("icon/down.png")))
        self.DOWN.setIconSize(QtCore.QSize(40, 90))
        self.DOWN.setObjectName("DOWN")

        self.IN = QtWidgets.QPushButton(self.frame_5)
        self.IN.setGeometry(QtCore.QRect(210, 60, 91, 51))
        self.IN.setIcon(QIcon(asset("icon/fw.png")))
        self.IN.setIconSize(QtCore.QSize(90, 50))
        self.IN.setObjectName("IN")

        self.LEFT = QtWidgets.QPushButton(self.frame_5)
        self.LEFT.setGeometry(QtCore.QRect(120, 110, 91, 41))
        self.LEFT.setIcon(QIcon(asset("icon/left.png")))
        self.LEFT.setIconSize(QtCore.QSize(90, 40))
        self.LEFT.setObjectName("LEFT")

        self.OUT = QtWidgets.QPushButton(self.frame_5)
        self.OUT.setGeometry(QtCore.QRect(210, 150, 91, 61))
        self.OUT.setIcon(QIcon(asset("icon/bw.png")))
        self.OUT.setIconSize(QtCore.QSize(91, 61))
        self.OUT.setObjectName("OUT")

        self.RIGHT = QtWidgets.QPushButton(self.frame_5)
        self.RIGHT.setGeometry(QtCore.QRect(300, 110, 91, 41))
        self.RIGHT.setIcon(QIcon(asset("icon/right.png")))
        self.RIGHT.setIconSize(QtCore.QSize(90, 40))
        self.RIGHT.setObjectName("RIGHT")

    def retranslate_jog_panel(self, MainWindow, _translate):
        """Apply display strings for this panel's widgets."""
        self.tools.setTitle(_translate("MainWindow", "Tools"))
