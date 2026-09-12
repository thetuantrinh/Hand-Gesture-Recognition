"""Live monitoring header: laboratory logo, view selectors and gesture readout."""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap

from ..resources import asset


class MonitorPanel:
    """Live monitoring header: laboratory logo, view selectors and gesture readout.

    Mixin contributing this panel's widgets to :class:`~radar_hgr.ui.layout.main_window.Ui_MainWindow`.
    """

    def build_monitor_panel(self, MainWindow):
        """Instantiate and position this panel's widgets."""
        self.label = QtWidgets.QLabel(self.tab2)
        pixmap = QPixmap(asset("ICSLAB.png"))
        # pixmap = QPixmap("_internal//ICSLAB.png")
        resized_pixmap = pixmap.scaled(261, 91)
        self.label.setPixmap(resized_pixmap)
        self.label.setGeometry(10, 0, 261, 91)

        self.map = QtWidgets.QComboBox(self.tab2)
        self.map.setGeometry(QtCore.QRect(520, 100, 371, 31))
        font = QtGui.QFont()
        font.setFamily("System")
        self.map.setFont(font)
        self.map.setObjectName("map")
        self.map.addItem("")
        self.map.addItem("")
        self.map.addItem("")
        self.map.addItem("")
        self.map.addItem("")
        self.channel = QtWidgets.QComboBox(self.tab2)
        self.channel.setGeometry(QtCore.QRect(890, 100, 131, 31))
        font = QtGui.QFont()
        font.setFamily("System")
        self.channel.setFont(font)
        self.channel.setObjectName("channel")
        self.channel.addItem("")
        self.channel.addItem("")
        self.channel.addItem("")
        self.channel.addItem("")
        self.channel.addItem("")

        self.chirp = QtWidgets.QSpinBox(self.tab2)
        self.chirp.setGeometry(QtCore.QRect(1120, 100, 88, 31))
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(17)
        font.setBold(False)
        self.chirp.setFont(font)
        self.chirp.setMinimum(1)
        self.chirp.setMaximum(128)
        self.chirp.setObjectName("chirp")

        self.gesture = QtWidgets.QLabel(self.tab2)
        self.gesture.setGeometry(QtCore.QRect(520, 20, 871, 61))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(38)
        font.setBold(False)
        self.gesture.setStyleSheet("border : 2px solid blue")
        self.gesture.setAlignment(QtCore.Qt.AlignCenter)
        self.gesture.setFont(font)
        self.gesture.setObjectName("gesture")

    def retranslate_monitor_panel(self, MainWindow, _translate):
        """Apply display strings for this panel's widgets."""
        self.map.setItemText(0, _translate("MainWindow", "Map"))
        self.map.setItemText(1, _translate("MainWindow", "Raw data"))
        self.map.setItemText(2, _translate("MainWindow", "Range - FFT"))
        self.map.setItemText(3, _translate("MainWindow", "Range - Doppler Map"))
        self.map.setItemText(4, _translate("MainWindow", "micro - Doppler Map"))
        self.channel.setItemText(0, _translate("MainWindow", "Channel"))
        self.channel.setItemText(1, _translate("MainWindow", "RX1"))
        self.channel.setItemText(2, _translate("MainWindow", "RX2"))
        self.channel.setItemText(3, _translate("MainWindow", "RX3"))
        self.channel.setItemText(4, _translate("MainWindow", "RX4"))
        self.gesture.setText(_translate("MainWindow", "Unknown"))
