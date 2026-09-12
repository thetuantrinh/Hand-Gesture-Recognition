"""Log console, model browser and secondary window controls."""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QScrollArea, QTextEdit, QVBoxLayout, QWidget

from ..resources import asset


class ToolbarPanel:
    """Log console, model browser and secondary window controls.

    Mixin contributing this panel's widgets to :class:`~radar_hgr.ui.layout.main_window.Ui_MainWindow`.
    """

    def build_toolbar_panel(self, MainWindow):
        """Instantiate and position this panel's widgets."""
        self.label_chirp = QtWidgets.QLabel(self.tab2)
        self.label_chirp.setGeometry(QtCore.QRect(1070, 100, 51, 31))
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        font.setBold(False)
        self.label_chirp.setFont(font)
        self.label_chirp.setObjectName("label_chirp")

        self.scroll_area = QScrollArea(self.tab2)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(1)
        self.output_widget = QWidget()
        self.layout = QVBoxLayout()
        self.output_widget.setLayout(self.layout)
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.layout.addWidget(self.text_edit)
        self.scroll_area.setWidget(self.output_widget)
        self.scroll_area.setGeometry(QtCore.QRect(1395, 20, 480, 960))
        self.scroll_area.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.scroll_area.setObjectName("logs")

        self.btn_a = QtWidgets.QPushButton(self.tab2)
        self.btn_a.setGeometry(QtCore.QRect(520, 780, 101, 101))
        self.btn_a.setIcon(QIcon(asset("logs.jpg")))
        self.btn_a.setIconSize(QtCore.QSize(100, 100))
        self.btn_a.setObjectName("btn_a")

        self.btn_file = QtWidgets.QPushButton(self.tab2)
        self.btn_file.setGeometry(QtCore.QRect(650, 780, 50, 50))
        self.btn_file.setIcon(QIcon(asset("file_brower.png")))
        self.btn_file.setIconSize(QtCore.QSize(51, 51))
        self.btn_file.setObjectName("btn_file")

        self.path_model = QtWidgets.QLabel(self.tab2)
        self.path_model.setGeometry(QtCore.QRect(700, 780, 510, 100))
        self.path_model.setWordWrap(True)
        font = QtGui.QFont()
        font.setFamily("System")
        font.setPointSize(16)
        font.setBold(False)
        self.path_model.setFont(font)
        self.path_model.setAlignment(QtCore.Qt.AlignCenter)
        self.path_model.setStyleSheet("border : 1px solid black")
        self.path_model.setObjectName("path_model")

        self.label_gesture = QtWidgets.QLabel(self.tab2)
        self.label_gesture.setGeometry(QtCore.QRect(280, 20, 231, 61))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(36)
        font.setBold(False)
        self.label_gesture.setFont(font)
        self.label_gesture.setObjectName("label_gesture")

    def retranslate_toolbar_panel(self, MainWindow, _translate):
        """Apply display strings for this panel's widgets."""
        self.path_model.setText(_translate("MainWindow", "MODEL PATH"))
        self.label_chirp.setText(_translate("MainWindow", "Chirp:"))
        self.label_gesture.setText(_translate("MainWindow", "Gesture:"))
