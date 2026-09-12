"""The main window: wiring between the widgets and the application services.

This class deliberately contains no signal processing, no inference and no
motion arithmetic. It connects widgets to sessions, and sessions' results to
views.
"""

import logging
import sys
from typing import Optional

import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThreadPool

from ..config.gestures import Gesture, label_for
from ..inference.predictor import GesturePredictor
from ..ui.layout import MainWindowLayout
from ..ui.views import LogConsole, MicroDopplerView, TelemetryView
from ..workers.robot_worker import RobotTelemetry
from ..workers.signals import OutputRedirector
from .radar_session import RadarSession
from .robot_session import RobotSession

logger = logging.getLogger(__name__)

#: Fraction by which the log pane widens the window when expanded.
EXPAND_FACTOR = 1.4


class MainWindow(MainWindowLayout):
    """Assembles the window's widgets, sessions and views.

    Parameters
    ----------
    model_path:
        Classifier checkpoint to load. When ``None``, the default is resolved
        from :func:`radar_hgr.config.paths.resolve_model_path`.
    """

    def __init__(self, model_path: Optional[str] = None) -> None:
        super().__init__()
        self.window = QtWidgets.QMainWindow()
        self.setupUi(self.window)

        self.thread_pool = QThreadPool.globalInstance()

        # Views
        self.log_console = LogConsole(self.text_edit)
        self.micro_doppler = MicroDopplerView(self.graph)
        self.telemetry = TelemetryView(
            tcp_labels=(self.X, self.Y, self.Z, self.RX, self.RY, self.RZ),
            joint_labels=(
                self.base,
                self.shoulder,
                self.elbow,
                self.wrist_4,
                self.wrist_5,
                self.wrist_6,
            ),
        )

        self._install_output_redirection()

        # Services
        self.predictor = GesturePredictor(model_path)
        self.path_model.setText(self.predictor.model_name)

        self.radar = RadarSession(
            on_features=self._on_radar_features,
            thread_pool=self.thread_pool,
        )
        self.robot = RobotSession(
            on_telemetry=self._on_robot_telemetry,
            thread_pool=self.thread_pool,
        )

        self._expanded = False
        self._default_size = self.window.size()

        self._connect_widgets()
        self.log_console.append(
            f"Thread pool ready with up to {self.thread_pool.maxThreadCount()} threads."
        )

    # ------------------------------------------------------------------ setup

    def _install_output_redirection(self) -> None:
        """Route console output into the GUI log pane.

        The vendored UR SDK and TensorFlow write directly to the standard
        streams, so they are captured rather than lost behind the GUI.
        """
        self._redirector = OutputRedirector()
        self._redirector.text_written.connect(self.log_console.append)
        sys.stdout = self._redirector
        sys.stderr = self._redirector

    def _connect_widgets(self) -> None:
        """Bind every widget to its handler."""
        self.btn_start_radar.clicked.connect(self.start_radar)
        self.btn_stop_radar.clicked.connect(self.stop_radar)
        self.btn_RESTART_RADAR.clicked.connect(self.report_radar_config)

        self.btn_start_UR3.clicked.connect(self.start_robot)
        self.btn_stop_UR3.clicked.connect(self.stop_robot)
        self.btn_reset_UR3.clicked.connect(self.report_robot_config)

        self.btn_file.clicked.connect(self.browse_model)
        self.btn_a.clicked.connect(self.toggle_log_pane)

        jog_buttons = {
            self.UP: "up",
            self.DOWN: "down",
            self.LEFT: "left",
            self.RIGHT: "right",
            self.IN: "inward",
            self.OUT: "outward",
        }
        for button, command in jog_buttons.items():
            button.clicked.connect(lambda _=False, name=command: self.jog(name))

        self.option_clamp.toggled.connect(
            lambda checked: self.jog("clamp") if checked else None
        )
        self.option_release.toggled.connect(
            lambda checked: self.jog("release") if checked else None
        )

    def show(self) -> None:
        """Display the window."""
        self.window.show()

    # ------------------------------------------------------------------ radar

    def start_radar(self) -> None:
        """Begin radar acquisition."""
        try:
            self.radar.start()
        except OSError as error:
            self.log_console.append(f"Could not start the radar: {error}")
            logger.exception("Radar start failed")
            return

        self.btn_start_radar.setEnabled(False)
        self.log_console.append("Radar acquisition started.")

    def stop_radar(self) -> None:
        """Stop radar acquisition and release the capture card."""
        self.radar.stop()
        self.btn_start_radar.setEnabled(True)
        self.btn_RESTART_RADAR.setEnabled(True)
        self.log_console.append("Radar acquisition stopped.")

    def report_radar_config(self) -> None:
        """Log the radar network settings currently entered in the GUI."""
        self.btn_RESTART_RADAR.setEnabled(False)
        self.log_console.append(f"System IP: {self._ip_from('sys_IP_ADD')}")
        self.log_console.append(f"FPGA IP: {self._ip_from('FPGA_IP_ADD')}")
        self.log_console.append(
            f"Config port: {self.CFG_PORT.value()} | data port: {self.REG_PORT.value()}"
        )

    def _on_radar_features(self, history: np.ndarray) -> None:
        """Render a new micro-Doppler frame and, if enabled, classify it."""
        self.micro_doppler.update(history)

        if not self.pred.isChecked():
            return

        # Inference runs on the GUI thread. It is fast enough to keep up with
        # the 20 fps frame rate, and keeping it synchronous means predictions
        # cannot be reordered relative to the frames that produced them.
        transition = self.predictor.predict(self.radar.pipeline.inference_window())
        if transition is None or not transition.changed:
            return

        self.gesture.setText(label_for(int(transition.current)))
        if self.ctrl_ur3.isChecked():
            self._act_on_gesture(transition.current)

    # ------------------------------------------------------------------ robot

    def start_robot(self) -> None:
        """Connect to the UR3 and begin polling its telemetry."""
        try:
            self.robot.start()
        except Exception as error:  # noqa: BLE001 - surfaced to the operator
            self.log_console.append(f"Could not connect to the UR3: {error}")
            logger.exception("Robot start failed")
            return

        self.btn_start_UR3.setEnabled(False)
        self.log_console.append("UR3 connected.")

    def stop_robot(self) -> None:
        """Disconnect from the UR3."""
        self.robot.stop()
        self.btn_start_UR3.setEnabled(True)
        self.btn_reset_UR3.setEnabled(True)
        self.log_console.append("UR3 disconnected.")

    def report_robot_config(self) -> None:
        """Log the robot network settings currently entered in the GUI."""
        self.btn_reset_UR3.setEnabled(False)
        self.log_console.append(f"UR3 IP: {self._ip_from('UR3_IP_ADD')}")
        self.log_console.append(f"RTDE port: {self.RTDE_PORT.value()}")

    def _on_robot_telemetry(self, telemetry: RobotTelemetry) -> None:
        """Show a telemetry reading in the pose and joint readouts."""
        self.telemetry.update_tcp_pose(telemetry.tcp_pose)
        self.telemetry.update_joint_positions(telemetry.joint_positions)

    def _act_on_gesture(self, gesture: Gesture) -> None:
        """Execute the robot action a gesture commands."""
        controller = self.robot.gesture_controller()
        if controller is None:
            self.log_console.append("Gesture control requested, but the UR3 is not connected.")
            return
        controller.execute(gesture)

    def jog(self, command: str) -> None:
        """Run one manual jog or gripper command.

        Parameters
        ----------
        command:
            Method name on :class:`~radar_hgr.robot.controllers.JogController`.
        """
        controller = self.robot.jog_controller()
        if controller is None:
            self.log_console.append("Jog requested, but the UR3 is not connected.")
            return
        getattr(controller, command)()

    # ------------------------------------------------------------------- misc

    def browse_model(self) -> None:
        """Let the operator pick a different classifier checkpoint."""
        from ..config.paths import models_dir

        selected, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.window,
            "Select a gesture model",
            str(models_dir()),
            "Keras models (*.h5 *.keras);;All files (*)",
        )
        if not selected:
            return

        self.predictor = GesturePredictor(selected)
        self.path_model.setText(self.predictor.model_name)
        self.log_console.append(f"Loaded model: {selected}")

    def toggle_log_pane(self) -> None:
        """Widen or restore the window to reveal the log pane."""
        if self._expanded:
            self.window.resize(self._default_size)
        else:
            self.window.resize(
                int(self.window.width() * EXPAND_FACTOR), self.window.height()
            )
        self._expanded = not self._expanded

    def _ip_from(self, prefix: str) -> str:
        """Join the four octet fields of an address group into dotted form."""
        octets = (getattr(self, f"{prefix}_{index}").text() for index in range(4))
        return ".".join(octets)
