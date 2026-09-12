"""Main Graphical User Interface & Control Pipeline.

Orchestrates real-time FMCW radar acquisition (TI DCA1000EVM),
Doppler-range DSP, uncertainty-aware deep learning inference,
and safety-gated Universal Robots (UR3) industrial manipulator control.
"""

import os
import sys
import time
import threading
from typing import Optional, List
import numpy as np

from PyQt5 import QtWidgets
from PyQt5.QtCore import QThreadPool, QDateTime
from PyQt5.QtWidgets import QFileDialog

# Ensure local src directory is on import path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from src.config import (
    RadarConfig,
    RobotConfig,
    GESTURE_LABELS,
    DEFAULT_MODEL_PATH,
)
from src.DSP import RadarDSP
from src.thread_fn import (
    WorkerSignals,
    PredictionWorker,
    RadarWorker,
    RobotWorker,
)
from src.UI import _UI_
from src.use_case import AutoController, ManualController, GesturePredictor
from src.utils import (
    update_tcp_display,
    update_joint_display,
    MicroDopplerPlotter,
)

# Suppress TensorFlow GPU allocation if running on CPU-only host
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")


class MainWindow(_UI_.Ui_MainWindow):
    """Primary PyQt5 window managing radar streaming, ML inference, and robot control."""

    def __init__(self, model_path: Optional[str] = None) -> None:
        super().__init__()
        self.mainWindow = QtWidgets.QMainWindow()
        self.setupUi(self.mainWindow)

        # Configurations
        self.radar_cfg = RadarConfig()
        self.robot_cfg = RobotConfig()

        # Core workers & communication
        self.signals = WorkerSignals()
        self.radar = RadarWorker()
        self.ur3 = RobotWorker()

        # Stdout/Stderr Redirection to GUI log window
        self.log_fn = self.radar.read_raw_data.log_fn
        self.log_fn.new_output.connect(self.append_output)
        sys.stdout = self.log_fn
        sys.stderr = self.log_fn

        # Connect UI Controls
        self._connect_signals()

        # State Variables
        self.UR3_TCP: List[float] = list(self.robot_cfg.default_tcp_pose)
        self.idx_ges: int = 9
        self.labels: List[str] = list(GESTURE_LABELS)

        # Thread Pool
        self.threadpool = QThreadPool.globalInstance()
        self.log_fn.write(
            f"Thread pool initialized with maximum {self.threadpool.maxThreadCount()} concurrent threads."
        )

        # Initialize Gesture Predictor
        resolved_model = model_path or DEFAULT_MODEL_PATH
        self.HRG_pred = GesturePredictor(resolved_model)
        self.path_model.setText(os.path.basename(self.HRG_pred.model_path or "None"))

        # Radar Buffer & DSP State
        self.line_buffer = np.array([], dtype=np.int16)
        self.num_samples_per_frame = self.radar_cfg.frame_size_samples
        self.micro_cube = np.ones((40, self.radar_cfg.num_rx_antennas, 2, 40), dtype=np.float32)
        self.dsp = RadarDSP(Nc=self.radar_cfg.num_chirps, Ns=self.radar_cfg.num_adc_samples)

        # UI Expand State
        self.expanded: bool = False
        self.originalSize = self.mainWindow.size()

    def _connect_signals(self) -> None:
        """Bind UI buttons and toggles to corresponding slot methods."""
        self.btn_start_radar.clicked.connect(self.thread_read_radar)
        self.btn_stop_radar.clicked.connect(self.stop_close_radar)
        self.btn_start_UR3.clicked.connect(self.thread_read_ur3)
        self.btn_stop_UR3.clicked.connect(self.stop_ur3)
        self.btn_RESTART_RADAR.clicked.connect(self.reset_cfg_radar)
        self.btn_reset_UR3.clicked.connect(self.reset_cfg_ur3)
        self.btn_file.clicked.connect(self.browse_file)

        # Manual Jog Buttons
        self.UP.clicked.connect(self.jog_up)
        self.LEFT.clicked.connect(self.jog_left)
        self.RIGHT.clicked.connect(self.jog_right)
        self.IN.clicked.connect(self.jog_in)
        self.OUT.clicked.connect(self.jog_out)

        # Gripper Toggles
        self.option_clamp.toggled.connect(self.jog_clamp)
        self.option_release.toggled.connect(self.jog_release)

        # Window expander button
        self.btn_a.clicked.connect(self.toggle_expand)

    def browse_file(self) -> None:
        """Open file dialog to load an alternative pre-trained Keras model."""
        file_dialog = QFileDialog.getOpenFileName(
            self.mainWindow,
            "Select Keras Gesture Model",
            os.path.join(CURRENT_DIR, "models"),
            "Model Files (*.h5 *.keras);;All Files (*)",
        )
        if file_dialog[0]:
            selected_path = file_dialog[0]
            self.HRG_pred = GesturePredictor(selected_path)
            self.path_model.setText(os.path.basename(selected_path))
            print(f"Loaded Keras model from: {selected_path}")

    def toggle_expand(self) -> None:
        """Toggle extended GUI layout view."""
        if self.expanded:
            self.mainWindow.resize(self.originalSize)
            self.expanded = False
        else:
            self.mainWindow.resize(int(self.mainWindow.width() * 1.4), self.mainWindow.height())
            self.expanded = True

    def connect_rs_cfg(self) -> None:
        """Initialize and configure DCA1000EVM capture card over Ethernet."""
        self.radar.read_raw_data.bind()
        self.radar.read_raw_data.configure_fpga()

    def stop_close_radar(self) -> None:
        """Halt radar acquisition and close network sockets."""
        self.radar.stop()
        self.radar.read_raw_data.stop_record()
        time.sleep(0.5)
        self.radar.read_raw_data.close()
        self.btn_start_radar.setEnabled(True)
        self.btn_RESTART_RADAR.setEnabled(True)

    def read_radar(self, udp_packet: np.ndarray) -> None:
        """Process incoming raw ADC UDP packet from worker thread."""
        self.line_buffer = np.concatenate((self.line_buffer, udp_packet))

        if len(self.line_buffer) >= self.num_samples_per_frame:
            frame = self.line_buffer[:self.num_samples_per_frame]
            self.line_buffer = self.line_buffer[self.num_samples_per_frame:]

            # DSP 2D Range-Doppler FFT
            data = self.dsp.pre_processing(np.array([frame]), fft=True)
            fft_cube = np.expand_dims(data[:, :, 33, 44:84], axis=0)

            # Slide micro-Doppler temporal window
            self.micro_cube = np.concatenate((self.micro_cube, fft_cube), axis=0)[-40:, :, :, :]
            MicroDopplerPlotter(self.graph, self.micro_cube).show_micro_doppler()

            # Execute gesture inference if enabled
            if self.pred.isChecked():
                window_feature = self.micro_cube[20:, :, :, :].transpose(0, 3, 1, 2)
                worker = PredictionWorker(
                    target=self.HRG_pred.prediction,
                    args=(window_feature.reshape(1, 20, 40, 8),),
                )
                worker.start()
                transitions = worker.join()

                if transitions is not None and len(transitions) >= 2:
                    prev_g, curr_g = transitions[0], transitions[1]
                    if curr_g != prev_g:
                        self.idx_ges = int(curr_g)
                        text = self.labels[self.idx_ges] if self.idx_ges < len(self.labels) else "Unknown"
                        self.gesture.setText(text)

                        # Trigger robot closed-loop action if enabled
                        if self.ctrl_ur3.isChecked():
                            ctrl_thread = threading.Thread(
                                target=self.hgr_ur3_control,
                                args=(self.idx_ges,),
                            )
                            ctrl_thread.start()
                            ctrl_thread.join()

    def thread_process_ur3(self, ur3_telemetry: List[np.ndarray]) -> None:
        """Update GUI with live UR3 Cartesian TCP and joint angle coordinates."""
        tcp, joint = ur3_telemetry[0], ur3_telemetry[1]
        self.UR3_TCP = list(tcp)

        update_tcp_display(
            tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5],
            self.X, self.Y, self.Z, self.RX, self.RY, self.RZ,
        )
        update_joint_display(
            joint[0], joint[1], joint[2], joint[3], joint[4], joint[5],
            self.base, self.shoulder, self.elbow, self.wrist_4, self.wrist_5, self.wrist_6,
        )

    def hgr_ur3_control(self, gesture_signal: int) -> None:
        """Execute automated gesture-directed robot manipulation."""
        AutoController(self.ur3, self.UR3_TCP).thread_control(gesture_signal)

    # Manual Jog Handlers
    def jog_up(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).up_state()

    def jog_left(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).left_state()

    def jog_right(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).right_state()

    def jog_in(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).in_state()

    def jog_out(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).out_state()

    def jog_clamp(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).clamp_state()

    def jog_release(self) -> None:
        ManualController(self.ur3, self.UR3_TCP).release_state()

    # Legacy method aliases for button compatibility
    up = jog_up
    left = jog_left
    right = jog_right
    in_ = jog_in
    out = jog_out
    clamp = jog_clamp
    release = jog_release
    browseFile = browse_file
    toggleExpand = toggle_expand

    def stop_ur3(self) -> None:
        """Halt UR3 communication."""
        self.ur3.stop()
        self.ur3.control_robot.close()
        self.btn_start_UR3.setEnabled(True)
        self.btn_reset_UR3.setEnabled(True)

    def thread_read_radar(self) -> None:
        """Start asynchronous radar acquisition worker."""
        self.connect_rs_cfg()
        self.radar.read_raw_data.start_record()
        self.btn_start_radar.setEnabled(False)
        self.radar.signals.bin_data_signal.connect(self.read_radar)
        self.threadpool.start(self.radar)

    def thread_read_ur3(self) -> None:
        """Start asynchronous robot telemetry worker."""
        self.btn_start_UR3.setEnabled(False)
        self.ur3.signals.robot_signal.connect(self.thread_process_ur3)
        self.threadpool.start(self.ur3)

    def reset_cfg_radar(self) -> None:
        """Log radar network parameters."""
        self.btn_RESTART_RADAR.setEnabled(False)
        print(f"System IP: {self.sys_IP_ADD_0.text()}.{self.sys_IP_ADD_1.text()}.{self.sys_IP_ADD_2.text()}.{self.sys_IP_ADD_3.text()}")
        print(f"FPGA IP: {self.FPGA_IP_ADD_0.text()}.{self.FPGA_IP_ADD_1.text()}.{self.FPGA_IP_ADD_2.text()}.{self.FPGA_IP_ADD_3.text()}")
        print(f"Config Port: {self.CFG_PORT.value()} | Data Port: {self.REG_PORT.value()}")

    def reset_cfg_ur3(self) -> None:
        """Log UR3 network parameters."""
        self.btn_reset_UR3.setEnabled(False)
        print(f"UR3 IP: {self.UR3_IP_ADD_0.text()}.{self.UR3_IP_ADD_1.text()}.{self.UR3_IP_ADD_2.text()}.{self.UR3_IP_ADD_3.text()}")
        print(f"RTDE Port: {self.RTDE_PORT.value()}")

    def append_output(self, message: str) -> None:
        """Append log message with timestamp to GUI QTextEdit widget."""
        timestamp = QDateTime.currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        self.text_edit.append(f"[{timestamp}] {message}")
        self.text_edit.verticalScrollBar().setValue(
            self.text_edit.verticalScrollBar().maximum()
        )


def main() -> None:
    """Application entrypoint."""
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.mainWindow.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()