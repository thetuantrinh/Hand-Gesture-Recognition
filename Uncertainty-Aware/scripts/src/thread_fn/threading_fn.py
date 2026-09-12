"""Multi-threading Worker Classes and Signals for PyQt5.

Provides concurrent worker threads for non-blocking radar packet acquisition,
real-time UR3 robotic telemetry polling, and deep learning model inference.
"""

from threading import Thread
from typing import Any, Callable, Optional, Tuple
from PyQt5.QtCore import QObject, pyqtSignal, QRunnable, pyqtSlot

from ..radar import DCA1000EVM_backend
from ..UR import UR3_GESTURE as ur3


class OutputRedirector(QObject):
    """Redirects stdout and stderr streams to a PyQt signal for GUI text logs."""
    new_output: pyqtSignal = pyqtSignal(str)

    def write(self, message: Any) -> None:
        """Emit stringified message to listening GUI widgets."""
        text = str(message)
        if text.strip():
            self.new_output.emit(text)

    def flush(self) -> None:
        """Stream flush placeholder for sys.stdout compatibility."""
        pass


class WorkerSignals(QObject):
    """Signals available from background QRunnable worker threads."""
    bin_data_signal: pyqtSignal = pyqtSignal(object)
    robot_signal: pyqtSignal = pyqtSignal(object)


class PredictionWorker(Thread):
    """Dedicated background thread for TensorFlow / Keras model inference."""

    def __init__(
        self,
        group: None = None,
        target: Optional[Callable] = None,
        name: Optional[str] = None,
        args: Tuple = (),
        kwargs: Optional[dict] = None,
        verbose: None = None,
    ) -> None:
        super().__init__(group=group, target=target, name=name, args=args, kwargs=kwargs or {})
        self._target: Optional[Callable] = target
        self._args: Tuple = args
        self._kwargs: dict = kwargs or {}
        self._return: Any = None

    def run(self) -> None:
        """Execute target inference callable and store result."""
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)

    def join(self, timeout: Optional[float] = None) -> Any:
        """Wait for thread completion and return inference output."""
        super().join(timeout)
        return self._return


# Backwards compatibility alias
Thread_Prediction = PredictionWorker


class RadarWorker(QRunnable):
    """QRunnable worker continuously polling raw ADC packets from the DCA1000EVM."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.read_raw_data = DCA1000EVM_backend.DCA1000()
        self.signals = WorkerSignals()
        self._is_running: bool = True

    def stop(self) -> None:
        """Signal worker loop to terminate."""
        self._is_running = False

    @pyqtSlot()
    def run(self) -> None:
        """Continuous polling loop emitting raw ADC arrays to GUI slot."""
        while self._is_running:
            try:
                _, adc_data = self.read_raw_data.read_data_packet()
                if adc_data is not None and len(adc_data) > 0:
                    self.signals.bin_data_signal.emit(adc_data)
            except Exception as e:
                print(f"Error in Radar worker: {e}")
                break


# Backwards compatibility alias
Radar = RadarWorker


class RobotWorker(QRunnable):
    """QRunnable worker continuously polling joint and Cartesian poses from UR3."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.control_robot = ur3.GES_POS()
        self.signals = WorkerSignals()
        self._is_running: bool = True

    def stop(self) -> None:
        """Signal worker loop to terminate."""
        self._is_running = False

    @pyqtSlot()
    def run(self) -> None:
        """Continuous polling loop emitting UR3 telemetry to GUI slot."""
        while self._is_running:
            try:
                tcp_data = self.control_robot.read_ur_data(fps=20, read_data="TCP Pos")
                joint_data = self.control_robot.read_ur_data(fps=20, read_data="joint Pos")
                self.signals.robot_signal.emit([tcp_data, joint_data])
            except Exception as e:
                print(f"Error in Robot worker: {e}")
                break


# Backwards compatibility alias
UR3 = RobotWorker