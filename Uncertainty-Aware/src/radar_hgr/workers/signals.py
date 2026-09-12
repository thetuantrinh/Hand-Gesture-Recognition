"""Qt signal carriers shared by the background workers.

Qt requires widgets to be touched only from the thread that created them, so
every worker publishes results through signals instead of writing to the GUI.
"""

from typing import Any

from PyQt5.QtCore import QObject, pyqtSignal


class RadarSignals(QObject):
    """Signals emitted by :class:`~radar_hgr.workers.radar_worker.RadarWorker`."""

    #: One packet of raw int16 ADC samples.
    samples_received = pyqtSignal(object)

    #: Human-readable reason the acquisition loop stopped.
    stopped = pyqtSignal(str)


class RobotSignals(QObject):
    """Signals emitted by :class:`~radar_hgr.workers.robot_worker.RobotWorker`."""

    #: One telemetry reading, as a ``RobotTelemetry``.
    telemetry_received = pyqtSignal(object)

    #: Human-readable reason the telemetry loop stopped.
    stopped = pyqtSignal(str)


class OutputRedirector(QObject):
    """A ``sys.stdout``/``sys.stderr`` stand-in that republishes writes as a signal.

    Installed so that output from the vendored UR SDK and from TensorFlow, which
    write to the console rather than through :mod:`logging`, still reach the
    operator in the GUI log console.
    """

    #: One non-empty line of captured output.
    text_written = pyqtSignal(str)

    def write(self, message: Any) -> None:
        """Publish ``message`` unless it is blank."""
        text = str(message)
        if text.strip():
            self.text_written.emit(text)

    def flush(self) -> None:
        """No-op, present for file-object compatibility."""
