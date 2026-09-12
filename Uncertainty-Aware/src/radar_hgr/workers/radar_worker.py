"""Background acquisition of the raw ADC stream."""

import logging

from PyQt5.QtCore import QRunnable, pyqtSlot

from ..radar.dca1000 import DCA1000Client
from .signals import RadarSignals

logger = logging.getLogger(__name__)


class RadarWorker(QRunnable):
    """Polls the capture card and republishes each packet as a Qt signal.

    Parameters
    ----------
    client:
        The capture card client to read from. The worker does not own its
        lifecycle: binding, configuring and closing stay with
        :class:`~radar_hgr.app.radar_session.RadarSession`, so a stopped worker
        can be restarted against the same sockets.
    """

    def __init__(self, client: DCA1000Client) -> None:
        super().__init__()
        self.client = client
        self.signals = RadarSignals()
        self._running = False

    def stop(self) -> None:
        """Ask the loop to finish after the packet in flight."""
        self._running = False

    @pyqtSlot()
    def run(self) -> None:
        """Read packets until stopped or until the socket fails."""
        self._running = True
        reason = "Radar acquisition stopped"

        while self._running:
            try:
                _, samples = self.client.read_packet()
            except Exception:
                logger.exception("Radar acquisition failed")
                reason = "Radar acquisition failed; see the log for details"
                break

            if len(samples):
                self.signals.samples_received.emit(samples)

        self.signals.stopped.emit(reason)
