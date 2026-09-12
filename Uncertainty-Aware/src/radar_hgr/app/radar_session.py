"""Lifecycle of one radar acquisition run."""

import logging
import time
from typing import Callable, Optional

import numpy as np
from PyQt5.QtCore import QThreadPool

from ..config.network import RadarNetworkConfig
from ..config.pipeline import PipelineConfig
from ..config.radar import RadarConfig
from ..dsp.framing import FrameAssembler
from ..radar.dca1000 import DCA1000Client
from ..workers.radar_worker import RadarWorker
from .gesture_pipeline import MicroDopplerPipeline

logger = logging.getLogger(__name__)

#: Seconds between the stop command and closing the sockets, so the card's
#: in-flight datagrams are drained rather than triggering a reset error.
DRAIN_DELAY = 0.5


class RadarSession:
    """Starts and stops radar acquisition, and turns packets into features.

    A session owns one capture-card client and one worker, and creates fresh
    ones on every :meth:`start`. Qt deletes a finished ``QRunnable`` and the
    card's sockets cannot be rebound once closed, so neither object can be
    reused across runs.

    Parameters
    ----------
    on_features:
        Called on the GUI thread for every completed frame, with the updated
        micro-Doppler history.
    radar_config, network_config, pipeline_config:
        Configuration objects; defaults are used when omitted.
    thread_pool:
        Pool to run the worker in. Defaults to the global instance.
    """

    def __init__(
        self,
        on_features: Callable[[np.ndarray], None],
        radar_config: Optional[RadarConfig] = None,
        network_config: Optional[RadarNetworkConfig] = None,
        pipeline_config: Optional[PipelineConfig] = None,
        thread_pool: Optional[QThreadPool] = None,
    ) -> None:
        self.on_features = on_features
        self.radar_config = radar_config or RadarConfig()
        self.network_config = network_config or RadarNetworkConfig()
        self.thread_pool = thread_pool or QThreadPool.globalInstance()

        self.pipeline = MicroDopplerPipeline(self.radar_config, pipeline_config)
        self._assembler = FrameAssembler(self.radar_config.samples_per_frame)

        self._client: Optional[DCA1000Client] = None
        self._worker: Optional[RadarWorker] = None

    @property
    def running(self) -> bool:
        """Whether acquisition is currently active."""
        return self._worker is not None

    def start(self) -> None:
        """Bind and configure the capture card, then begin acquisition.

        Raises
        ------
        OSError
            If the capture interface cannot be bound, e.g. because the address
            is not configured on this host or another process holds the port.
        """
        if self.running:
            logger.debug("Radar acquisition already running")
            return

        self._assembler.reset()
        self.pipeline.reset()

        client = DCA1000Client(self.network_config)
        try:
            client.bind()
            client.configure()
            client.start_recording()
        except OSError:
            client.close()
            raise

        worker = RadarWorker(client)
        worker.setAutoDelete(False)
        worker.signals.samples_received.connect(self._consume_packet)

        self._client, self._worker = client, worker
        self.thread_pool.start(worker)
        logger.info("Radar acquisition started")

    def stop(self) -> None:
        """Stop acquisition and release the capture card."""
        if self._worker is not None:
            self._worker.stop()
            self._worker = None

        if self._client is not None:
            self._client.stop_recording()
            time.sleep(DRAIN_DELAY)
            self._client.close()
            self._client = None

        logger.info("Radar acquisition stopped")

    def _consume_packet(self, samples: np.ndarray) -> None:
        """Buffer a packet and push every frame it completes through the DSP chain."""
        for frame in self._assembler.push(samples):
            self.on_features(self.pipeline.push(frame))
