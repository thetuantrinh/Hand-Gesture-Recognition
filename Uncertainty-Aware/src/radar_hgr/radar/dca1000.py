"""Socket client for the TI DCA1000EVM data capture card."""

import logging
import socket
import time
from typing import Optional

import numpy as np

from ..config.network import RadarNetworkConfig
from .protocol import (
    CONFIG_FPGA_GEN_ARGS,
    CONFIG_PACKET_DATA_ARGS,
    MAX_PACKET_SIZE,
    Command,
    decode_data_packet,
    encode_command,
)

logger = logging.getLogger(__name__)

#: Seconds to wait for the card to acknowledge a configuration command.
COMMAND_TIMEOUT = 1.0

#: Seconds the FPGA needs to settle after the general configuration command.
FPGA_SETTLE_DELAY = 1.0


class DCA1000Client:
    """Configures the capture card and reads its raw ADC stream.

    Two UDP sockets are used, matching the card's design: a request/response
    socket for configuration, and a receive-only socket for the high-rate ADC
    stream.

    Parameters
    ----------
    config:
        Network endpoints. Defaults to :class:`RadarNetworkConfig`.

    Notes
    -----
    The instance is usable as a context manager, which guarantees the sockets
    are closed even if configuration fails:

    >>> with DCA1000Client() as card:  # doctest: +SKIP
    ...     card.configure()
    ...     card.start_recording()
    """

    def __init__(self, config: Optional[RadarNetworkConfig] = None) -> None:
        self.config = config or RadarNetworkConfig()
        self._config_socket = self._udp_socket()
        self._data_socket = self._udp_socket()
        self._bound = False

    @staticmethod
    def _udp_socket() -> socket.socket:
        return socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def __enter__(self) -> "DCA1000Client":
        self.bind()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def bind(self) -> None:
        """Bind both sockets to the local capture interface."""
        self._data_socket.bind(self.config.host_data_endpoint)
        self._config_socket.bind(self.config.host_config_endpoint)
        self._bound = True
        logger.info(
            "Bound radar sockets: config %s, data %s",
            self.config.host_config_endpoint,
            self.config.host_data_endpoint,
        )

    def send_command(
        self,
        command: Command,
        length: str = "0000",
        body: str = "",
        timeout: float = COMMAND_TIMEOUT,
    ) -> bytes:
        """Send a configuration command and return the card's response.

        A timeout is reported and returns empty bytes rather than raising: the
        capture card is frequently powered up after the application, and the
        operator is expected to retry from the GUI.
        """
        self._config_socket.settimeout(timeout)
        request = encode_command(command, length, body)
        logger.debug("FPGA request %s: %s", command.name, request.hex())

        try:
            self._config_socket.sendto(request, self.config.fpga_endpoint)
            response, sender = self._config_socket.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout:
            logger.warning("No FPGA response to %s within %.1fs", command.name, timeout)
            return b""
        except OSError:
            logger.exception("Failed to send %s to the capture card", command.name)
            return b""

        logger.debug("FPGA response from %s: %s", sender, response.hex())
        return response

    def read_packet(self) -> tuple[int, np.ndarray]:
        """Read one raw ADC datagram.

        Returns
        -------
        tuple of (int, np.ndarray)
            Sequence number and int16 samples. On a timeout or read error the
            sample array is empty, which the caller treats as "no data yet".
        """
        try:
            datagram, _ = self._data_socket.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout:
            logger.debug("Timed out waiting for an ADC datagram")
            return 0, np.empty(0, dtype=np.int16)
        except OSError:
            logger.exception("Failed to read from the ADC data socket")
            return 0, np.empty(0, dtype=np.int16)

        return decode_data_packet(datagram)

    def configure(self) -> None:
        """Run the card's initialisation sequence.

        Connects, resets the FPGA, selects raw 16-bit 4-lane LVDS capture, then
        configures the packet format.
        """
        self.send_command(Command.SYSTEM_CONNECT)
        self.send_command(Command.RESET_FPGA)

        self.send_command(Command.CONFIG_FPGA_GEN, *CONFIG_FPGA_GEN_ARGS)
        time.sleep(FPGA_SETTLE_DELAY)

        self.send_command(Command.CONFIG_PACKET_DATA, *CONFIG_PACKET_DATA_ARGS)
        logger.info("Capture card configured for raw ADC streaming")

    def start_recording(self) -> None:
        """Instruct the card to begin streaming ADC data."""
        logger.info("Starting radar capture")
        self.send_command(Command.RECORD_START)

    def stop_recording(self) -> bytes:
        """Instruct the card to stop streaming ADC data."""
        logger.info("Stopping radar capture")
        return self.send_command(Command.RECORD_STOP)

    def close(self) -> None:
        """Close both sockets. Safe to call more than once."""
        for name, sock in (("data", self._data_socket), ("config", self._config_socket)):
            try:
                sock.close()
            except OSError:
                logger.exception("Error closing the radar %s socket", name)
        self._bound = False
        logger.info("Radar sockets closed")
