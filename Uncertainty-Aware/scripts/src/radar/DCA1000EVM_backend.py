"""Texas Instruments DCA1000EVM Real-Time Ethernet Backend Client.

Provides socket-level UDP communication for FPGA configuration,
streaming control, and high-speed raw ADC data reception.
"""

import socket
import struct
import time
from enum import Enum
from typing import Optional, Tuple, Union, Any
import numpy as np


# Protocol Framing Constants
CONFIG_HEADER: str = "5aa5"
CONFIG_STATUS: str = "0000"
CONFIG_FOOTER: str = "aaee"

# Hardware Packet Constraints
MAX_PACKET_SIZE: int = 4096
BYTES_IN_PACKET: int = 1456
NUM_LANES: int = 4  # AWR1243BOOST standard LVDS lanes


class _DefaultLogger:
    """Fallback stdout logger when no GUI OutputRedirector is attached."""
    def write(self, message: Any) -> None:
        text = str(message).strip()
        if text:
            print(f"[Radar] {text}")


class CMD(str, Enum):
    """FPGA Command opcodes for DCA1000EVM board control."""
    RESET_FPGA_CMD_CODE = "0100"
    RESET_AR_DEV_CMD_CODE = "0200"
    CONFIG_FPGA_GEN_CMD_CODE = "0300"
    CONFIG_EEPROM_CMD_CODE = "0400"
    RECORD_START_CMD_CODE = "0500"
    RECORD_STOP_CMD_CODE = "0600"
    PLAYBACK_START_CMD_CODE = "0700"
    PLAYBACK_STOP_CMD_CODE = "0800"
    SYSTEM_CONNECT_CMD_CODE = "0900"
    SYSTEM_ERROR_CMD_CODE = "0a00"
    CONFIG_PACKET_DATA_CMD_CODE = "0b00"
    CONFIG_DATA_MODE_AR_DEV_CMD_CODE = "0c00"
    INIT_FPGA_PLAYBACK_CMD_CODE = "0d00"
    READ_FPGA_VERSION_CMD_CODE = "0e00"

    def __str__(self) -> str:
        return str(self.value)


class DCA1000Client:
    """UDP Socket interface to the DCA1000EVM data capture card.

    Parameters
    ----------
    fpga_ip : str, optional
        Target FPGA IP address, default "192.168.33.180".
    fpga_port : int, optional
        Target FPGA configuration UDP port, default 4096.
    host_ip : str, optional
        Local host network interface IP, default "192.168.33.30".
    host_cfg_port : int, optional
        Local UDP port bound for configuration messages, default 4096.
    host_data_port : int, optional
        Local UDP port bound for high-speed ADC packet reception, default 4098.
    log_fn : Optional[Any], optional
        Logger or OutputRedirector instance with a `.write(str)` method.
    """

    def __init__(
        self,
        fpga_ip: str = "192.168.33.180",
        fpga_port: int = 4096,
        host_ip: str = "192.168.33.30",
        host_cfg_port: int = 4096,
        host_data_port: int = 4098,
        log_fn: Optional[Any] = None,
    ) -> None:
        self.log_fn = log_fn or _DefaultLogger()

        # Network endpoints
        self.cfg_dest: Tuple[str, int] = (fpga_ip, fpga_port)
        self.cfg_recv: Tuple[str, int] = (host_ip, host_cfg_port)
        self.data_recv: Tuple[str, int] = (host_ip, host_data_port)

        # Sockets
        self.config_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def bind(self) -> None:
        """Bind configuration and data UDP sockets to host interfaces."""
        self.data_socket.bind(self.data_recv)
        self.config_socket.bind(self.cfg_recv)
        self.log_fn.write("Ethernet Radar UDP ports bound successfully.")

    # Legacy alias
    _bind_ = bind

    def send_command(
        self,
        cmd: Union[CMD, str],
        length: str = "0000",
        body: str = "",
        timeout: float = 1.0,
    ) -> bytes:
        """Construct and transmit a structured binary command frame to the FPGA.

        Parameters
        ----------
        cmd : CMD or str
            Command opcode.
        length : str, optional
            Payload length hex string, by default "0000".
        body : str, optional
            Command body hex string, by default "".
        timeout : float, optional
            Socket timeout in seconds, by default 1.0.

        Returns
        -------
        bytes
            Response received from FPGA, or empty bytes on timeout.
        """
        self.config_socket.settimeout(timeout)
        hex_string = f"{CONFIG_HEADER}{str(cmd)}{length}{body}{CONFIG_FOOTER}"
        msg = bytes.fromhex(hex_string)

        self.log_fn.write(f"Request: {msg}")
        resp = b""
        try:
            self.config_socket.sendto(msg, self.cfg_dest)
            resp, addr = self.config_socket.recvfrom(MAX_PACKET_SIZE)
            self.log_fn.write(f"Response: {resp} from {addr}")
        except socket.timeout:
            self.log_fn.write(f"Socket timeout waiting for FPGA response to {cmd}")
        except Exception as e:
            self.log_fn.write(f"Command error: {e}")

        return resp

    # Legacy alias
    _send_command = send_command

    def read_data_packet(self) -> Tuple[int, np.ndarray]:
        """Read a single raw ADC UDP packet from the DCA1000EVM.

        Returns
        -------
        Tuple[int, np.ndarray]
            (packet_index, 1D array of int16 ADC samples).
        """
        packet_num = 0
        packet_data = np.array([], dtype=np.int16)

        try:
            data, _ = self.data_socket.recvfrom(MAX_PACKET_SIZE)
            packet_num = struct.unpack("<1l", data[:4])[0]
            packet_data = np.frombuffer(data[10:], dtype=np.int16)
        except socket.timeout:
            self.log_fn.write("Timeout waiting for ADC data packet.")
        except Exception as e:
            self.log_fn.write(f"Packet read error: {e}")

        return packet_num, packet_data

    # Legacy alias
    _read_data_packet = read_data_packet

    def start_record(self) -> None:
        """Transmit RECORD_START command to begin data streaming."""
        self.log_fn.write("Sending FPGA record start command...\n")
        self.send_command(CMD.RECORD_START_CMD_CODE)

    def stop_record(self) -> bytes:
        """Transmit RECORD_STOP command to halt data streaming."""
        self.log_fn.write("Sending FPGA record stop command...\n")
        return self.send_command(CMD.RECORD_STOP_CMD_CODE)

    # Legacy alias
    _stop_record_ = stop_record

    def configure_fpga(self) -> None:
        """Execute full initialization sequence: connect, reset, and configure FPGA."""
        self.send_command(CMD.SYSTEM_CONNECT_CMD_CODE)
        self.log_fn.write("System connected to FPGA.\n")

        self.send_command(CMD.RESET_FPGA_CMD_CODE)
        self.log_fn.write("FPGA reset complete.\n")

        self.send_command(CMD.CONFIG_FPGA_GEN_CMD_CODE, "0600", "01010102031E")
        time.sleep(1)

        self.log_fn.write("Configuring packet data format...\n")
        self.send_command(CMD.CONFIG_PACKET_DATA_CMD_CODE, "0600", "C005350C0000")

    # Legacy alias
    _cfg_fpga_ = configure_fpga

    def close(self) -> None:
        """Safely close open UDP sockets."""
        try:
            self.data_socket.close()
            self.config_socket.close()
            self.log_fn.write("Ethernet radar ports closed cleanly.\n")
        except Exception as e:
            self.log_fn.write(f"Error closing sockets: {e}")


# Backwards compatibility alias
DCA1000 = DCA1000Client