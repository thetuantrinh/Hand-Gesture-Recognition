"""Wire protocol of the TI DCA1000EVM capture card.

Encoding and decoding are kept free of socket I/O so the framing can be
verified without the hardware present.
"""

import struct
from enum import Enum
from typing import Union

import numpy as np

#: Every configuration frame is delimited by these markers.
FRAME_HEADER = "5aa5"
FRAME_FOOTER = "aaee"

#: Largest datagram the card emits; also the receive buffer size.
MAX_PACKET_SIZE = 4096

#: Payload bytes per raw-ADC datagram, excluding the sequence header.
BYTES_IN_PACKET = 1456

#: LVDS lanes wired on the AWR1243BOOST evaluation board.
NUM_LANES = 4

#: Byte offsets within a raw-ADC datagram.
_SEQUENCE_NUMBER = slice(0, 4)
_ADC_PAYLOAD = slice(10, None)


class Command(str, Enum):
    """FPGA command opcodes, as little-endian hex digit pairs."""

    RESET_FPGA = "0100"
    RESET_AR_DEVICE = "0200"
    CONFIG_FPGA_GEN = "0300"
    CONFIG_EEPROM = "0400"
    RECORD_START = "0500"
    RECORD_STOP = "0600"
    PLAYBACK_START = "0700"
    PLAYBACK_STOP = "0800"
    SYSTEM_CONNECT = "0900"
    SYSTEM_ERROR = "0a00"
    CONFIG_PACKET_DATA = "0b00"
    CONFIG_DATA_MODE_AR_DEVICE = "0c00"
    INIT_FPGA_PLAYBACK = "0d00"
    READ_FPGA_VERSION = "0e00"

    def __str__(self) -> str:
        return str(self.value)


#: Arguments of ``CONFIG_FPGA_GEN``: raw mode, 16-bit, 4-lane LVDS, no decimation.
CONFIG_FPGA_GEN_ARGS = ("0600", "01010102031E")

#: Arguments of ``CONFIG_PACKET_DATA``: 1472-byte packets, 25 us inter-packet delay.
CONFIG_PACKET_DATA_ARGS = ("0600", "C005350C0000")


def encode_command(
    command: Union[Command, str],
    length: str = "0000",
    body: str = "",
) -> bytes:
    """Build a configuration frame for the card.

    Parameters
    ----------
    command:
        Opcode to send.
    length:
        Payload length, as a hex digit pair sequence.
    body:
        Command arguments, as a hex digit pair sequence.

    Returns
    -------
    bytes
        The framed command, ready to send.
    """
    return bytes.fromhex(f"{FRAME_HEADER}{command}{length}{body}{FRAME_FOOTER}")


def decode_data_packet(datagram: bytes) -> tuple[int, np.ndarray]:
    """Split a raw-ADC datagram into its sequence number and samples.

    Parameters
    ----------
    datagram:
        One UDP payload received on the data port.

    Returns
    -------
    tuple of (int, np.ndarray)
        The card's monotonically increasing packet counter, and the int16 ADC
        samples it carries.
    """
    sequence_number = struct.unpack("<1l", datagram[_SEQUENCE_NUMBER])[0]
    samples = np.frombuffer(datagram[_ADC_PAYLOAD], dtype=np.int16)
    return sequence_number, samples
