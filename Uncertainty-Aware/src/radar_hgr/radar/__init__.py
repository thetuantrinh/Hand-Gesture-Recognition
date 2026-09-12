"""Radar hardware interface.

:mod:`protocol` holds the pure encoding and decoding of the DCA1000EVM wire
format; :mod:`dca1000` adds the sockets.
"""

from .dca1000 import DCA1000Client
from .protocol import Command, decode_data_packet, encode_command

__all__ = ["Command", "DCA1000Client", "decode_data_packet", "encode_command"]
