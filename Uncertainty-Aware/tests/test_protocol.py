"""Tests for the DCA1000EVM wire protocol."""

import struct

import numpy as np

from radar_hgr.radar.protocol import (
    FRAME_FOOTER,
    FRAME_HEADER,
    Command,
    decode_data_packet,
    encode_command,
)


def test_encode_command_is_delimited_by_the_frame_markers():
    frame = encode_command(Command.RECORD_START)
    assert frame.hex().startswith(FRAME_HEADER)
    assert frame.hex().endswith(FRAME_FOOTER)


def test_encode_command_carries_the_opcode_and_body():
    frame = encode_command(Command.CONFIG_FPGA_GEN, "0600", "01010102031E")
    assert frame.hex() == f"{FRAME_HEADER}0300060001010102031e{FRAME_FOOTER}"


def test_encode_command_of_a_bodyless_command_is_eight_bytes():
    assert len(encode_command(Command.SYSTEM_CONNECT)) == 8


def test_command_stringifies_to_its_opcode():
    assert str(Command.RECORD_STOP) == "0600"


def test_decode_data_packet_splits_the_sequence_number_from_the_samples():
    samples = np.array([1, -2, 3, -4], dtype=np.int16)
    datagram = struct.pack("<1l", 42) + b"\x00" * 6 + samples.tobytes()

    sequence, decoded = decode_data_packet(datagram)

    assert sequence == 42
    assert decoded.tolist() == samples.tolist()
