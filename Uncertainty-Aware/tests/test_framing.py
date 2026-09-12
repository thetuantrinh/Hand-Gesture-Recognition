"""Tests for packet-to-frame reassembly."""

import numpy as np
import pytest

from radar_hgr.dsp.framing import FrameAssembler


def test_rejects_a_non_positive_frame_size():
    with pytest.raises(ValueError):
        FrameAssembler(samples_per_frame=0)


def test_yields_nothing_until_a_frame_is_complete():
    assembler = FrameAssembler(samples_per_frame=4)
    assert list(assembler.push(np.arange(3, dtype=np.int16))) == []
    assert assembler.pending == 3


def test_yields_a_frame_once_enough_samples_arrive():
    assembler = FrameAssembler(samples_per_frame=4)
    assembler.push(np.arange(3, dtype=np.int16))

    frames = list(assembler.push(np.array([3, 4], dtype=np.int16)))

    assert len(frames) == 1
    assert frames[0].tolist() == [0, 1, 2, 3]
    assert assembler.pending == 1


def test_yields_several_frames_from_one_oversized_packet():
    assembler = FrameAssembler(samples_per_frame=2)
    frames = list(assembler.push(np.arange(7, dtype=np.int16)))

    assert [f.tolist() for f in frames] == [[0, 1], [2, 3], [4, 5]]
    assert assembler.pending == 1


def test_ignores_empty_and_missing_packets():
    assembler = FrameAssembler(samples_per_frame=2)
    assert list(assembler.push(np.empty(0, dtype=np.int16))) == []
    assert list(assembler.push(None)) == []
    assert assembler.pending == 0


def test_reset_discards_a_partial_frame():
    assembler = FrameAssembler(samples_per_frame=4)
    assembler.push(np.arange(3, dtype=np.int16))
    assembler.reset()
    assert assembler.pending == 0
