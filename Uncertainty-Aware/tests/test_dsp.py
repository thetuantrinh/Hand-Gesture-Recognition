"""Tests for the range-Doppler processing chain."""

import numpy as np
import pytest

from radar_hgr.config.radar import RadarConfig
from radar_hgr.dsp.filters import remove_static_clutter
from radar_hgr.dsp.pipeline import RangeDopplerProcessor
from radar_hgr.dsp.transforms import split_iq_channels
from radar_hgr.dsp.windows import doppler_window, range_window


@pytest.fixture
def config():
    """Return a small configuration, so the transforms stay fast to exercise."""
    return RadarConfig(num_chirps=8, num_adc_samples=4)


def test_samples_per_frame_counts_every_lane():
    cfg = RadarConfig()
    assert cfg.samples_per_frame == 128 * 64 * 4 * 2


def test_windows_are_cached_and_correctly_sized():
    assert range_window(16).shape == (16,)
    assert doppler_window(32).dtype == np.float32
    assert range_window(16) is range_window(16)


def test_static_clutter_removal_cancels_a_constant_return():
    # A reflector identical in every chirp is exactly what MTI must remove.
    static = np.ones((4, 8, 4), dtype=complex)
    assert np.allclose(remove_static_clutter(static, chirp_axis=1), 0.0)


def test_static_clutter_removal_preserves_a_varying_return():
    profile = np.zeros((2, 4, 3), dtype=complex)
    profile[:, 1, :] = 1.0
    filtered = remove_static_clutter(profile, chirp_axis=1)
    assert not np.allclose(filtered, 0.0)
    # Removing the mean leaves a zero-mean signal along the chirp axis.
    assert np.allclose(filtered.mean(axis=1), 0.0)


def test_split_iq_channels_inserts_an_axis_of_two():
    spectrum = np.array([[1 + 2j, 3 + 4j]])
    channels = split_iq_channels(spectrum)
    assert channels.shape == (1, 2, 2)
    assert channels.dtype == np.float32
    assert np.allclose(channels[0, 0], [1.0, 3.0])
    assert np.allclose(channels[0, 1], [2.0, 4.0])


def test_deinterleave_recovers_iq_lanes(config):
    processor = RangeDopplerProcessor(config)
    frame = np.arange(config.samples_per_frame, dtype=np.int16)

    adc = processor.deinterleave(frame)

    assert adc.shape == (config.num_rx_antennas, config.num_chirps, config.num_adc_samples)
    assert np.iscomplexobj(adc)


def test_deinterleave_matches_the_documented_lane_ordering(config):
    """The first four lanes carry I, the last four Q, column-major within a frame."""
    processor = RangeDopplerProcessor(config)
    num_lanes = config.num_rx_antennas * config.iq_channels
    frame = np.arange(config.samples_per_frame, dtype=np.int16)

    adc = processor.deinterleave(frame)
    lanes = frame.reshape(num_lanes, config.samples_per_channel, order="F")

    expected_rx0 = lanes[0] + 1j * lanes[config.num_rx_antennas]
    assert np.allclose(adc[0].ravel(), expected_rx0)


def test_process_produces_the_network_input_layout(config):
    processor = RangeDopplerProcessor(config)
    frame = np.random.default_rng(0).integers(
        -2048, 2048, config.samples_per_frame, dtype=np.int16
    )

    tensor = processor.process(frame)

    assert tensor.shape == (
        config.num_rx_antennas,
        config.iq_channels,
        config.num_adc_samples,
        config.num_chirps,
    )
    assert tensor.dtype == np.float32
    assert np.isfinite(tensor).all()
