"""Tests for micro-Doppler feature assembly."""

import numpy as np
import pytest

from radar_hgr.app.gesture_pipeline import MicroDopplerPipeline
from radar_hgr.config.pipeline import PipelineConfig
from radar_hgr.config.radar import RadarConfig


@pytest.fixture
def pipeline():
    """Return a pipeline over a small radar geometry, with in-range feature indices."""
    radar = RadarConfig(num_chirps=16, num_adc_samples=8)
    config = PipelineConfig(
        range_bin=3,
        doppler_bin_start=2,
        doppler_bin_count=4,
        history_frames=6,
        inference_frames=3,
    )
    return MicroDopplerPipeline(radar, config)


def frame_for(pipeline, seed=0):
    rng = np.random.default_rng(seed)
    return rng.integers(-2048, 2048, pipeline.radar_config.samples_per_frame, dtype=np.int16)


def test_doppler_slice_is_half_open():
    config = PipelineConfig(doppler_bin_start=44, doppler_bin_count=40)
    assert config.doppler_slice == slice(44, 84)


def test_input_shape_flattens_the_antenna_and_iq_axes():
    config = PipelineConfig(inference_frames=20, doppler_bin_count=40)
    assert config.input_shape(num_rx_antennas=4, iq_channels=2) == (1, 20, 40, 8)


def test_history_is_primed_so_the_first_frame_has_a_full_window(pipeline):
    assert pipeline.history.shape == (6, 4, 2, 4)
    assert np.allclose(pipeline.history, 1.0)


def test_pushing_a_frame_keeps_the_history_depth_fixed(pipeline):
    for seed in range(10):
        history = pipeline.push(frame_for(pipeline, seed))
        assert history.shape == pipeline.history_shape


def test_the_newest_frame_lands_at_the_end_of_the_history(pipeline):
    pipeline.push(frame_for(pipeline, seed=1))
    # The primed rows are all ones; a real frame is not.
    assert not np.allclose(pipeline.history[-1], 1.0)
    assert np.allclose(pipeline.history[0], 1.0)


def test_the_history_rolls_once_it_is_full(pipeline):
    for seed in range(pipeline.config.history_frames):
        pipeline.push(frame_for(pipeline, seed))
    oldest = pipeline.history[0].copy()

    pipeline.push(frame_for(pipeline, seed=99))

    assert not np.allclose(pipeline.history[0], oldest)


def test_inference_window_matches_the_configured_input_shape(pipeline):
    pipeline.push(frame_for(pipeline))
    window = pipeline.inference_window()

    assert window.shape == pipeline.config.input_shape(
        pipeline.radar_config.num_rx_antennas,
        pipeline.radar_config.iq_channels,
    )
    assert window.dtype == np.float32
    assert np.isfinite(window).all()


def test_inference_window_preserves_the_history_values(pipeline):
    """The reshape must only reorder axes, never resample."""
    pipeline.push(frame_for(pipeline))
    recent = pipeline.history[-pipeline.config.inference_frames :]

    window = pipeline.inference_window()

    assert sorted(window.ravel().tolist()) == pytest.approx(sorted(recent.ravel().tolist()))


def test_reset_clears_the_history(pipeline):
    pipeline.push(frame_for(pipeline))
    pipeline.reset()
    assert np.allclose(pipeline.history, 1.0)
