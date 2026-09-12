"""Tests for configuration objects and model path resolution."""

from radar_hgr.config.gestures import GESTURE_LABELS, NUM_CLASSES, Gesture, label_for
from radar_hgr.config.network import RadarNetworkConfig
from radar_hgr.config.paths import DEFAULT_MODEL_ENV_VAR, MODELS_ENV_VAR, resolve_model_path
from radar_hgr.config.robot import WorkspaceLimits


def test_every_gesture_has_a_label():
    assert len(GESTURE_LABELS) == NUM_CLASSES
    assert all(gesture.label for gesture in Gesture)


def test_label_for_tolerates_an_out_of_range_index():
    assert label_for(NUM_CLASSES) == "Unknown"
    assert label_for(-1) == "Unknown"
    assert label_for(int(Gesture.MOVE_LEFT)) == "Move Left"


def test_network_endpoints_pair_address_with_port():
    cfg = RadarNetworkConfig()
    assert cfg.fpga_endpoint == (cfg.fpga_ip, cfg.fpga_port)
    assert cfg.host_data_endpoint == (cfg.host_ip, cfg.host_data_port)


def test_workspace_limits_are_inclusive():
    limits = WorkspaceLimits(z_min=-0.4, z_max=-0.3)
    assert limits.contains_z(-0.4)
    assert limits.contains_z(-0.3)
    assert not limits.contains_z(-0.29)


def test_resolve_model_path_prefers_an_existing_explicit_path(tmp_path):
    checkpoint = tmp_path / "custom.h5"
    checkpoint.write_bytes(b"")
    assert resolve_model_path(str(checkpoint)) == checkpoint.resolve()


def test_resolve_model_path_falls_back_to_the_environment(tmp_path, monkeypatch):
    checkpoint = tmp_path / "from_env.h5"
    checkpoint.write_bytes(b"")
    monkeypatch.setenv(DEFAULT_MODEL_ENV_VAR, str(checkpoint))

    assert resolve_model_path("/does/not/exist.h5") == checkpoint.resolve()


def test_resolve_model_path_searches_the_models_directory(tmp_path, monkeypatch):
    monkeypatch.delenv(DEFAULT_MODEL_ENV_VAR, raising=False)
    monkeypatch.setenv(MODELS_ENV_VAR, str(tmp_path))
    checkpoint = tmp_path / "model_2.h5"
    checkpoint.write_bytes(b"")

    assert resolve_model_path() == checkpoint.resolve()


def test_resolve_model_path_returns_none_when_nothing_is_available(tmp_path, monkeypatch):
    monkeypatch.delenv(DEFAULT_MODEL_ENV_VAR, raising=False)
    monkeypatch.setenv(MODELS_ENV_VAR, str(tmp_path / "empty"))
    assert resolve_model_path() is None
