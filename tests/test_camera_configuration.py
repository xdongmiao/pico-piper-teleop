"""Check local camera configuration, including the identity-mismatch fallback."""

import json

import camera_config


def test_local_configuration_retains_rig_settings(tmp_path, monkeypatch):
    config_path = tmp_path / "camera_config.json"
    video = tmp_path / "top_video"
    video.touch()
    config_path.write_text(json.dumps({"top": {
        "mode": "manual", "fallback_path": str(video), "serial": "my-local-camera",
    }}), encoding="utf-8")
    monkeypatch.setattr(camera_config, "CONFIG_PATH", config_path)
    bindings = camera_config.get_camera_bindings()
    assert bindings["top"].effective_path == str(video)
    assert bindings["top"].serial == "my-local-camera"
    assert bindings["top"].mode == "manual"
    assert set(bindings) == {"top", "left_wrist", "right_wrist"}


def test_auto_prefers_stable_path_when_identity_matches(tmp_path, monkeypatch):
    stable, identity, fallback = [tmp_path / name for name in ["by-path", "by-id", "fallback"]]
    for path in [stable, identity, fallback]:
        path.touch()
    monkeypatch.setattr(camera_config, "_same_device", lambda *_: True)
    assert camera_config._resolve_effective_path("auto", str(stable), str(identity), str(fallback)) == (str(stable), "by-path")


def test_auto_avoids_wrong_camera_at_reused_usb_port(tmp_path, monkeypatch):
    stable, identity, fallback = [tmp_path / name for name in ["by-path", "by-id", "fallback"]]
    for path in [stable, identity, fallback]:
        path.touch()
    monkeypatch.setattr(camera_config, "_same_device", lambda *_: False)
    assert camera_config._resolve_effective_path("auto", str(stable), str(identity), str(fallback)) == (str(identity), "by-id")


def test_missing_cameras_keep_configured_fallback(tmp_path):
    fallback = str(tmp_path / "video0")
    assert camera_config._resolve_effective_path("auto", "", "", fallback) == (fallback, "missing")
    assert camera_config._resolve_effective_path("manual", "", "", fallback) == (fallback, "manual")
