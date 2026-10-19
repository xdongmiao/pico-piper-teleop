import json
from dataclasses import dataclass, field
from pathlib import Path

from lerobot.robots.config import RobotConfig


DEFAULT_CAMERA_PATHS = {
    "top": "/dev/video0",
    "left_wrist": "/dev/video2",
    "right_wrist": "/dev/video4",
}

DEFAULT_CAMERA_CONFIG = {
    "top": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video0",
    },
    "left_wrist": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6.4.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video2",
    },
    "right_wrist": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:11.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video4",
    },
}

CAMERA_CONFIG_PATH = Path("/projects/pico_piper_streamlit_app/camera_config.json")


def default_camera_paths() -> dict[str, str]:
    config = {name: dict(values) for name, values in DEFAULT_CAMERA_CONFIG.items()}
    if CAMERA_CONFIG_PATH.exists():
        try:
            saved = json.loads(CAMERA_CONFIG_PATH.read_text())
            for name, values in saved.items():
                if name in config and isinstance(values, dict):
                    config[name].update({str(k): str(v) for k, v in values.items()})
        except Exception:
            pass
    return {name: _resolve_camera_path(values) for name, values in config.items()}


def _resolve_camera_path(values: dict[str, str]) -> str:
    if values.get("mode") == "manual":
        return values.get("fallback_path", "")
    for key in ["stable_path", "by_id", "fallback_path"]:
        path = values.get(key, "")
        if path and Path(path).exists():
            return path
    return values.get("fallback_path", "")


@RobotConfig.register_subclass("pico_piper")
@dataclass(kw_only=True)
class PicoPiperConfig(RobotConfig):
    bridge_host: str = "127.0.0.1"
    bridge_port: int = 8765
    bridge_timeout_s: float = 0.2
    require_bridge: bool = True

    camera_paths: dict[str, str] = field(default_factory=default_camera_paths)
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    camera_fourcc: str = "MJPG"
    camera_ring_size: int = 120

    fps: int = 30
    max_camera_age_ms: float = 80.0
    max_ros_age_ms: float = 100.0
    max_vr_age_ms: float = 500.0
    max_action_age_ms: float = 500.0
    sleep_until_frame_time: bool = True
    no_op_send_action: bool = True
