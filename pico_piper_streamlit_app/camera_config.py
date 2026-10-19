from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


CONFIG_PATH = Path("/projects/pico_piper_streamlit_app/camera_config.json")

CAMERA_LABELS = {
    "top": "Head / Top",
    "left_wrist": "Left Wrist",
    "right_wrist": "Right Wrist",
}

DEFAULT_CAMERA_CONFIG = {
    "top": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video0",
        "serial": "",
        "usb_port": "1.3",
    },
    "left_wrist": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6.4.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video2",
        "serial": "",
        "usb_port": "6.4.3",
    },
    "right_wrist": {
        "mode": "auto",
        "stable_path": "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:11.3:1.0-video-index0",
        "by_id": "",
        "fallback_path": "/dev/video4",
        "serial": "",
        "usb_port": "11.3",
    },
}


@dataclass(frozen=True)
class CameraBinding:
    name: str
    label: str
    stable_path: str
    by_id: str
    fallback_path: str
    effective_path: str
    serial: str
    usb_port: str
    mode: str


def load_camera_config() -> dict[str, dict[str, str]]:
    data = _deepcopy_default()
    if CONFIG_PATH.exists():
        try:
            saved = json.loads(CONFIG_PATH.read_text())
            for name, values in saved.items():
                if name in data and isinstance(values, dict):
                    data[name].update({str(k): str(v) for k, v in values.items()})
        except Exception:
            pass
    return data


def save_camera_config(config: dict[str, dict[str, str]]) -> None:
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")


def reset_camera_config() -> None:
    save_camera_config(_deepcopy_default())


def get_camera_bindings() -> dict[str, CameraBinding]:
    config = load_camera_config()
    return {name: _binding_from_config(name, values) for name, values in config.items()}


def get_camera_paths() -> dict[str, str]:
    return {name: binding.effective_path for name, binding in get_camera_bindings().items()}


def list_video_candidates() -> list[str]:
    candidates: list[str] = []
    for pattern in ["/dev/v4l/by-path/*video-index0", "/dev/v4l/by-id/*video-index0", "/dev/video*"]:
        for path in sorted(Path("/").glob(pattern.removeprefix("/"))):
            full = str(path)
            if full not in candidates:
                candidates.append(full)
    return candidates


def v4l_available() -> bool:
    return Path("/dev/v4l/by-path").exists() or Path("/dev/v4l/by-id").exists()


def _binding_from_config(name: str, values: dict[str, str]) -> CameraBinding:
    configured_mode = values.get("mode", "auto")
    stable_path = values.get("stable_path", "")
    by_id = values.get("by_id", "")
    fallback_path = values.get("fallback_path", "")
    serial = values.get("serial", "")
    effective_path, mode = _resolve_effective_path(configured_mode, stable_path, by_id, fallback_path, serial)
    return CameraBinding(
        name=name,
        label=CAMERA_LABELS.get(name, name),
        stable_path=stable_path,
        by_id=by_id,
        fallback_path=fallback_path,
        effective_path=effective_path,
        serial=serial,
        usb_port=values.get("usb_port", ""),
        mode=mode,
    )


def _resolve_effective_path(
    configured_mode: str,
    stable_path: str,
    by_id: str,
    fallback_path: str,
    serial: str = "",
) -> tuple[str, str]:
    if configured_mode == "manual":
        return fallback_path, "manual"
    if configured_mode == "by-id":
        if by_id and Path(by_id).exists():
            return by_id, "by-id"
        return fallback_path or stable_path or by_id, "missing"
    if configured_mode == "by-path":
        if stable_path and Path(stable_path).exists():
            return stable_path, "by-path"
        return fallback_path or by_id or stable_path, "missing"

    stable_matches_serial = True
    if stable_path and by_id and Path(stable_path).exists() and Path(by_id).exists():
        stable_matches_serial = _same_device(stable_path, by_id)
    for mode, path in [("by-path", stable_path), ("by-id", by_id), ("fallback", fallback_path)]:
        if mode == "by-path" and not stable_matches_serial:
            continue
        if path and Path(path).exists():
            return path, mode
    return fallback_path or stable_path or by_id, "missing"


def _deepcopy_default() -> dict[str, dict[str, str]]:
    return {name: dict(values) for name, values in DEFAULT_CAMERA_CONFIG.items()}


def _same_device(left: str, right: str) -> bool:
    try:
        return Path(left).resolve(strict=True) == Path(right).resolve(strict=True)
    except OSError:
        return False
