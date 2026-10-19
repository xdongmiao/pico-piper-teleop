from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2

from camera_config import get_camera_bindings
from telemetry import BridgeClient


@dataclass
class CheckResult:
    name: str
    ok: bool
    details: str
    value: dict[str, object] = field(default_factory=dict)


@dataclass
class ScanResult:
    ok: bool
    checks: dict[str, CheckResult]
    scanned_at: float


def scan_all() -> ScanResult:
    checks: dict[str, CheckResult] = {}
    checks.update(scan_can())
    checks.update(scan_cameras())
    checks.update(scan_bridge_and_ros())
    required = [
        "can_left",
        "can_right",
        "camera_top",
        "camera_left_wrist",
        "camera_right_wrist",
        "bridge",
        "left_arm",
        "right_arm",
    ]
    ok = all(checks.get(key, CheckResult(key, False, "missing")).ok for key in required)
    return ScanResult(ok=ok, checks=checks, scanned_at=time.time())


def scan_can() -> dict[str, CheckResult]:
    result = _run("ip -details link show type can 2>/dev/null || true")
    checks = {}
    for iface in ["can_left", "can_right"]:
        block = _extract_iface_block(result.stdout, iface)
        if not block:
            checks[iface] = CheckResult(iface, False, "interface missing")
            continue
        up = "UP" in block
        active = "ERROR-ACTIVE" in block
        bitrate = _regex(block, r"bitrate\s+(\d+)")
        ok = up and active and bitrate == "1000000"
        checks[iface] = CheckResult(
            iface,
            ok,
            f"UP={up}, state={'ERROR-ACTIVE' if active else 'not active'}, bitrate={bitrate or 'unknown'}",
            {"up": up, "error_active": active, "bitrate": bitrate},
        )
    return checks


def scan_cameras() -> dict[str, CheckResult]:
    checks = {}
    for name, binding in get_camera_bindings().items():
        path = binding.effective_path
        exists = Path(path).exists()
        if not exists:
            checks[f"camera_{name}"] = CheckResult(f"camera_{name}", False, f"{path} missing")
            continue
        ok, shape = _read_camera_once(path)
        checks[f"camera_{name}"] = CheckResult(
            f"camera_{name}",
            ok,
            f"{path} ({binding.mode}), shape={shape}",
            {"path": path, "shape": shape, "mode": binding.mode, "serial": binding.serial},
        )
    return checks


def _read_camera_once(path: str, attempts: int = 5) -> tuple[bool, tuple[int, ...] | None]:
    for attempt in range(attempts):
        cap = _open_capture(path)
        try:
            ok, frame = cap.read() if cap.isOpened() else (False, None)
            if ok and frame is not None:
                return True, tuple(frame.shape)
        finally:
            cap.release()
        if attempt < attempts - 1:
            time.sleep(0.25)
    return False, None


def scan_bridge_and_ros() -> dict[str, CheckResult]:
    checks = {}
    try:
        client = BridgeClient(timeout_s=0.25)
        snap = client.snapshot(time.monotonic_ns())
        health = client.health()
        client.close()
    except Exception as exc:
        return {
            "bridge": CheckResult("bridge", False, f"bridge unavailable: {exc}"),
            "left_arm": CheckResult("left_arm", False, "bridge unavailable"),
            "right_arm": CheckResult("right_arm", False, "bridge unavailable"),
            "vr": CheckResult("vr", False, "bridge unavailable"),
            "left_action": CheckResult("left_action", False, "bridge unavailable"),
            "right_action": CheckResult("right_action", False, "bridge unavailable"),
        }
    ages = snap.get("ages_ms", {})
    counts = health.get("counts", {})
    checks["bridge"] = CheckResult("bridge", bool(snap.get("ok")), f"counts={counts}", {"counts": counts})
    checks["left_arm"] = _age_group_check(
        "left_arm",
        ages,
        ["left_feedback_joint_states", "left_feedback_tcp_pose", "left_feedback_gripper_status"],
    )
    checks["right_arm"] = _age_group_check(
        "right_arm",
        ages,
        ["right_feedback_joint_states", "right_feedback_tcp_pose", "right_feedback_gripper_status"],
    )
    xr_age = ages.get("xr_pose")
    checks["vr"] = CheckResult(
        "vr",
        bool(xr_age is not None and xr_age < 500.0),
        f"xr_age_ms={_fmt_age(xr_age)}",
        {"xr_age_ms": xr_age},
    )
    for side in ["left", "right"]:
        age = ages.get(f"{side}_control_move_j")
        checks[f"{side}_action"] = CheckResult(
            f"{side}_action",
            bool(age is not None and age < 500.0),
            f"{side}_control_move_j age_ms={_fmt_age(age)}",
            {"age_ms": age},
        )
    return checks


def _age_group_check(name: str, ages: dict[str, float | None], keys: list[str]) -> CheckResult:
    values = {key: ages.get(key) for key in keys}
    ok = all(value is not None and value < 100.0 for value in values.values())
    details = ", ".join(f"{key}={_fmt_age(value)}ms" for key, value in values.items())
    return CheckResult(name, ok, details, values)


def _fmt_age(value: float | None) -> str:
    return "missing" if value is None else f"{value:.1f}"


def _run(cmd: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", "-lc", cmd], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def _extract_iface_block(text: str, iface: str) -> str:
    pattern = re.compile(rf"^\d+:\s+{re.escape(iface)}:.*?(?=^\d+:\s|\Z)", re.S | re.M)
    match = pattern.search(text)
    return match.group(0) if match else ""


def _regex(text: str, pattern: str) -> str | None:
    match = re.search(pattern, text)
    return match.group(1) if match else None


def _open_capture(path: str) -> cv2.VideoCapture:
    last_cap = cv2.VideoCapture()
    for target in _opencv_targets(path):
        cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            return cap
        cap.release()
        last_cap = cap
    return last_cap


def _opencv_targets(path: str) -> list[int | str]:
    targets: list[int | str] = [path]
    try:
        resolved = Path(path).resolve()
    except OSError:
        resolved = Path(path)
    resolved_str = str(resolved)
    if resolved_str not in targets:
        targets.append(resolved_str)
    name = resolved.name
    if name.startswith("video") and name[5:].isdigit():
        targets.append(int(name[5:]))
    return targets
