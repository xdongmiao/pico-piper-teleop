from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceNotConnectedError

from lerobot.robots.robot import Robot

from .bridge_client import BridgeClient
from .config_pico_piper import PicoPiperConfig
from .shared_timing import set_frame_context
from .threaded_camera import ThreadedOpenCVCamera

logger = logging.getLogger(__name__)

LEFT_JOINT_KEYS = [f"left_joint_{i}" for i in range(1, 7)]
RIGHT_JOINT_KEYS = [f"right_joint_{i}" for i in range(1, 7)]
ACTION_KEYS = [*LEFT_JOINT_KEYS, "left_gripper", *RIGHT_JOINT_KEYS, "right_gripper"]

OBS_KEYS = [
    *LEFT_JOINT_KEYS,
    *RIGHT_JOINT_KEYS,
    "left_gripper_width",
    "left_gripper_force",
    "right_gripper_width",
    "right_gripper_force",
    "left_tcp_x",
    "left_tcp_y",
    "left_tcp_z",
    "left_tcp_qx",
    "left_tcp_qy",
    "left_tcp_qz",
    "left_tcp_qw",
    "right_tcp_x",
    "right_tcp_y",
    "right_tcp_z",
    "right_tcp_qx",
    "right_tcp_qy",
    "right_tcp_qz",
    "right_tcp_qw",
    "xr_head_x",
    "xr_head_y",
    "xr_head_z",
    "xr_head_qx",
    "xr_head_qy",
    "xr_head_qz",
    "xr_head_qw",
    "xr_left_x",
    "xr_left_y",
    "xr_left_z",
    "xr_left_qx",
    "xr_left_qy",
    "xr_left_qz",
    "xr_left_qw",
    "xr_left_axis_x",
    "xr_left_axis_y",
    "xr_left_axis_click",
    "xr_left_gripper",
    "xr_left_trigger",
    "xr_left_primary",
    "xr_left_secondary",
    "xr_left_menu",
    "xr_right_x",
    "xr_right_y",
    "xr_right_z",
    "xr_right_qx",
    "xr_right_qy",
    "xr_right_qz",
    "xr_right_qw",
    "xr_right_axis_x",
    "xr_right_axis_y",
    "xr_right_axis_click",
    "xr_right_gripper",
    "xr_right_trigger",
    "xr_right_primary",
    "xr_right_secondary",
    "xr_right_menu",
    "frame_index",
    "frame_time_s",
    "loop_dt_ms",
    "schedule_lag_ms",
    "cam_top_age_ms",
    "cam_left_wrist_age_ms",
    "cam_right_wrist_age_ms",
    "cam_top_valid",
    "cam_left_wrist_valid",
    "cam_right_wrist_valid",
    "left_joint_age_ms",
    "right_joint_age_ms",
    "left_tcp_age_ms",
    "right_tcp_age_ms",
    "left_gripper_age_ms",
    "right_gripper_age_ms",
    "xr_age_ms",
    "left_action_age_ms",
    "right_action_age_ms",
    "bridge_valid",
    "vr_valid",
    "left_action_valid",
    "right_action_valid",
    "action_valid",
    "left_control_active",
    "right_control_active",
    "left_action_from_ik",
    "right_action_from_ik",
    "left_action_from_hold",
    "right_action_from_hold",
    "left_action_semantic_valid",
    "right_action_semantic_valid",
    "action_semantic_valid",
    "bridge_failure_count",
    "bridge_recovery_count",
    "bridge_lost_duration_ms",
]


class PicoPiper(Robot):
    config_class = PicoPiperConfig
    name = "pico_piper"

    def __init__(self, config: PicoPiperConfig):
        super().__init__(config)
        self.config = config
        self.cameras = dict(config.camera_paths)
        self._camera_threads: dict[str, ThreadedOpenCVCamera] = {}
        self._bridge = BridgeClient(config.bridge_host, config.bridge_port, config.bridge_timeout_s)
        self._is_connected = False
        self._episode_start_ns: int | None = None
        self._last_frame_time_ns: int | None = None
        self._frame_index = 0
        self._schedule_realigned = False
        self._bridge_failure_count = 0
        self._bridge_recovery_count = 0
        self._bridge_lost_since_ns: int | None = None
        self._last_bridge_warning_s = 0.0

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {
            name: (self.config.camera_height, self.config.camera_width, 3) for name in self.cameras
        }
        features.update({key: float for key in OBS_KEYS})
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in ACTION_KEYS}

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            return
        try:
            self._bridge.ping()
        except Exception as exc:
            if self.config.require_bridge:
                raise DeviceNotConnectedError(
                    f"Cannot connect to Pico/Piper bridge at "
                    f"{self.config.bridge_host}:{self.config.bridge_port}: {exc}"
                ) from exc
            logger.warning("Pico/Piper bridge unavailable; continuing with zero ROS state: %s", exc)

        for name, path in self.cameras.items():
            cam = ThreadedOpenCVCamera(
                name=name,
                path=path,
                width=self.config.camera_width,
                height=self.config.camera_height,
                fps=self.config.camera_fps,
                fourcc=self.config.camera_fourcc,
                ring_size=self.config.camera_ring_size,
            )
            cam.start()
            self._camera_threads[name] = cam

        warmup_deadline = time.monotonic() + 3.0
        while time.monotonic() < warmup_deadline:
            if all(cam.is_connected and cam.frames >= 5 for cam in self._camera_threads.values()):
                break
            time.sleep(0.02)
        failed = {name: cam.last_error for name, cam in self._camera_threads.items() if not cam.is_connected}
        if failed:
            self.disconnect()
            raise DeviceNotConnectedError(f"Failed to open cameras: {failed}")

        now_ns = time.monotonic_ns()
        self._episode_start_ns = now_ns
        self._last_frame_time_ns = None
        self._frame_index = 0
        self._is_connected = True

    def disconnect(self) -> None:
        for cam in self._camera_threads.values():
            cam.stop()
        self._camera_threads.clear()
        self._bridge.close()
        self._is_connected = False

    def _next_frame_time_ns(self) -> int:
        now_ns = time.monotonic_ns()
        self._schedule_realigned = False
        if self._episode_start_ns is None:
            self._episode_start_ns = now_ns
        period_ns = int(1_000_000_000 / self.config.fps)
        frame_time_ns = self._episode_start_ns + self._frame_index * period_ns
        if frame_time_ns < now_ns - period_ns:
            self._episode_start_ns = now_ns - self._frame_index * period_ns
            frame_time_ns = now_ns
            self._last_frame_time_ns = None
            self._schedule_realigned = True
        if self.config.sleep_until_frame_time and frame_time_ns > now_ns:
            time.sleep((frame_time_ns - now_ns) / 1_000_000_000)
        return frame_time_ns

    def get_observation(self) -> RobotObservation:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        frame_time_ns = self._next_frame_time_ns()
        now_after_wait_ns = time.monotonic_ns()
        loop_dt_ms = 0.0
        if self._last_frame_time_ns is not None:
            loop_dt_ms = (frame_time_ns - self._last_frame_time_ns) / 1_000_000.0
        if self._schedule_realigned:
            loop_dt_ms = 0.0
        schedule_lag_ms = (now_after_wait_ns - frame_time_ns) / 1_000_000.0

        snapshot: dict[str, Any] | None
        try:
            snapshot = self._bridge.snapshot(frame_time_ns)
            self._mark_bridge_success()
        except Exception as exc:
            self._mark_bridge_failure(exc)
            snapshot = None
        set_frame_context(frame_time_ns, snapshot)

        obs: RobotObservation = self._state_from_snapshot(snapshot, frame_time_ns)
        for name, cam in self._camera_threads.items():
            image, age_ms, valid = cam.sample_at(frame_time_ns)
            obs[name] = image
            obs[f"cam_{name}_age_ms"] = float(age_ms)
            obs[f"cam_{name}_valid"] = float(valid and age_ms <= self.config.max_camera_age_ms)

        obs["frame_index"] = float(self._frame_index)
        obs["frame_time_s"] = float((frame_time_ns - (self._episode_start_ns or frame_time_ns)) / 1_000_000_000.0)
        obs["loop_dt_ms"] = float(loop_dt_ms)
        obs["schedule_lag_ms"] = float(schedule_lag_ms)
        obs["bridge_valid"] = float(bool(snapshot and snapshot.get("ok")))
        obs["bridge_failure_count"] = float(self._bridge_failure_count)
        obs["bridge_recovery_count"] = float(self._bridge_recovery_count)
        obs["bridge_lost_duration_ms"] = float(self._bridge_lost_duration_ms())

        for key in OBS_KEYS:
            obs.setdefault(key, 0.0)

        self._last_frame_time_ns = frame_time_ns
        self._frame_index += 1
        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        sanitized = {key: float(action.get(key, 0.0)) for key in ACTION_KEYS}
        if not self.config.no_op_send_action:
            raise NotImplementedError("Direct LeRobot control is intentionally disabled for Pico/Piper.")
        return sanitized

    def _state_from_snapshot(self, snapshot: dict[str, Any] | None, frame_time_ns: int) -> dict[str, float]:
        out = {key: 0.0 for key in OBS_KEYS}
        for key in [
            "left_joint_age_ms",
            "right_joint_age_ms",
            "left_tcp_age_ms",
            "right_tcp_age_ms",
            "left_gripper_age_ms",
            "right_gripper_age_ms",
            "xr_age_ms",
            "left_action_age_ms",
            "right_action_age_ms",
        ]:
            out[key] = 1_000_000.0
        if not snapshot or not snapshot.get("ok"):
            return out
        sources = snapshot.get("sources", {})
        ages = snapshot.get("ages_ms", {})

        self._fill_joint(out, sources.get("left_feedback_joint_states"), "left")
        self._fill_joint(out, sources.get("right_feedback_joint_states"), "right")
        self._fill_tcp(out, sources.get("left_feedback_tcp_pose"), "left")
        self._fill_tcp(out, sources.get("right_feedback_tcp_pose"), "right")
        self._fill_gripper(out, sources.get("left_feedback_gripper_status"), "left")
        self._fill_gripper(out, sources.get("right_feedback_gripper_status"), "right")
        self._fill_xr(out, sources.get("xr_pose"))

        age_map = {
            "left_joint_age_ms": "left_feedback_joint_states",
            "right_joint_age_ms": "right_feedback_joint_states",
            "left_tcp_age_ms": "left_feedback_tcp_pose",
            "right_tcp_age_ms": "right_feedback_tcp_pose",
            "left_gripper_age_ms": "left_feedback_gripper_status",
            "right_gripper_age_ms": "right_feedback_gripper_status",
            "xr_age_ms": "xr_pose",
            "left_action_age_ms": "left_control_move_j",
            "right_action_age_ms": "right_control_move_j",
        }
        for key, source_key in age_map.items():
            age = ages.get(source_key)
            out[key] = float(age if age is not None else 1_000_000.0)
        out["vr_valid"] = float(out["xr_age_ms"] <= self.config.max_vr_age_ms)
        out["left_action_valid"] = float(out["left_action_age_ms"] <= self.config.max_action_age_ms)
        out["right_action_valid"] = float(out["right_action_age_ms"] <= self.config.max_action_age_ms)
        out["action_valid"] = float(bool(out["left_action_valid"] or out["right_action_valid"]))
        for side in ["left", "right"]:
            action_fresh = bool(out[f"{side}_action_valid"])
            hold_available = out[f"{side}_joint_age_ms"] <= self.config.max_ros_age_ms
            control_active = bool(out["vr_valid"] and out[f"xr_{side}_gripper"] >= 0.99)
            out[f"{side}_control_active"] = float(control_active)
            out[f"{side}_action_from_ik"] = float(action_fresh)
            out[f"{side}_action_from_hold"] = float((not action_fresh) and hold_available)
            out[f"{side}_action_semantic_valid"] = float(action_fresh or hold_available)
        out["action_semantic_valid"] = float(
            bool(out["left_action_semantic_valid"] and out["right_action_semantic_valid"])
        )
        return out

    def _mark_bridge_failure(self, exc: Exception) -> None:
        self._bridge_failure_count += 1
        if self._bridge_lost_since_ns is None:
            self._bridge_lost_since_ns = time.monotonic_ns()
        now_s = time.monotonic()
        if now_s - self._last_bridge_warning_s >= 2.0:
            logger.warning("Bridge snapshot failed; recording stale quality fields and continuing: %s", exc)
            self._last_bridge_warning_s = now_s

    def _mark_bridge_success(self) -> None:
        if self._bridge_lost_since_ns is not None:
            self._bridge_recovery_count += 1
            self._bridge_lost_since_ns = None

    def _bridge_lost_duration_ms(self) -> float:
        if self._bridge_lost_since_ns is None:
            return 0.0
        return (time.monotonic_ns() - self._bridge_lost_since_ns) / 1_000_000.0

    @staticmethod
    def _sample_data(sample: dict[str, Any] | None) -> dict[str, Any]:
        if not sample or not sample.get("valid"):
            return {}
        data = sample.get("data")
        return data if isinstance(data, dict) else {}

    def _fill_joint(self, out: dict[str, float], sample: dict[str, Any] | None, side: str) -> None:
        data = self._sample_data(sample)
        positions = data.get("position") or []
        for i in range(6):
            out[f"{side}_joint_{i + 1}"] = float(positions[i]) if i < len(positions) else 0.0

    def _fill_tcp(self, out: dict[str, float], sample: dict[str, Any] | None, side: str) -> None:
        data = self._sample_data(sample)
        pose = data.get("pose") or {}
        pos = pose.get("position") or [0.0, 0.0, 0.0]
        quat = pose.get("orientation") or [0.0, 0.0, 0.0, 1.0]
        for key, value in zip(["x", "y", "z"], pos, strict=False):
            out[f"{side}_tcp_{key}"] = float(value)
        for key, value in zip(["qx", "qy", "qz", "qw"], quat, strict=False):
            out[f"{side}_tcp_{key}"] = float(value)

    def _fill_gripper(self, out: dict[str, float], sample: dict[str, Any] | None, side: str) -> None:
        data = self._sample_data(sample)
        out[f"{side}_gripper_width"] = float(data.get("width", 0.0))
        out[f"{side}_gripper_force"] = float(data.get("force", 0.0))

    def _fill_xr(self, out: dict[str, float], sample: dict[str, Any] | None) -> None:
        data = self._sample_data(sample)
        head = data.get("head", {})
        self._fill_pose7(out, "xr_head", head.get("pose") or [])
        for side in ["left", "right"]:
            ctrl = data.get(f"{side}_controller", {})
            self._fill_pose7(out, f"xr_{side}", ctrl.get("pose") or [])
            out[f"xr_{side}_axis_x"] = float(ctrl.get("axis_x", 0.0))
            out[f"xr_{side}_axis_y"] = float(ctrl.get("axis_y", 0.0))
            out[f"xr_{side}_axis_click"] = float(bool(ctrl.get("axis_click", False)))
            out[f"xr_{side}_gripper"] = float(ctrl.get("gripper", 0.0))
            out[f"xr_{side}_trigger"] = float(ctrl.get("trigger", 0.0))
            out[f"xr_{side}_primary"] = float(bool(ctrl.get("primary_button", False)))
            out[f"xr_{side}_secondary"] = float(bool(ctrl.get("secondary_button", False)))
            out[f"xr_{side}_menu"] = float(bool(ctrl.get("menu_button", False)))

    @staticmethod
    def _fill_pose7(out: dict[str, float], prefix: str, pose: list[float]) -> None:
        keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        for i, key in enumerate(keys):
            out[f"{prefix}_{key}"] = float(pose[i]) if i < len(pose) else (1.0 if key == "qw" else 0.0)
