from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction
from lerobot.utils.errors import DeviceNotConnectedError

from lerobot_robot_pico_piper.bridge_client import BridgeClient
from lerobot_robot_pico_piper.pico_piper import ACTION_KEYS
from lerobot_robot_pico_piper.shared_timing import get_frame_context

from .config_pico_vr import PicoVrConfig

logger = logging.getLogger(__name__)


class PicoVr(Teleoperator):
    config_class = PicoVrConfig
    name = "pico_vr"

    def __init__(self, config: PicoVrConfig):
        super().__init__(config)
        self.config = config
        self._bridge = BridgeClient(config.bridge_host, config.bridge_port, config.bridge_timeout_s)
        self._is_connected = False
        self._last_action: RobotAction = {key: 0.0 for key in ACTION_KEYS}
        self._last_action_time_ns: int | None = None
        self._bridge_failure_count = 0
        self._bridge_recovery_count = 0
        self._bridge_lost_since_ns: int | None = None
        self._last_bridge_warning_s = 0.0

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in ACTION_KEYS}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

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
            logger.warning("Pico/Piper bridge unavailable; actions will be zeros: %s", exc)
        self._is_connected = True

    def disconnect(self) -> None:
        self._bridge.close()
        self._is_connected = False

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def get_action(self) -> RobotAction:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        frame_time_ns, snapshot = get_frame_context()
        if frame_time_ns is None:
            frame_time_ns = time.monotonic_ns()
        if not snapshot:
            try:
                snapshot = self._bridge.snapshot(frame_time_ns)
                self._mark_bridge_success()
            except Exception as exc:
                self._mark_bridge_failure(exc)
                return self._fallback_action()
        elif snapshot.get("ok"):
            self._mark_bridge_success()
        action = self._action_from_snapshot(snapshot)
        self._last_action = dict(action)
        self._last_action_time_ns = time.monotonic_ns()
        return action

    def _fallback_action(self) -> RobotAction:
        if self._last_action_time_ns is None:
            return {key: 0.0 for key in ACTION_KEYS}
        age_ms = (time.monotonic_ns() - self._last_action_time_ns) / 1_000_000.0
        if age_ms <= self.config.max_cached_action_age_ms:
            return dict(self._last_action)
        return {key: 0.0 for key in ACTION_KEYS}

    def _action_from_snapshot(self, snapshot: dict[str, Any] | None) -> RobotAction:
        action = {key: 0.0 for key in ACTION_KEYS}
        if not snapshot or not snapshot.get("ok"):
            return action
        sources = snapshot.get("sources", {})
        ages = snapshot.get("ages_ms", {})
        self._fill_side_action(action, sources, ages, "left")
        self._fill_side_action(action, sources, ages, "right")
        return action

    @staticmethod
    def _sample_data(sample: dict[str, Any] | None) -> dict[str, Any]:
        if not sample or not sample.get("valid"):
            return {}
        data = sample.get("data")
        return data if isinstance(data, dict) else {}

    def _fill_side_action(
        self,
        action: dict[str, float],
        sources: dict[str, Any],
        ages: dict[str, float | None],
        side: str,
    ) -> None:
        move_key = f"{side}_control_move_j"
        if self._is_fresh(ages, move_key) and self._fill_joint_positions(action, sources.get(move_key), side):
            pass
        else:
            self._fill_joint_positions(action, sources.get(f"{side}_feedback_joint_states"), side)

        gripper_key = f"{side}_control_joint_states"
        if self._is_fresh(ages, gripper_key) and self._fill_control_gripper(action, sources.get(gripper_key), side):
            return
        self._fill_feedback_gripper(action, sources.get(f"{side}_feedback_gripper_status"), side)

    def _is_fresh(self, ages: dict[str, float | None], key: str) -> bool:
        age = ages.get(key)
        return bool(age is not None and age <= self.config.max_cached_action_age_ms)

    def _fill_joint_positions(self, action: dict[str, float], sample: dict[str, Any] | None, side: str) -> bool:
        data = self._sample_data(sample)
        positions = data.get("position") or []
        if len(positions) < 6:
            return False
        for i in range(6):
            action[f"{side}_joint_{i + 1}"] = float(positions[i])
        return True

    def _fill_control_gripper(self, action: dict[str, float], sample: dict[str, Any] | None, side: str) -> bool:
        data = self._sample_data(sample)
        positions = data.get("position") or []
        if not positions:
            return False
        action[f"{side}_gripper"] = float(positions[0])
        return True

    def _fill_feedback_gripper(self, action: dict[str, float], sample: dict[str, Any] | None, side: str) -> bool:
        data = self._sample_data(sample)
        if "width" not in data:
            return False
        action[f"{side}_gripper"] = float(data.get("width", 0.0))
        return True

    def _mark_bridge_failure(self, exc: Exception) -> None:
        self._bridge_failure_count += 1
        if self._bridge_lost_since_ns is None:
            self._bridge_lost_since_ns = time.monotonic_ns()
        now_s = time.monotonic()
        if now_s - self._last_bridge_warning_s >= 2.0:
            logger.warning("Bridge snapshot failed; using cached/zero VR action and continuing: %s", exc)
            self._last_bridge_warning_s = now_s

    def _mark_bridge_success(self) -> None:
        if self._bridge_lost_since_ns is not None:
            self._bridge_recovery_count += 1
            self._bridge_lost_since_ns = None
