from __future__ import annotations

import os
import shutil
import threading
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from camera_config import get_camera_paths
from camera_preview import downscale_rgb


QUALITY_RATIO_KEYS = [
    "vr_valid",
    "left_action_valid",
    "right_action_valid",
    "action_valid",
    "left_action_semantic_valid",
    "right_action_semantic_valid",
    "action_semantic_valid",
    "bridge_valid",
    "cam_top_valid",
    "cam_left_wrist_valid",
    "cam_right_wrist_valid",
    "left_control_active",
    "right_control_active",
    "left_action_from_ik",
    "right_action_from_ik",
    "left_action_from_hold",
    "right_action_from_hold",
]

QUALITY_MAX_KEYS = [
    "xr_age_ms",
    "left_action_age_ms",
    "right_action_age_ms",
    "bridge_lost_duration_ms",
    "schedule_lag_ms",
    "cam_top_age_ms",
    "cam_left_wrist_age_ms",
    "cam_right_wrist_age_ms",
]


@dataclass
class RecordSettings:
    dataset_root: str
    dataset_name: str
    repo_id: str
    task: str
    episode_time_s: float
    reset_time_s: float
    fps: int = 30
    robot_id: str = "pico_piper_webui"
    teleop_id: str = "pico_vr_webui"
    video: bool = True
    vcodec: str = "h264"
    streaming_encoding: bool = False
    push_to_hub: bool = False

    @property
    def root_path(self) -> Path:
        return Path(self.dataset_root) / self.dataset_name


@dataclass
class RecorderStatus:
    state: str = "IDLE"
    message: str = "Idle"
    error: str | None = None
    dataset_path: str | None = None
    repo_id: str | None = None
    task: str = ""
    current_episode: int = 0
    saved_episodes: int = 0
    episode_elapsed_s: float = 0.0
    episode_time_s: float = 0.0
    reset_remaining_s: float = 0.0
    countdown_remaining_s: float = 0.0
    latest_obs: dict[str, float] = field(default_factory=dict)
    latest_action: dict[str, float] = field(default_factory=dict)
    latest_images: dict[str, np.ndarray] = field(default_factory=dict)
    started_at: float | None = None
    finished_at: float | None = None
    pending_decision: bool = False
    pending_episode_elapsed_s: float = 0.0
    pending_episode_estimated_frames: int = 0
    pending_episode_quality: dict[str, float] = field(default_factory=dict)
    finish_requested: bool = False


class RecorderController:
    def __init__(self):
        self._thread: threading.Thread | None = None
        self._status = RecorderStatus()
        self._lock = threading.RLock()
        self._decision_condition = threading.Condition(self._lock)
        self._events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False, "finish_stage": False}
        self._episode_decision: str | None = None
        self._settings: RecordSettings | None = None
        self._episode_quality = _new_episode_quality_stats()

    def start(self, settings: RecordSettings) -> tuple[bool, str]:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return False, "A recording job is already running."
            if settings.root_path.exists():
                return False, f"Dataset path already exists: {settings.root_path}"
            self._events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False, "finish_stage": False}
            self._episode_decision = None
            self._settings = settings
            self._status = RecorderStatus(
                state="STARTING",
                message="Starting recorder",
                dataset_path=str(settings.root_path),
                repo_id=settings.repo_id,
                task=settings.task,
                episode_time_s=settings.episode_time_s,
                started_at=time.time(),
            )
            self._thread = threading.Thread(target=self._run, args=(settings,), name="lerobot-web-recorder", daemon=True)
            self._thread.start()
            return True, "Recording job started."

    def finish_episode(self) -> None:
        with self._lock:
            if self._status.state == "RECORDING":
                self._events["exit_early"] = True
                self._status.message = "Finishing current episode and preparing save decision."

    def rerecord_episode(self) -> None:
        with self._lock:
            if self._status.state == "RECORDING":
                self._events["rerecord_episode"] = True
                self._events["exit_early"] = True
                self._status.message = "Discarding current episode and preparing rerecord."
            elif self._status.state == "AWAIT_SAVE_DECISION":
                self._episode_decision = "discard_retake"
                self._status.message = "Discarding current episode and recording it again."
                self._decision_condition.notify_all()

    def stop(self) -> None:
        with self._lock:
            self._events["stop_recording"] = True
            self._events["exit_early"] = True
            self._status.message = "Stopping recording."
            self._decision_condition.notify_all()

    def request_finish_stage(self) -> None:
        with self._lock:
            state = self._status.state
            self._events["finish_stage"] = True
            self._status.finish_requested = True
            if state == "COUNTDOWN":
                self._events["stop_recording"] = True
                self._events["exit_early"] = True
                self._status.message = "Finishing collection stage before the next episode starts."
                self._decision_condition.notify_all()
            elif state == "RECORDING":
                self._events["exit_early"] = True
                self._status.message = "Finishing current episode, then ending this collection stage."
            elif state == "RESETTING":
                self._status.message = "Finishing this collection stage after reset and save decision."
            elif state == "AWAIT_SAVE_DECISION":
                self._status.message = "Choose save-and-finish or discard-and-finish."

    def save_and_continue(self) -> None:
        self._set_episode_decision("save_continue", "Saving current episode and continuing.")

    def save_and_finish(self) -> None:
        self._set_episode_decision("save_finish", "Saving current episode and finishing this collection stage.")

    def discard_and_retake(self) -> None:
        self._set_episode_decision("discard_retake", "Discarding current episode and recording it again.")

    def discard_and_finish(self) -> None:
        self._set_episode_decision("discard_finish", "Discarding current episode and finishing this collection stage.")

    def _set_episode_decision(self, decision: str, message: str) -> None:
        with self._decision_condition:
            if self._status.state != "AWAIT_SAVE_DECISION":
                return
            self._episode_decision = decision
            self._status.message = message
            self._decision_condition.notify_all()

    def arm_home(self) -> tuple[bool, str]:
        return (
            False,
            "Arm homing is intentionally not implemented yet. Confirm the real ROS home/reset interface before enabling it.",
        )

    def status(self) -> RecorderStatus:
        with self._lock:
            return RecorderStatus(
                state=self._status.state,
                message=self._status.message,
                error=self._status.error,
                dataset_path=self._status.dataset_path,
                repo_id=self._status.repo_id,
                task=self._status.task,
                current_episode=self._status.current_episode,
                saved_episodes=self._status.saved_episodes,
                episode_elapsed_s=self._status.episode_elapsed_s,
                episode_time_s=self._status.episode_time_s,
                reset_remaining_s=self._status.reset_remaining_s,
                countdown_remaining_s=self._status.countdown_remaining_s,
                latest_obs=dict(self._status.latest_obs),
                latest_action=dict(self._status.latest_action),
                latest_images={k: v.copy() for k, v in self._status.latest_images.items()},
                started_at=self._status.started_at,
                finished_at=self._status.finished_at,
                pending_decision=self._status.pending_decision,
                pending_episode_elapsed_s=self._status.pending_episode_elapsed_s,
                pending_episode_estimated_frames=self._status.pending_episode_estimated_frames,
                pending_episode_quality=dict(self._status.pending_episode_quality),
                finish_requested=self._status.finish_requested,
            )

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._status.state not in {"IDLE", "DONE", "ERROR"}

    @property
    def can_start_new_stage(self) -> bool:
        with self._lock:
            thread_alive = bool(self._thread and self._thread.is_alive())
            return self._status.state in {"IDLE", "DONE", "ERROR"} and not thread_alive

    def _set(self, **kwargs: Any) -> None:
        with self._lock:
            for key, value in kwargs.items():
                setattr(self._status, key, value)

    def _run(self, settings: RecordSettings) -> None:
        dataset = None
        dataset_finalized = False
        robot = None
        teleop = None
        try:
            from lerobot.datasets.feature_utils import combine_feature_dicts
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
            from lerobot.datasets.video_utils import VideoEncodingManager
            from lerobot.processor import make_default_processors
            from lerobot.scripts.lerobot_record import record_loop

            from lerobot_robot_pico_piper.config_pico_piper import PicoPiperConfig
            from lerobot_robot_pico_piper.pico_piper import PicoPiper
            from lerobot_teleoperator_pico_vr.config_pico_vr import PicoVrConfig
            from lerobot_teleoperator_pico_vr.pico_vr import PicoVr

            robot_cfg = PicoPiperConfig(
                id=settings.robot_id,
                fps=settings.fps,
                camera_paths=get_camera_paths(),
                no_op_send_action=True,
                require_bridge=True,
            )
            teleop_cfg = PicoVrConfig(id=settings.teleop_id, require_bridge=True)
            robot = PicoPiper(robot_cfg)
            teleop = PicoVr(teleop_cfg)

            teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
            dataset_features = combine_feature_dicts(
                aggregate_pipeline_dataset_features(
                    pipeline=teleop_action_processor,
                    initial_features=create_initial_features(action=robot.action_features),
                    use_videos=settings.video,
                ),
                aggregate_pipeline_dataset_features(
                    pipeline=robot_observation_processor,
                    initial_features=create_initial_features(observation=robot.observation_features),
                    use_videos=settings.video,
                ),
            )

            self._set(state="CONNECTING", message="Connecting robot, teleop and cameras.")
            dataset = LeRobotDataset.create(
                settings.repo_id,
                settings.fps,
                root=settings.root_path,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=settings.video,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
                batch_encoding_size=1,
                vcodec=settings.vcodec,
                streaming_encoding=settings.streaming_encoding,
                encoder_queue_maxsize=30,
                encoder_threads=None,
            )

            robot.connect()
            teleop.connect()
            self._wrap_robot(robot)
            self._wrap_teleop(teleop)

            saved = 0
            with VideoEncodingManager(dataset):
                while not self._events["stop_recording"]:
                    episode_index = saved
                    self._events["exit_early"] = False
                    self._events["rerecord_episode"] = False
                    self._episode_decision = None
                    self._set(
                        current_episode=episode_index + 1,
                        saved_episodes=saved,
                        episode_elapsed_s=0.0,
                        reset_remaining_s=0.0,
                        countdown_remaining_s=0.0,
                        pending_decision=False,
                        pending_episode_elapsed_s=0.0,
                        pending_episode_estimated_frames=0,
                        pending_episode_quality={},
                    )
                    self._countdown(5.0, "COUNTDOWN", f"Episode {episode_index + 1} starts after countdown.")
                    if self._events["stop_recording"]:
                        break

                    self._events["exit_early"] = False
                    self._events["rerecord_episode"] = False
                    self._set(
                        state="RECORDING",
                        message=f"Recording episode {episode_index + 1}.",
                        current_episode=episode_index + 1,
                        saved_episodes=saved,
                        episode_elapsed_s=0.0,
                        reset_remaining_s=0.0,
                        countdown_remaining_s=0.0,
                    )
                    self._reset_episode_quality()
                    episode_start = time.time()
                    self._start_elapsed_clock(episode_start, settings.episode_time_s)
                    record_loop(
                        robot=robot,
                        events=self._events,
                        fps=settings.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        policy=None,
                        preprocessor=None,
                        postprocessor=None,
                        dataset=dataset,
                        control_time_s=settings.episode_time_s,
                        single_task=settings.task,
                        display_data=False,
                    )
                    self._set(episode_elapsed_s=min(settings.episode_time_s, max(0.0, time.time() - episode_start)))

                    if self._events["rerecord_episode"]:
                        dataset.clear_episode_buffer()
                        self._set(message="Current episode discarded.")
                        if not self._events["stop_recording"]:
                            self._reset_loop(robot, teleop, teleop_action_processor, robot_action_processor, robot_observation_processor, settings)
                        continue

                    pending_elapsed = self.status().episode_elapsed_s
                    episode_quality = self._episode_quality_summary()
                    if not self._events["stop_recording"]:
                        self._reset_loop(robot, teleop, teleop_action_processor, robot_action_processor, robot_observation_processor, settings)

                    if self._events["stop_recording"]:
                        dataset.clear_episode_buffer()
                        break

                    decision = self._wait_for_episode_decision(settings, episode_index + 1, pending_elapsed, episode_quality)
                    if self._events["finish_stage"] and decision == "save_continue":
                        decision = "save_finish"
                    if self._events["finish_stage"] and decision == "discard_retake":
                        decision = "discard_finish"

                    if decision in {"save_continue", "save_finish"}:
                        self._set(state="SAVING", message=f"Saving episode {episode_index + 1}.")
                        dataset.save_episode()
                        saved += 1
                        self._set(saved_episodes=saved, pending_decision=False, finish_requested=False)
                    else:
                        self._set(state="DISCARDING", message=f"Discarding episode {episode_index + 1}.")
                        dataset.clear_episode_buffer()
                        self._set(pending_decision=False, finish_requested=False)

                    if decision in {"save_finish", "discard_finish"}:
                        break

            self._set(state="FINALIZING", message="Finalizing dataset.")
            dataset.finalize()
            dataset_finalized = True
            _chown_to_host_user(settings.root_path)
            self._set(state="CLEANING_UP", message="Cleaning up recording resources.")
        except Exception as exc:
            self._set(state="ERROR", error=f"{exc}\n{traceback.format_exc()}", message=str(exc), finished_at=time.time())
            if settings.root_path.exists():
                _chown_to_host_user(settings.root_path)
        finally:
            try:
                if dataset is not None and not dataset_finalized and self.status().state not in {"DONE", "ERROR"}:
                    dataset.finalize()
                    dataset_finalized = True
            except Exception:
                pass
            try:
                if robot is not None and robot.is_connected:
                    robot.disconnect()
            except Exception:
                pass
            try:
                if teleop is not None and teleop.is_connected:
                    teleop.disconnect()
            except Exception:
                pass
            if self.status().state == "CLEANING_UP":
                self._set(state="DONE", message="Recording complete.", finished_at=time.time())

    def _reset_loop(self, robot: Any, teleop: Any, teleop_action_processor: Any, robot_action_processor: Any, robot_observation_processor: Any, settings: RecordSettings) -> None:
        from lerobot.scripts.lerobot_record import record_loop

        if settings.reset_time_s <= 0:
            self._set(reset_remaining_s=0.0)
            return

        reset_start = time.time()
        self._set(state="RESETTING", message="Reset the desktop/environment before the next episode.")
        self._start_reset_clock(reset_start, settings.reset_time_s)
        self._events["exit_early"] = False
        record_loop(
            robot=robot,
            events=self._events,
            fps=settings.fps,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            policy=None,
            preprocessor=None,
            postprocessor=None,
            dataset=None,
            control_time_s=settings.reset_time_s,
            single_task=settings.task,
            display_data=False,
        )
        self._set(reset_remaining_s=0.0)

    def _wait_for_episode_decision(self, settings: RecordSettings, episode_number: int, elapsed_s: float, quality: dict[str, float]) -> str:
        estimated_frames = int(round(max(0.0, elapsed_s) * settings.fps))
        with self._decision_condition:
            self._episode_decision = None
            self._status.state = "AWAIT_SAVE_DECISION"
            self._status.message = f"Episode {episode_number} is ready for review."
            self._status.pending_decision = True
            self._status.pending_episode_elapsed_s = elapsed_s
            self._status.pending_episode_estimated_frames = estimated_frames
            self._status.pending_episode_quality = dict(quality)
            while self._episode_decision is None and not self._events["stop_recording"]:
                self._decision_condition.wait(timeout=0.5)
            if self._events["stop_recording"] and self._episode_decision is None:
                return "discard_finish"
            return self._episode_decision or "discard_finish"

    def _countdown(self, seconds: float, state: str, message: str) -> None:
        deadline = time.time() + seconds
        self._set(state=state, message=message)
        while time.time() < deadline:
            if self._events["stop_recording"]:
                return
            self._set(countdown_remaining_s=max(0.0, deadline - time.time()))
            time.sleep(0.1)
        self._set(countdown_remaining_s=0.0)

    def _start_elapsed_clock(self, start: float, total: float) -> None:
        def run() -> None:
            while True:
                st = self.status().state
                if st != "RECORDING":
                    return
                elapsed = min(total, max(0.0, time.time() - start))
                self._set(episode_elapsed_s=elapsed)
                time.sleep(0.2)

        threading.Thread(target=run, daemon=True).start()

    def _start_reset_clock(self, start: float, total: float) -> None:
        def run() -> None:
            while True:
                st = self.status().state
                if st != "RESETTING":
                    return
                remaining = max(0.0, total - (time.time() - start))
                self._set(reset_remaining_s=remaining)
                if remaining <= 0:
                    return
                time.sleep(0.2)

        threading.Thread(target=run, daemon=True).start()

    def _wrap_robot(self, robot: Any) -> None:
        original = robot.get_observation

        def wrapped() -> dict[str, Any]:
            obs = original()
            scalar = {key: float(value) for key, value in obs.items() if isinstance(value, (int, float))}
            self._update_episode_quality(scalar)
            images = {}
            for name in get_camera_paths():
                image = obs.get(name)
                if isinstance(image, np.ndarray):
                    images[name] = downscale_rgb(image)
            self._set(latest_obs=scalar, latest_images=images)
            return obs

        robot.get_observation = wrapped

    def _reset_episode_quality(self) -> None:
        with self._lock:
            self._episode_quality = _new_episode_quality_stats()

    def _update_episode_quality(self, obs: dict[str, float]) -> None:
        with self._lock:
            if self._status.state != "RECORDING":
                return
            stats = self._episode_quality
            stats["frames"] += 1.0
            for key in QUALITY_RATIO_KEYS:
                if obs.get(key, 0.0) > 0.0:
                    stats[f"{key}_frames"] += 1.0
            for key in QUALITY_MAX_KEYS:
                value = obs.get(key)
                if value is not None:
                    stats[f"max_{key}"] = max(stats[f"max_{key}"], float(value))

    def _episode_quality_summary(self) -> dict[str, float]:
        with self._lock:
            stats = dict(self._episode_quality)
        frames = max(0.0, stats.get("frames", 0.0))
        summary = {"frames": frames}
        for key in QUALITY_RATIO_KEYS:
            valid = stats.get(f"{key}_frames", 0.0)
            summary[f"{key}_ratio"] = valid / frames if frames > 0 else 0.0
            summary[f"{key}_frames"] = valid
        for key in QUALITY_MAX_KEYS:
            summary[f"max_{key}"] = stats.get(f"max_{key}", 0.0)
        return summary

    def _wrap_teleop(self, teleop: Any) -> None:
        original = teleop.get_action

        def wrapped() -> dict[str, Any]:
            action = original()
            self._set(latest_action={key: float(value) for key, value in action.items()})
            return action

        teleop.get_action = wrapped


def _new_episode_quality_stats() -> dict[str, float]:
    stats = {"frames": 0.0}
    for key in QUALITY_RATIO_KEYS:
        stats[f"{key}_frames"] = 0.0
    for key in QUALITY_MAX_KEYS:
        stats[f"max_{key}"] = 0.0
    return stats


def check_dataset_name(root: str, name: str) -> tuple[bool, str, str | None]:
    path = Path(root) / name
    if not name.strip():
        return False, "Dataset name is required.", None
    if "/" in name or "\\" in name:
        return False, "Dataset name must not contain path separators.", None
    if path.exists():
        return False, f"Dataset already exists: {path}", suggest_dataset_name(root, name)
    return True, f"Dataset path is available: {path}", None


def suggest_dataset_name(root: str, name: str) -> str:
    base = name.rstrip("0123456789_") or name
    for i in range(1, 10000):
        candidate = f"{base}_{i:03d}"
        if not (Path(root) / candidate).exists():
            return candidate
    return f"{name}_{int(time.time())}"


def _chown_to_host_user(path: Path) -> None:
    if not path.exists():
        return
    try:
        shutil.chown(path, user=1000, group=1000)
    except Exception:
        pass
    for root, dirs, files in os.walk(path):
        for name in dirs:
            try:
                shutil.chown(Path(root) / name, user=1000, group=1000)
            except Exception:
                pass
        for name in files:
            try:
                shutil.chown(Path(root) / name, user=1000, group=1000)
            except Exception:
                pass
