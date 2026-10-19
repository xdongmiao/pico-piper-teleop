from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any


ACTION_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "gripper",
]


@dataclass
class TelemetrySample:
    stamp_s: float
    bridge_ok: bool
    left_action: list[float] = field(default_factory=lambda: [0.0] * 7)
    right_action: list[float] = field(default_factory=lambda: [0.0] * 7)
    left_state: list[float] = field(default_factory=lambda: [0.0] * 7)
    right_state: list[float] = field(default_factory=lambda: [0.0] * 7)
    ages_ms: dict[str, float | None] = field(default_factory=dict)
    counts: dict[str, int] = field(default_factory=dict)
    xr_live: bool = False
    action_live: bool = False


class BridgeClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 0.2,
        max_retries: int = 1,
        retry_backoff_s: float = 0.03,
    ):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.max_retries = max(0, int(max_retries))
        self.retry_backoff_s = max(0.0, float(retry_backoff_s))
        self._sock: socket.socket | None = None
        self._file = None
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._file:
            try:
                self._file.close()
            except OSError:
                pass
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._file = None

    def _connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self._sock = sock
        self._file = sock.makefile("rwb", buffering=0)

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            last_exc: Exception | None = None
            for attempt in range(self.max_retries + 1):
                if self._sock is None or self._file is None:
                    self._connect()
                assert self._file is not None
                try:
                    self._file.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                    line = self._file.readline()
                    if not line:
                        raise ConnectionError("bridge closed connection")
                    return json.loads(line.decode("utf-8"))
                except Exception as exc:
                    last_exc = exc
                    self.close()
                    if attempt < self.max_retries and self.retry_backoff_s > 0:
                        time.sleep(self.retry_backoff_s)
            assert last_exc is not None
            raise last_exc

    def health(self) -> dict[str, Any]:
        return self.request({"cmd": "health"})

    def snapshot(self, frame_time_ns: int | None = None) -> dict[str, Any]:
        if frame_time_ns is None:
            frame_time_ns = time.monotonic_ns()
        return self.request({"cmd": "snapshot", "frame_time_ns": int(frame_time_ns)})


class TelemetryMonitor:
    def __init__(self):
        self.client = BridgeClient()
        self.samples: deque[TelemetrySample] = deque(maxlen=600)
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="telemetry-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.client.close()

    def latest(self) -> TelemetrySample | None:
        with self._lock:
            return self.samples[-1] if self.samples else None

    def history(self) -> list[TelemetrySample]:
        with self._lock:
            return list(self.samples)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self._read_sample()
                with self._lock:
                    self.samples.append(sample)
                self.last_error = None
            except Exception as exc:
                self.last_error = str(exc)
                self.client.close()
            time.sleep(0.1)

    def _read_sample(self) -> TelemetrySample:
        snap = self.client.snapshot(time.monotonic_ns())
        health: dict[str, Any] = {}
        try:
            health = self.client.health()
        except Exception:
            pass
        sources = snap.get("sources", {})
        ages = snap.get("ages_ms", {})
        left_state = _joint_plus_gripper(
            sources.get("left_feedback_joint_states"),
            sources.get("left_feedback_gripper_status"),
            gripper_key="width",
        )
        right_state = _joint_plus_gripper(
            sources.get("right_feedback_joint_states"),
            sources.get("right_feedback_gripper_status"),
            gripper_key="width",
        )
        left_action = _joint_plus_gripper(
            sources.get("left_control_move_j"),
            sources.get("left_control_joint_states"),
            gripper_key="position",
        )
        right_action = _joint_plus_gripper(
            sources.get("right_control_move_j"),
            sources.get("right_control_joint_states"),
            gripper_key="position",
        )
        xr_age = ages.get("xr_pose")
        left_action_age = ages.get("left_control_move_j")
        right_action_age = ages.get("right_control_move_j")
        return TelemetrySample(
            stamp_s=time.time(),
            bridge_ok=bool(snap.get("ok")),
            left_action=left_action,
            right_action=right_action,
            left_state=left_state,
            right_state=right_state,
            ages_ms=ages,
            counts=health.get("counts", {}) if health else {},
            xr_live=bool(xr_age is not None and xr_age < 500.0),
            action_live=bool(
                (left_action_age is not None and left_action_age < 500.0)
                or (right_action_age is not None and right_action_age < 500.0)
            ),
        )


def _sample_data(sample: dict[str, Any] | None) -> dict[str, Any]:
    if not sample or not sample.get("valid"):
        return {}
    data = sample.get("data")
    return data if isinstance(data, dict) else {}


def _joint_plus_gripper(
    joint_sample: dict[str, Any] | None,
    gripper_sample: dict[str, Any] | None,
    gripper_key: str,
) -> list[float]:
    out = [0.0] * 7
    joint_data = _sample_data(joint_sample)
    positions = joint_data.get("position") or []
    for i in range(6):
        out[i] = float(positions[i]) if i < len(positions) else 0.0

    gripper_data = _sample_data(gripper_sample)
    if gripper_key == "position":
        gpos = gripper_data.get("position") or []
        out[6] = float(gpos[0]) if gpos else 0.0
    else:
        out[6] = float(gripper_data.get(gripper_key, 0.0))
    return out
