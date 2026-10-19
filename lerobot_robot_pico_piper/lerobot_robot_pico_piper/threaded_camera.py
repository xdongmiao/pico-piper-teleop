from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class CameraSample:
    recv_stamp_ns: int
    image_rgb: np.ndarray


class ThreadedOpenCVCamera:
    def __init__(
        self,
        name: str,
        path: str,
        width: int,
        height: int,
        fps: int,
        fourcc: str,
        ring_size: int,
    ):
        self.name = name
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self._ring: deque[CameraSample] = deque(maxlen=ring_size)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._last_error: str | None = None
        self._frames = 0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @property
    def frames(self) -> int:
        return self._frames

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"camera-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._connected = False

    def _run(self) -> None:
        while not self._stop.is_set():
            cap = self._open_capture()
            if not cap.isOpened():
                self._last_error = f"failed to open {self.path}"
                self._connected = False
                cap.release()
                self._stop.wait(0.25)
                continue
            self._connected = True
            read_failures = 0
            try:
                while not self._stop.is_set():
                    ok, frame_bgr = cap.read()
                    recv_ns = time.monotonic_ns()
                    if not ok or frame_bgr is None:
                        read_failures += 1
                        self._last_error = f"read failed from {self.path}"
                        if read_failures >= 10:
                            break
                        self._stop.wait(0.002)
                        continue
                    read_failures = 0
                    if frame_bgr.shape[1] != self.width or frame_bgr.shape[0] != self.height:
                        frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    with self._lock:
                        self._ring.append(CameraSample(recv_ns, frame_rgb))
                        self._frames += 1
            finally:
                cap.release()
                self._connected = False
            if not self._stop.is_set():
                self._stop.wait(0.25)

    def _open_capture(self) -> cv2.VideoCapture:
        last_cap = cv2.VideoCapture()
        for target in self._opencv_targets():
            cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            if cap.isOpened():
                return cap
            cap.release()
            last_cap = cap
        return last_cap

    def _opencv_targets(self) -> list[int | str]:
        targets: list[int | str] = [self.path]
        try:
            resolved = Path(self.path).resolve()
        except OSError:
            resolved = Path(self.path)
        resolved_str = str(resolved)
        if resolved_str not in targets:
            targets.append(resolved_str)
        name = resolved.name
        if name.startswith("video") and name[5:].isdigit():
            targets.append(int(name[5:]))
        return targets

    def sample_at(self, frame_time_ns: int) -> tuple[np.ndarray, float, bool]:
        with self._lock:
            samples = list(self._ring)
        if not samples:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8), 1_000_000.0, False
        chosen = min(samples, key=lambda sample: abs(frame_time_ns - sample.recv_stamp_ns))
        distance_ms = abs(frame_time_ns - chosen.recv_stamp_ns) / 1_000_000.0
        return chosen.image_rgb.copy(), float(distance_ms), True
