from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from camera_config import CAMERA_LABELS, get_camera_paths


@dataclass
class CameraFrame:
    name: str
    image_rgb: np.ndarray
    stamp_s: float
    fps: float
    valid: bool
    error: str | None = None


class CameraPreviewManager:
    def __init__(self, width: int = 320, height: int = 240, fps: int = 20, stale_after_s: float = 2.0):
        self.width = width
        self.height = height
        self.fps = fps
        self.stale_after_s = stale_after_s
        self.frames: dict[str, CameraFrame] = {}
        self._threads: dict[str, threading.Thread] = {}
        self._stops: dict[str, threading.Event] = {}
        self._lock = threading.Lock()

    def start(self) -> None:
        for name, path in get_camera_paths().items():
            if name in self._threads and self._threads[name].is_alive():
                continue
            stop = threading.Event()
            self._stops[name] = stop
            thread = threading.Thread(target=self._run_camera, args=(name, path, stop), daemon=True)
            self._threads[name] = thread
            thread.start()

    def stop(self, clear: bool = False) -> None:
        for stop in self._stops.values():
            stop.set()
        for thread in self._threads.values():
            thread.join(timeout=3.0)
        self._threads.clear()
        self._stops.clear()
        if clear:
            with self._lock:
                self.frames.clear()

    def restart(self, clear: bool = True) -> None:
        self.stop(clear=clear)
        self.start()

    def latest(self, name: str) -> CameraFrame | None:
        with self._lock:
            frame = self.frames.get(name)
        if frame and frame.valid:
            age_s = time.time() - frame.stamp_s
            if age_s > self.stale_after_s:
                return CameraFrame(
                    name=frame.name,
                    image_rgb=frame.image_rgb,
                    stamp_s=frame.stamp_s,
                    fps=0.0,
                    valid=False,
                    error=f"stale frame age={age_s:.1f}s; camera preview is not updating",
                )
        return frame

    def all_latest(self) -> dict[str, CameraFrame]:
        return {name: self.latest(name) for name in list(self.frames)}

    def _run_camera(self, name: str, path: str, stop: threading.Event) -> None:
        last_times: list[float] = []
        while not stop.is_set():
            cap = _open_capture(path)
            if not cap.isOpened():
                self._store_error(name, f"failed to open {path}")
                cap.release()
                stop.wait(0.5)
                continue
            interval = 1.0 / max(self.fps, 1)
            read_failures = 0
            try:
                while not stop.is_set():
                    start = time.time()
                    ok, frame_bgr = cap.read()
                    now = time.time()
                    if not ok or frame_bgr is None:
                        read_failures += 1
                        self._store_error(name, f"read failed from {path}")
                        if read_failures >= 5:
                            break
                        stop.wait(0.05)
                        continue
                    read_failures = 0
                    frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    last_times.append(now)
                    last_times = [t for t in last_times if now - t <= 2.0]
                    fps = len(last_times) / max(now - last_times[0], 0.001) if len(last_times) > 1 else 0.0
                    with self._lock:
                        self.frames[name] = CameraFrame(name, frame_rgb, now, fps, True)
                    elapsed = time.time() - start
                    stop.wait(max(0.0, interval - elapsed))
            finally:
                cap.release()
            if not stop.is_set():
                stop.wait(0.5)

    def _store_error(self, name: str, error: str) -> None:
        blank = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        with self._lock:
            self.frames[name] = CameraFrame(name, blank, time.time(), 0.0, False, error)


def downscale_rgb(image: np.ndarray, width: int = 320, height: int = 240) -> np.ndarray:
    if image.shape[1] == width and image.shape[0] == height:
        return image
    return cv2.resize(image, (width, height))


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
