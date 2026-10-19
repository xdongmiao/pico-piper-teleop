from __future__ import annotations

from threading import Lock
from typing import Any

_lock = Lock()
_frame_time_ns: int | None = None
_snapshot: dict[str, Any] | None = None


def set_frame_context(frame_time_ns: int, snapshot: dict[str, Any] | None) -> None:
    global _frame_time_ns, _snapshot
    with _lock:
        _frame_time_ns = frame_time_ns
        _snapshot = snapshot


def get_frame_context() -> tuple[int | None, dict[str, Any] | None]:
    with _lock:
        return _frame_time_ns, _snapshot

