from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any


class BridgeClient:
    def __init__(
        self,
        host: str,
        port: int,
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

    def connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self._sock = sock
        self._file = sock.makefile("rwb", buffering=0)

    def close(self) -> None:
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._file = None
        self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None and self._file is not None

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            last_exc: Exception | None = None
            for attempt in range(self.max_retries + 1):
                if not self.is_connected:
                    self.connect()
                assert self._file is not None
                try:
                    self._file.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                    line = self._file.readline()
                    if not line:
                        raise ConnectionError("bridge closed the connection")
                    return json.loads(line.decode("utf-8"))
                except Exception as exc:
                    last_exc = exc
                    self.close()
                    if attempt < self.max_retries and self.retry_backoff_s > 0:
                        time.sleep(self.retry_backoff_s)
            assert last_exc is not None
            raise last_exc

    def ping(self) -> dict[str, Any]:
        return self.request({"cmd": "ping", "client_monotonic_ns": time.monotonic_ns()})

    def snapshot(self, frame_time_ns: int) -> dict[str, Any]:
        return self.request({"cmd": "snapshot", "frame_time_ns": int(frame_time_ns)})
