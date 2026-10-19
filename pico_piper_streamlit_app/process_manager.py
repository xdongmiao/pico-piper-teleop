from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ProcessSpec:
    key: str
    label: str
    command: str
    cwd: str = "/projects"
    pattern: str | None = None
    env: dict[str, str] = field(default_factory=dict)


@dataclass
class ProcessStatus:
    key: str
    label: str
    running: bool
    managed: bool
    pid: int | None
    external_pids: list[int]
    started_at: float | None
    returncode: int | None
    logs: list[str]


class ManagedProcess:
    def __init__(self, spec: ProcessSpec):
        self.spec = spec
        self.process: subprocess.Popen[str] | None = None
        self.started_at: float | None = None
        self.logs: deque[str] = deque(maxlen=300)
        self._reader: threading.Thread | None = None

    def start(self) -> None:
        if self.is_running:
            return
        env = os.environ.copy()
        env.update(self.spec.env)
        self.process = subprocess.Popen(
            ["bash", "-lc", self.spec.command],
            cwd=self.spec.cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        self.started_at = time.time()
        self.logs.append(f"$ {self.spec.command}")
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(self, timeout_s: float = 5.0) -> None:
        if not self.process:
            return
        if self.process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=timeout_s)
        except Exception:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception:
                pass

    def _read_output(self) -> None:
        if not self.process or not self.process.stdout:
            return
        for line in self.process.stdout:
            self.logs.append(line.rstrip())


class ProcessManager:
    def __init__(self):
        self.specs: dict[str, ProcessSpec] = {
            "ik": ProcessSpec(
                key="ik",
                label="IK Solver",
                command="source /projects/pico_tele_agx/install/setup.bash && ros2 run ik_solver ik_solver",
                pattern="ik_solver/ik_solver|ros2 run ik_solver ik_solver",
            ),
            "arms": ProcessSpec(
                key="arms",
                label="Dual Piper Arms",
                command=(
                    "source /projects/pico_tele_agx/install/setup.bash && "
                    "ros2 launch agx_arm_ctrl start_dual_agx_arm.launch.py "
                    "left_can_port:=can_left right_can_port:=can_right arm_type:=piper "
                    "left_effector_type:=agx_gripper right_effector_type:=agx_gripper "
                    "tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'"
                ),
                pattern="start_dual_agx_arm.launch.py|agx_arm_ctrl_single",
            ),
            "xr": ProcessSpec(
                key="xr",
                label="XR Service",
                command=(
                    "export LD_LIBRARY_PATH=/projects/third_party/libstdcxx_jammy_ppa/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && "
                    "bash /opt/apps/roboticsservice/runService.sh"
                ),
                pattern="runService.sh|roboticsservice|RoboticsServiceProcess",
            ),
            "bridge": ProcessSpec(
                key="bridge",
                label="LeRobot ROS Bridge",
                command=(
                    "source /opt/ros/humble/setup.bash && "
                    "source /projects/pico_tele_agx/install/setup.bash && "
                    "exec /usr/bin/python3 /projects/lerobot_robot_pico_piper/scripts/ros2_pico_piper_bridge.py "
                    "--host 127.0.0.1 --port 8765 --ring-size 900"
                ),
                pattern="ros2_pico_piper_bridge.py",
            ),
        }
        self._managed: dict[str, ManagedProcess] = {key: ManagedProcess(spec) for key, spec in self.specs.items()}
        self._lock = threading.Lock()
        self.one_shot_logs: deque[str] = deque(maxlen=300)

    def start(self, key: str) -> tuple[bool, str]:
        with self._lock:
            if key not in self._managed:
                return False, f"Unknown process key: {key}"
            spec = self.specs[key]
            external = self._external_pids(spec)
            managed = self._managed[key]
            if managed.is_running:
                return True, f"{spec.label} is already running under UI management."
            if len(external) > 1:
                return False, f"{spec.label} has multiple process groups running: {external}. Use one-click process optimization first."
            if external:
                return True, f"{spec.label} is already running externally: {external}"
            managed.start()
            time.sleep(0.3)
            if not managed.is_running:
                recent_logs = "; ".join(list(managed.logs)[-5:])
                return False, (
                    f"{spec.label} exited immediately with code "
                    f"{managed.process.poll() if managed.process else 'unknown'}"
                    + (f". Logs: {recent_logs}" if recent_logs else ".")
                )
            return True, f"Started {spec.label}."

    def stop(self, key: str) -> tuple[bool, str]:
        with self._lock:
            if key not in self._managed:
                return False, f"Unknown process key: {key}"
            managed = self._managed[key]
            if managed.is_running:
                managed.stop()
                return True, f"Stopped {managed.spec.label}."
            external = self._external_pids(managed.spec)
            if external:
                return False, f"{managed.spec.label} is external ({external}); stop it from its owning terminal."
            return True, f"{managed.spec.label} is not running."

    def force_stop(self, key: str, timeout_s: float = 5.0) -> tuple[bool, str]:
        with self._lock:
            if key not in self._managed:
                return False, f"Unknown process key: {key}"
            managed = self._managed[key]
            stopped: list[str] = []
            if managed.is_running:
                managed.stop(timeout_s=timeout_s)
                stopped.append(f"managed pid {managed.process.pid if managed.process else 'unknown'}")

            external = self._external_pids(managed.spec)
            pgids = self._pgids_for_pids(external)
            for pgid in pgids:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                    stopped.append(f"external pgid {pgid}")
                except ProcessLookupError:
                    pass
                except Exception as exc:
                    self.one_shot_logs.append(f"Failed SIGTERM {managed.spec.label} pgid {pgid}: {exc}")

            deadline = time.time() + timeout_s
            while time.time() < deadline and any(self._pid_alive(pid) for pid in external):
                time.sleep(0.1)

            remaining = [pid for pid in self._external_pids(managed.spec) if self._pid_alive(pid)]
            for pgid in self._pgids_for_pids(remaining):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    stopped.append(f"SIGKILL pgid {pgid}")
                except ProcessLookupError:
                    pass
                except Exception as exc:
                    self.one_shot_logs.append(f"Failed SIGKILL {managed.spec.label} pgid {pgid}: {exc}")

            time.sleep(0.5)
            still_running = self._external_pids(managed.spec)
            if still_running:
                return False, f"{managed.spec.label} still running after cleanup: {still_running}"
            return True, f"Stopped {managed.spec.label}: {', '.join(stopped) if stopped else 'nothing to stop'}."

    def start_can(self) -> tuple[bool, str]:
        if self._can_interfaces_ok():
            return True, "CAN already active: can_left/can_right are UP, ERROR-ACTIVE, bitrate 1000000."
        cmd = "bash /projects/pico_tele_agx/src/agx_arm_ros/scripts/can_muti_activate.sh --ignore"
        result = subprocess.run(
            ["bash", "-lc", cmd],
            cwd="/projects",
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=30,
        )
        lines = [f"$ {cmd}", *result.stdout.splitlines()]
        self.one_shot_logs.extend(lines)
        if result.returncode != 0 and self._can_interfaces_ok():
            return True, "CAN activation script reported an error, but can_left/can_right are already active."
        return result.returncode == 0, result.stdout.strip() or f"exit={result.returncode}"

    def status(self, key: str) -> ProcessStatus:
        managed = self._managed[key]
        proc = managed.process
        external = self._external_pids(managed.spec)
        return ProcessStatus(
            key=key,
            label=managed.spec.label,
            running=managed.is_running or bool(external),
            managed=managed.is_running,
            pid=proc.pid if proc else None,
            external_pids=external,
            started_at=managed.started_at,
            returncode=proc.poll() if proc else None,
            logs=list(managed.logs),
        )

    def all_statuses(self) -> dict[str, ProcessStatus]:
        return {key: self.status(key) for key in self.specs}

    def _external_pids(self, spec: ProcessSpec) -> list[int]:
        if not spec.pattern:
            return []
        cmd = "ps -eo pid=,args="
        result = subprocess.run(["bash", "-lc", cmd], text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        pids_by_pgid: dict[int, int] = {}
        managed_pid = self._managed.get(spec.key).process.pid if spec.key in self._managed and self._managed[spec.key].process else None
        managed_pgid: int | None = None
        if managed_pid is not None:
            try:
                managed_pgid = os.getpgid(managed_pid)
            except ProcessLookupError:
                managed_pgid = None
        import re

        pattern = re.compile(spec.pattern)
        for line in result.stdout.splitlines():
            raw_pid, _, args = line.strip().partition(" ")
            if not raw_pid or not args:
                continue
            if "ps -eo pid=,args=" in args or "grep -E" in args:
                continue
            parts = args.split(maxsplit=2)
            exe = Path(parts[0]).name if parts else ""
            if exe in {"bash", "sh"} and len(parts) > 1 and parts[1] in {"-c", "-lc"}:
                continue
            if not pattern.search(args):
                continue
            try:
                pid = int(raw_pid)
            except ValueError:
                continue
            if pid == os.getpid() or pid == managed_pid:
                continue
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                continue
            if pgid == managed_pgid:
                continue
            pids_by_pgid.setdefault(pgid, pid)
        return [pids_by_pgid[pgid] for pgid in sorted(pids_by_pgid)]

    def _pgids_for_pids(self, pids: list[int]) -> list[int]:
        pgids: set[int] = set()
        current_pgid = os.getpgid(os.getpid())
        for pid in pids:
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                continue
            if pgid != current_pgid:
                pgids.add(pgid)
        return sorted(pgids)

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _can_interfaces_ok(self) -> bool:
        result = subprocess.run(
            ["bash", "-lc", "ip -details link show can_left 2>/dev/null; ip -details link show can_right 2>/dev/null"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        text = result.stdout
        for iface in ["can_left", "can_right"]:
            if iface not in text:
                return False
        return text.count("ERROR-ACTIVE") >= 2 and text.count("bitrate 1000000") >= 2 and text.count("UP") >= 2


def file_mtime(path: str | Path) -> float | None:
    try:
        return Path(path).stat().st_mtime
    except OSError:
        return None
