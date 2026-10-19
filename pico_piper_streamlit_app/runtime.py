from __future__ import annotations

import time
from dataclasses import dataclass, field

from camera_config import get_camera_bindings
from camera_preview import CameraPreviewManager
from device_scan import ScanResult, scan_all
from process_manager import ProcessManager
from recorder import RecorderController
from telemetry import TelemetryMonitor


@dataclass
class AppRuntime:
    processes: ProcessManager
    preview: CameraPreviewManager
    telemetry: TelemetryMonitor
    recorder: RecorderController
    last_scan: ScanResult | None = None
    background_enabled: bool = True
    last_vr_recovery_s: float = 0.0
    recovery_logs: list[str] = field(default_factory=list)

    def enable_devices(self) -> list[tuple[str, bool, str]]:
        self.background_enabled = True
        self.ensure_background()
        steps: list[tuple[str, bool, str]] = []
        bindings = get_camera_bindings()
        camera_summary = ", ".join(f"{name}={binding.effective_path}({binding.mode})" for name, binding in bindings.items())
        steps.append(("相机配置", True, camera_summary))

        duplicate_messages = []
        for key in ["bridge", "ik", "arms"]:
            status = self.processes.status(key)
            instance_count = len(status.external_pids) + (1 if status.managed else 0)
            if instance_count > 1:
                duplicate_messages.append(f"{status.label}: {instance_count} process groups {status.external_pids}")
        if duplicate_messages:
            steps.append(
                (
                    "重复进程检查",
                    False,
                    "启用设备不会清理已经重复的控制链路。请先在本页勾选确认后执行“一键优化设备进程”："
                    + "；".join(duplicate_messages),
                )
            )
            return steps

        ok, msg = self.processes.start_can()
        steps.append(("CAN", ok, msg))
        if not ok:
            return steps

        for key, label in [("ik", "IK Solver"), ("arms", "双臂控制"), ("xr", "VR 服务"), ("bridge", "Bridge")]:
            ok, msg = self.processes.start(key)
            steps.append((label, ok, msg))
            if not ok:
                return steps

        time.sleep(2.0)
        steps.append(("启动等待", True, "Waited 2s for ROS topics, bridge and cameras to warm up."))
        scan = self.scan_devices()
        steps.append(("设备扫描", scan.ok, "关键设备正常" if scan.ok else "存在异常，请查看扫描详情"))
        return steps

    def disable_devices(self) -> list[tuple[str, bool, str]]:
        steps: list[tuple[str, bool, str]] = []
        self.background_enabled = False
        self.preview.stop(clear=True)
        steps.append(("相机预览", True, "Preview stopped."))
        self.telemetry.stop()
        steps.append(("Telemetry", True, "Telemetry monitor stopped."))
        for key, label in [("bridge", "Bridge"), ("xr", "VR 服务"), ("ik", "IK Solver"), ("arms", "双臂控制")]:
            ok, msg = self.processes.stop(key)
            if not ok and "external" in msg:
                steps.append((label, True, f"External process left running: {msg}"))
            else:
                steps.append((label, ok, msg))
        self.last_scan = None
        return steps

    def optimize_device_processes(self) -> list[tuple[str, bool, str]]:
        steps: list[tuple[str, bool, str]] = []
        if self.recorder.is_active:
            return [("采集状态", False, "Recorder is active. Stop or finish recording before optimizing device processes.")]

        self.background_enabled = True
        self.preview.stop(clear=True)
        steps.append(("相机预览", True, "Preview stopped before process cleanup."))
        self.telemetry.stop()
        steps.append(("Telemetry", True, "Telemetry stopped before process cleanup."))

        for key, label in [("bridge", "Bridge"), ("ik", "IK Solver"), ("arms", "双臂控制")]:
            ok, msg = self.processes.force_stop(key)
            steps.append((f"清理 {label}", ok, msg))
            if not ok:
                self.last_scan = None
                return steps

        ok, msg = self.processes.start_can()
        steps.append(("CAN", ok, msg))
        if not ok:
            self.last_scan = None
            return steps

        for key, label in [("ik", "IK Solver"), ("arms", "双臂控制"), ("bridge", "Bridge")]:
            ok, msg = self.processes.start(key)
            steps.append((f"启动 {label}", ok, msg))
            if not ok:
                self.last_scan = None
                return steps

        time.sleep(2.0)
        steps.append(("启动等待", True, "Waited 2s for ROS topics and bridge to warm up."))
        self.telemetry.start()
        if not self.recorder.is_active:
            self.preview.start()
        scan = self.scan_devices()
        steps.append(("设备扫描", scan.ok, "关键设备正常" if scan.ok else "存在异常，请查看扫描详情"))
        return steps

    def scan_devices(self) -> ScanResult:
        preview_was_running = bool(self.preview._threads)
        if preview_was_running:
            self.preview.stop(clear=True)
            time.sleep(0.5)
        result = scan_all()
        self.last_scan = result
        if preview_was_running and not self.recorder.is_active:
            self.preview.start()
        return result

    def ensure_background(self) -> None:
        if not self.background_enabled:
            return
        self.telemetry.start()
        if not self.recorder.is_active:
            self.preview.start()

    def recover_vr_stack_if_needed(self, min_interval_s: float = 5.0) -> tuple[bool, str]:
        now = time.time()
        if now - self.last_vr_recovery_s < min_interval_s:
            return False, "recovery check throttled"
        self.last_vr_recovery_s = now

        sample = self.telemetry.latest()
        messages: list[str] = []
        if sample is None or not sample.bridge_ok:
            ok, msg = self.processes.start("bridge")
            messages.append(f"Bridge: {msg}")
            if not ok:
                self._append_recovery_log(messages[-1])
                return False, "; ".join(messages)

        if sample is None or not sample.xr_live:
            xr_status = self.processes.status("xr")
            if not xr_status.running:
                ok, msg = self.processes.start("xr")
                messages.append(f"VR service: {msg}")
                if not ok:
                    self._append_recovery_log(messages[-1])
                    return False, "; ".join(messages)

        if not messages:
            return True, "VR/bridge recovery not needed."
        message = "; ".join(messages)
        self._append_recovery_log(message)
        return True, message

    def _append_recovery_log(self, message: str) -> None:
        self.recovery_logs.append(f"{time.strftime('%H:%M:%S')} {message}")
        del self.recovery_logs[:-20]

    def shutdown(self) -> None:
        self.background_enabled = False
        self.preview.stop(clear=True)
        self.telemetry.stop()
        if self.recorder.is_active:
            self.recorder.stop()


def create_runtime() -> AppRuntime:
    runtime = AppRuntime(
        processes=ProcessManager(),
        preview=CameraPreviewManager(),
        telemetry=TelemetryMonitor(),
        recorder=RecorderController(),
    )
    runtime.ensure_background()
    return runtime
