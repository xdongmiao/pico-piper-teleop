from __future__ import annotations

import time

import streamlit as st

from runtime_state import get_runtime
from camera_config import (
    CAMERA_LABELS,
    get_camera_bindings,
    list_video_candidates,
    load_camera_config,
    reset_camera_config,
    save_camera_config,
    v4l_available,
)


rt = get_runtime()
rt.ensure_background()

st.title("设备控制台")

if "last_device_message" not in st.session_state:
    st.session_state.last_device_message = ""
if "last_enable_steps" not in st.session_state:
    st.session_state.last_enable_steps = []


def _duplicate_process_messages(statuses) -> list[str]:
    messages = []
    for key in ["ik", "arms", "xr", "bridge"]:
        proc_status = statuses[key]
        instance_count = len(proc_status.external_pids) + (1 if proc_status.managed else 0)
        if instance_count > 1:
            messages.append(f"{proc_status.label}: {instance_count} 套进程组 {proc_status.external_pids}")
    return messages


def _status_text(ok: bool) -> str:
    return "正常" if ok else "异常"


def _age_ok(sample, key: str, limit_ms: float) -> bool:
    if sample is None:
        return False
    age = sample.ages_ms.get(key)
    return bool(age is not None and age < limit_ms)


def _fmt_age(sample, key: str) -> str:
    if sample is None:
        return "missing"
    age = sample.ages_ms.get(key)
    return "missing" if age is None else f"{age:.0f}ms"


def _scan_ok(scan_result, key: str) -> bool:
    return bool(scan_result and scan_result.checks.get(key) and scan_result.checks[key].ok)


def _preview_ok() -> bool:
    frames = [rt.preview.latest(name) for name in get_camera_bindings()]
    return bool(frames and all(frame and frame.valid and time.time() - frame.stamp_s < 2.0 for frame in frames))


def _summary_state(
    label: str,
    ok: bool,
    detail: str,
    *,
    warning: bool = False,
) -> tuple[str, str, str, str]:
    if ok:
        return label, "正常", detail, "normal"
    if warning:
        return label, "待确认", detail, "off"
    return label, "异常", detail, "inverse"


def _render_steps(title: str, steps: list[tuple[str, bool, str]]) -> None:
    if not steps:
        return
    with st.status(title, expanded=True, state="complete" if all(step[1] for step in steps) else "error"):
        for name, ok, msg in steps:
            icon = "OK" if ok else "FAIL"
            st.write(f"{icon} {name}: {msg}")


st.subheader("总控")
scan = rt.last_scan
statuses = rt.processes.all_statuses()
sample = rt.telemetry.latest()
duplicate_processes = _duplicate_process_messages(statuses)
enable_disabled = bool(duplicate_processes)
top_left, top_mid, top_right = st.columns([1, 1, 2])
with top_left:
    if st.button("启用设备", type="primary", width="stretch", disabled=enable_disabled):
        with st.status("正在启用设备...", expanded=True) as status:
            steps = rt.enable_devices()
            for name, ok, msg in steps:
                st.write(f"{'OK' if ok else 'FAIL'} {name}: {msg}")
            status.update(label="启用完成" if all(step[1] for step in steps) else "启用存在异常", state="complete" if all(step[1] for step in steps) else "error")
        st.session_state.last_enable_steps = steps
        st.rerun()
    if enable_disabled:
        st.caption("检测到重复控制进程，启用设备已暂停。请先执行下方“一键优化设备进程”。")
with top_mid:
    if st.button("设备失能", width="stretch"):
        steps = rt.disable_devices()
        st.session_state.last_enable_steps = steps
        st.session_state.last_device_message = "设备失能完成。外部终端启动的进程不会被 UI 强制停止。"
        st.rerun()
with top_right:
    st.info("日常流程：启用设备 -> 确认状态 -> 数据采集。单项启动和相机配置在下方高级区。")

if st.session_state.last_device_message:
    st.caption(st.session_state.last_device_message)
_render_steps("最近操作", st.session_state.last_enable_steps)

st.divider()

if duplicate_processes:
    st.error("检测到重复设备进程，采集前建议只保留一套： " + "；".join(duplicate_processes))
    st.caption("一键优化会停止重复的 Bridge、IK Solver、双臂控制进程，然后重新启动一套干净进程。执行期间机械臂控制链路会短暂中断。")
    confirm_optimize = st.checkbox("我确认当前没有正在采集，允许停止并重启 Bridge / IK / 双臂控制", key="confirm_process_optimize")
    optimize_disabled = (not confirm_optimize) or rt.recorder.is_active
    if rt.recorder.is_active:
        st.warning("当前正在采集或保存数据，不能优化设备进程。请先结束采集并等待保存完成。")
    elif not confirm_optimize:
        st.caption("勾选确认后才会启用一键优化按钮。")
    else:
        st.caption("确认已勾选，可以执行一键优化。")
    if st.button("一键优化设备进程", type="primary", width="stretch", disabled=optimize_disabled):
        with st.status("正在清理并重启设备进程...", expanded=True) as status:
            steps = rt.optimize_device_processes()
            for name, ok, msg in steps:
                st.write(f"{'OK' if ok else 'FAIL'} {name}: {msg}")
            status.update(
                label="进程优化完成" if all(step[1] for step in steps) else "进程优化存在异常",
                state="complete" if all(step[1] for step in steps) else "error",
            )
        st.session_state.last_enable_steps = steps
        st.rerun()
status_cols = st.columns(6)
bridge_proc_ok = statuses["bridge"].running
bridge_live = bool(sample and sample.bridge_ok and time.time() - sample.stamp_s < 1.0)
left_feedback_ok = _age_ok(sample, "left_feedback_joint_states", 500.0)
right_feedback_ok = _age_ok(sample, "right_feedback_joint_states", 500.0)
vr_input_ok = bool(sample and sample.xr_live)
semantic_action_ok = bool(sample and (sample.action_live or (left_feedback_ok and right_feedback_ok)))
cameras_live_ok = _preview_ok()
summary_items = [
    _summary_state(
        "CAN",
        _scan_ok(scan, "can_left") and _scan_ok(scan, "can_right"),
        "来自上次设备扫描" if scan else "尚未扫描 CAN",
        warning=scan is None,
    ),
    _summary_state(
        "Cameras",
        cameras_live_ok or all(_scan_ok(scan, f"camera_{name}") for name in get_camera_bindings()),
        "预览画面正常" if cameras_live_ok else ("来自上次设备扫描" if scan else "等待预览帧"),
        warning=not cameras_live_ok and scan is None,
    ),
    _summary_state(
        "Left Arm",
        bridge_live and left_feedback_ok,
        f"joint age={_fmt_age(sample, 'left_feedback_joint_states')}" if bridge_live else "Bridge 未连接，无法读取左臂状态",
        warning=bridge_proc_ok and not bridge_live,
    ),
    _summary_state(
        "Right Arm",
        bridge_live and right_feedback_ok,
        f"joint age={_fmt_age(sample, 'right_feedback_joint_states')}" if bridge_live else "Bridge 未连接，无法读取右臂状态",
        warning=bridge_proc_ok and not bridge_live,
    ),
    _summary_state(
        "Bridge",
        bridge_live,
        "遥测连接正常" if bridge_live else ("进程存在，等待遥测" if bridge_proc_ok else "Bridge 进程未运行"),
        warning=bridge_proc_ok and not bridge_live,
    ),
    _summary_state(
        "VR/Action",
        bridge_live and vr_input_ok and semantic_action_ok,
        (
            f"xr={_fmt_age(sample, 'xr_pose')}, "
            f"IK={'live' if sample and sample.action_live else 'hold'}, "
            f"semantic={'ok' if semantic_action_ok else 'missing'}"
        )
        if bridge_live
        else "Bridge 未连接，无法读取 VR/Action",
        warning=(bridge_proc_ok and not bridge_live) or (bridge_live and semantic_action_ok),
    ),
]
for col, (name, value, detail, delta_color) in zip(status_cols, summary_items, strict=False):
    col.metric(name, value, detail, delta_color=delta_color)

if not bridge_live:
    st.warning(
        "当前 UI 没有读到 LeRobot ROS Bridge 遥测。VR 仍可能可以直接控制机械臂，但页面无法确认左右臂、VR 和 Action 的实时状态。"
    )
    recover_col, note_col = st.columns([1, 2])
    with recover_col:
        if st.button("恢复 Bridge/VR 状态读取", type="primary", width="stretch"):
            ok, msg = rt.recover_vr_stack_if_needed(min_interval_s=0.0)
            st.session_state.last_device_message = msg
            st.toast("状态读取恢复命令已执行" if ok else "状态读取恢复失败")
            time.sleep(1.0)
            st.rerun()
    with note_col:
        st.caption("这个按钮只尝试启动 Bridge 和缺失的 VR 服务，不会停止正在工作的机械臂控制链路。")

main_left, main_right = st.columns([1, 1.45])

with main_left:
    st.subheader("设备扫描")
    if st.button("扫描设备", type="primary", width="stretch"):
        with st.status("Scanning devices...", expanded=True):
            scan = rt.scan_devices()
            st.write(f"Overall: {_status_text(scan.ok)}")
        st.rerun()
    scan = rt.last_scan
    if scan is None:
        st.warning("还没有扫描。点击“启用设备”或“扫描设备”。")
    else:
        for key, check in scan.checks.items():
            if check.ok:
                st.success(f"{key}: {check.details}")
            else:
                level = st.warning if key in {"vr", "left_action", "right_action"} else st.error
                level(f"{key}: {check.details}")

with main_right:
    st.subheader("三路相机预览")
    st.caption("预览线程目标为 20Hz，网页面板每 0.5 秒刷新一次；LeRobot 录制目标为 30Hz，目标频率不代表实际帧率。")
    preview_col1, preview_col2 = st.columns([1, 2])
    with preview_col1:
        if st.button("重启相机预览", width="stretch"):
            rt.preview.restart(clear=True)
            st.session_state.last_device_message = "相机预览已重启，旧缓存帧已清空。"
            time.sleep(0.5)
            st.rerun()
    with preview_col2:
        st.caption("如果画面 age 超过 2 秒，页面会判定为过期帧，不再把旧图当成实时画面。")

    def render_preview() -> None:
        cols = st.columns(3)
        for idx, (name, binding) in enumerate(get_camera_bindings().items()):
            frame = rt.preview.latest(name)
            with cols[idx]:
                st.caption(f"{CAMERA_LABELS[name]} ({name})")
                st.caption(f"{binding.effective_path} · {binding.mode}")
                if frame and frame.valid:
                    st.image(frame.image_rgb, channels="RGB", output_format="JPEG", width="stretch")
                    st.caption(f"preview_capture_fps={frame.fps:.1f}/20, recording_target_fps=30, ui_refresh_s=0.5, age={time.time() - frame.stamp_s:.1f}s")
                elif frame:
                    st.error(frame.error or "camera invalid")
                    st.caption("请先重启相机预览；如果仍失败，检查 USB Hub/线缆和相机稳定配置。")
                else:
                    st.warning("waiting for frame")

    if hasattr(st, "fragment"):
        @st.fragment(run_every=0.5)
        def preview_fragment() -> None:
            render_preview()

        preview_fragment()
    else:
        render_preview()

st.divider()

with st.expander("高级：相机稳定配置", expanded=False):
    bindings = get_camera_bindings()
    if v4l_available():
        st.success("容器可见 /dev/v4l，优先使用稳定 by-path/by-id。")
    else:
        st.warning("当前容器看不到 /dev/v4l，正在使用 fallback /dev/videoN。若要彻底稳定，请让容器挂载 /dev/v4l。")

    candidate_paths = list_video_candidates()
    current_config = load_camera_config()
    config_cols = st.columns(3)
    for idx, name in enumerate(["top", "left_wrist", "right_wrist"]):
        binding = bindings[name]
        with config_cols[idx]:
            st.caption(f"{CAMERA_LABELS[name]} ({name})")
            st.code(
                f"effective: {binding.effective_path}\n"
                f"mode: {binding.mode}\n"
                f"serial: {binding.serial}\n"
                f"usb: {binding.usb_port}"
            )
            mode_options = ["auto", "manual"]
            current_mode = current_config[name].get("mode", "auto")
            selected_mode = st.selectbox(
                "Resolve mode",
                options=mode_options,
                index=mode_options.index(current_mode) if current_mode in mode_options else 0,
                key=f"camera_mode_{name}",
            )
            options = list(dict.fromkeys([binding.fallback_path, binding.stable_path, binding.by_id, *candidate_paths]))
            selected = st.selectbox(
                "Manual / fallback path",
                options=options,
                index=options.index(binding.fallback_path) if binding.fallback_path in options else 0,
                key=f"camera_fallback_{name}",
            )
            current_config[name]["mode"] = selected_mode
            current_config[name]["fallback_path"] = selected

    cfg_col1, cfg_col2 = st.columns(2)
    with cfg_col1:
        if st.button("保存相机配置", width="stretch"):
            save_camera_config(current_config)
            rt.preview.restart(clear=True)
            st.success("相机配置已保存并重新启动预览。")
            st.rerun()
    with cfg_col2:
        if st.button("恢复默认 by-path 配置", width="stretch"):
            reset_camera_config()
            rt.preview.restart(clear=True)
            st.success("已恢复默认相机配置。")
            st.rerun()

with st.expander("高级：单项启动与停止", expanded=False):
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        if st.button("启动 CAN", width="stretch"):
            ok, msg = rt.processes.start_can()
            st.session_state.last_device_message = msg
            st.toast("CAN command finished" if ok else "CAN command failed")
    with col2:
        if st.button("启动 IK", width="stretch"):
            ok, msg = rt.processes.start("ik")
            st.session_state.last_device_message = msg
    with col3:
        if st.button("启动双臂", width="stretch"):
            ok, msg = rt.processes.start("arms")
            st.session_state.last_device_message = msg
    with col4:
        if st.button("启动 VR 服务", width="stretch"):
            ok, msg = rt.processes.start("xr")
            st.session_state.last_device_message = msg
    with col5:
        if st.button("启动 Bridge", width="stretch"):
            ok, msg = rt.processes.start("bridge")
            st.session_state.last_device_message = msg

    st.subheader("进程状态")
    proc_cols = st.columns(4)
    for i, status in enumerate(rt.processes.all_statuses().values()):
        with proc_cols[i % 4]:
            st.metric(status.label, "运行" if status.running else "停止")
            if status.pid:
                st.caption(f"UI pid: {status.pid}")
            if status.external_pids:
                st.caption(f"External pid: {status.external_pids}")
            if status.started_at:
                st.caption(f"Started: {time.strftime('%H:%M:%S', time.localtime(status.started_at))}")
            if st.button(f"停止 {status.label}", key=f"stop_{status.key}", width="stretch"):
                ok, msg = rt.processes.stop(status.key)
                st.session_state.last_device_message = msg
                st.rerun()

with st.expander("高级：日志", expanded=False):
    st.caption("CAN / one-shot logs")
    st.code("\n".join(rt.processes.one_shot_logs) or "No logs yet.")
    for key, status in rt.processes.all_statuses().items():
        st.caption(status.label)
        st.code("\n".join(status.logs[-80:]) or "No logs yet.")
