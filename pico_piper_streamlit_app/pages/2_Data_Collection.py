from __future__ import annotations

from pathlib import Path

import pandas as pd
import streamlit as st

from runtime_state import get_runtime
from camera_config import CAMERA_LABELS, get_camera_bindings
from recorder import RecordSettings, check_dataset_name
from telemetry import ACTION_NAMES


rt = get_runtime()
rt.ensure_background()

st.title("数据采集")

DEFAULT_ROOT = "/projects/lerobot/NX/data"
TERMINAL_STATES = {"IDLE", "DONE", "ERROR"}
DATA_SAVE_STATES = {"SAVING", "DISCARDING", "FINALIZING", "CLEANING_UP"}
RECORDING_FPS = 30

status = rt.recorder.status()
can_start_new_stage = rt.recorder.can_start_new_stage
is_running = not can_start_new_stage
active_dataset_path = Path(status.dataset_path) if is_running and status.dataset_path else None
default_root = str(active_dataset_path.parent) if active_dataset_path else DEFAULT_ROOT
default_dataset_name = active_dataset_path.name if active_dataset_path else "pico_piper_task_001"
default_task = status.task if is_running and status.task else "pico vr teleoperation data collection"

with st.form("record_config"):
    st.subheader("阶段采集配置")
    root = st.text_input("保存根目录", value=default_root, disabled=is_running)
    dataset_name = st.text_input("Dataset name", value=default_dataset_name, disabled=is_running)
    task = st.text_input("Task", value=default_task, disabled=is_running)
    c1, c2 = st.columns(2)
    with c1:
        episode_time_s = st.number_input("单条时间(s)", min_value=0.5, max_value=3600.0, value=30.0, step=0.5)
    with c2:
        reset_time_s = st.number_input("重置时间(s)", min_value=0.0, max_value=3600.0, value=10.0, step=0.5)
    st.caption("一个阶段对应一个 LeRobot Dataset；阶段内可以连续采集多条 episode，每条由人工确认保存或丢弃。")
    submitted = st.form_submit_button("开始阶段采集", type="primary", disabled=not can_start_new_stage)

ok_name, msg_name, suggestion = check_dataset_name(root, dataset_name)
if status.state in DATA_SAVE_STATES:
    st.warning("当前阶段正在保存/丢弃/编码视频/写入 meta，请等待状态变为 DONE 后再开始下一个阶段采集。")
elif status.state == "DONE":
    st.success("上一阶段数据已经保存完成，可以开始新的阶段采集。")
elif status.state == "ERROR":
    st.error("上一阶段采集异常结束。请先查看错误和数据目录，再决定是否开始新阶段。")

if can_start_new_stage:
    if ok_name:
        st.success(msg_name)
    else:
        st.error(msg_name)
        if suggestion:
            st.info(f"建议名称: {suggestion}")
else:
    st.info(
        f"当前阶段尚未完全结束，状态={status.state}。"
        f" 当前 Dataset: {status.dataset_path}。必须等待保存完成后才能开始新的阶段采集。"
    )

if submitted:
    if not rt.recorder.can_start_new_stage:
        latest = rt.recorder.status()
        st.error(f"当前阶段还没有完全保存完成，状态={latest.state}。请等待 DONE 后再开始新阶段。")
    elif not ok_name:
        st.error("Dataset 路径重名，已阻止开始。")
    else:
        scan = rt.scan_devices()
        if not scan.ok:
            st.error("关键设备未全部正常，已阻止开始。先回首页检查 CAN、三相机、左右臂和 bridge。")
        else:
            rt.preview.stop()
            settings = RecordSettings(
                dataset_root=root,
                dataset_name=dataset_name,
                repo_id=f"zme/{dataset_name}",
                task=task,
                episode_time_s=float(episode_time_s),
                reset_time_s=float(reset_time_s),
            )
            ok, message = rt.recorder.start(settings)
            if ok:
                st.session_state.last_dataset_path = str(Path(root) / dataset_name)
                st.success(message)
                st.rerun()
            else:
                st.error(message)

status = rt.recorder.status()

st.divider()
control_cols = st.columns([1.4, 1.4, 1.2, 1.1])
with control_cols[0]:
    if st.button("结束当前条并进入确认", width="stretch", disabled=status.state != "RECORDING"):
        rt.recorder.finish_episode()
        st.rerun()
with control_cols[1]:
    if st.button("丢弃当前条重采", width="stretch", disabled=status.state != "RECORDING"):
        rt.recorder.rerecord_episode()
        st.rerun()
with control_cols[2]:
    finish_disabled = status.state in TERMINAL_STATES or status.state == "AWAIT_SAVE_DECISION"
    if st.button("结束阶段采集", width="stretch", disabled=finish_disabled):
        rt.recorder.request_finish_stage()
        st.rerun()
with control_cols[3]:
    if st.button("机械臂复位", width="stretch"):
        ok, msg = rt.recorder.arm_home()
        if ok:
            st.success(msg)
        else:
            st.warning(msg)


def _quality_label(value: float | None, warn_ms: float = 100.0) -> str:
    if value is None:
        return "missing"
    if value <= warn_ms:
        return f"{value:.1f} ms 正常"
    return f"{value:.1f} ms 偏高"


def _valid_label(value: float | None) -> str:
    return "正常" if value and value > 0.0 else "异常"


def _ratio_label(value: float | None) -> str:
    if value is None:
        return "0.0%"
    return f"{max(0.0, min(1.0, value)) * 100.0:.1f}%"


def _max_age_label(value: float | None) -> str:
    if value is None:
        return "max missing"
    if value >= 999_999.0:
        return "max missing"
    return f"max {value:.1f} ms"


def _quality_ratio(quality: dict[str, float], key: str) -> float:
    return max(0.0, min(1.0, float(quality.get(f"{key}_ratio", 0.0))))


def _quality_verdict(quality: dict[str, float]) -> tuple[str, str]:
    frames = quality.get("frames", 0.0)
    if frames <= 0:
        return "error", "本条没有统计到录制帧，建议重采。"
    errors = []
    warnings = []
    if _quality_ratio(quality, "bridge_valid") < 0.99:
        errors.append("Bridge 有效帧低于 99%")
    if _quality_ratio(quality, "vr_valid") < 0.95:
        errors.append("VR 有效帧低于 95%")
    for key, label in [
        ("cam_top_valid", "顶部相机"),
        ("cam_left_wrist_valid", "左腕相机"),
        ("cam_right_wrist_valid", "右腕相机"),
    ]:
        if _quality_ratio(quality, key) < 0.999:
            errors.append(f"{label}存在无效帧")
    if _quality_ratio(quality, "action_semantic_valid") < 0.999:
        errors.append("语义 action 不完整，IK 和 hold 都不可用")

    if quality.get("max_schedule_lag_ms", 0.0) > 50.0:
        warnings.append("调度延迟峰值超过 50ms")
    for key, label in [
        ("max_cam_top_age_ms", "顶部相机"),
        ("max_cam_left_wrist_age_ms", "左腕相机"),
        ("max_cam_right_wrist_age_ms", "右腕相机"),
    ]:
        if quality.get(key, 0.0) > 80.0:
            warnings.append(f"{label}帧年龄峰值超过 80ms")
    hold_ratio = max(_quality_ratio(quality, "left_action_from_hold"), _quality_ratio(quality, "right_action_from_hold"))
    if hold_ratio > 0.70:
        warnings.append("hold action 占比较高，本条主要是等待/静止动作")

    if errors:
        return "error", "；".join(errors)
    if warnings:
        return "warning", "；".join(warnings)
    return "success", "本条关键设备、时间轴和语义 action 正常，可以保存。"


def render_decision_panel(status) -> None:
    obs = status.latest_obs
    quality = status.pending_episode_quality
    with st.container(border=True):
        st.markdown("### 当前条待确认")
        st.caption("当前条采集和环境重置已经完成。保存后会写入当前阶段的同一个 Dataset；继续采集会追加下一条 episode。")

        p1, p2, p3, p4 = st.columns(4)
        p1.metric("Episode", status.current_episode)
        p2.metric("采集时长", f"{status.pending_episode_elapsed_s:.1f}s")
        p3.metric("预估帧数", status.pending_episode_estimated_frames)
        p4.metric("已保存", status.saved_episodes)

        st.markdown("#### 本条录制质量")
        verdict, verdict_msg = _quality_verdict(quality)
        if verdict == "success":
            st.success(verdict_msg)
        elif verdict == "warning":
            st.warning(verdict_msg)
        else:
            st.error(verdict_msg)
        st.caption("这里统计的是刚刚这条 episode 录制期间的质量，不受重置时间和等待确认时间影响。松开手柄时 action 会写成当前机械臂 hold 目标，避免旧 IK 命令污染数据。")
        q1, q2, q3, q4 = st.columns(4)
        q1.metric("VR 有效帧", _ratio_label(quality.get("vr_valid_ratio")), _max_age_label(quality.get("max_xr_age_ms")))
        q2.metric(
            "Left action 语义有效",
            _ratio_label(quality.get("left_action_semantic_valid_ratio")),
            f"IK {_ratio_label(quality.get('left_action_from_ik_ratio'))} / Hold {_ratio_label(quality.get('left_action_from_hold_ratio'))}",
        )
        q3.metric(
            "Right action 语义有效",
            _ratio_label(quality.get("right_action_semantic_valid_ratio")),
            f"IK {_ratio_label(quality.get('right_action_from_ik_ratio'))} / Hold {_ratio_label(quality.get('right_action_from_hold_ratio'))}",
        )
        q4.metric("Bridge 有效帧", _ratio_label(quality.get("bridge_valid_ratio")), f"lost max {quality.get('max_bridge_lost_duration_ms', 0.0):.0f} ms")

        c1, c2, c3, c4 = st.columns(4)
        c1.metric("统计帧数", f"{quality.get('frames', 0.0):.0f}")
        c2.metric("顶部/左腕/右腕相机", f"{_ratio_label(quality.get('cam_top_valid_ratio'))} / {_ratio_label(quality.get('cam_left_wrist_valid_ratio'))} / {_ratio_label(quality.get('cam_right_wrist_valid_ratio'))}")
        c3.metric("当前 Left action", _valid_label(obs.get("left_action_semantic_valid")), _quality_label(obs.get("left_action_age_ms"), warn_ms=500.0))
        c4.metric("当前 Right action", _valid_label(obs.get("right_action_semantic_valid")), _quality_label(obs.get("right_action_age_ms"), warn_ms=500.0))

        st.markdown("#### 操作")
        if status.finish_requested:
            a, b = st.columns(2)
            with a:
                if st.button("保存并结束当前阶段", type="primary", width="stretch"):
                    rt.recorder.save_and_finish()
                    st.rerun()
            with b:
                if st.button("丢弃并结束当前阶段", width="stretch"):
                    rt.recorder.discard_and_finish()
                    st.rerun()
        else:
            save_a, save_b = st.columns(2)
            with save_a:
                if st.button("保存并继续采集下一条", type="primary", width="stretch"):
                    rt.recorder.save_and_continue()
                    st.rerun()
            with save_b:
                if st.button("保存并结束当前阶段", width="stretch"):
                    rt.recorder.save_and_finish()
                    st.rerun()

            discard_a, discard_b = st.columns(2)
            with discard_a:
                if st.button("丢弃并重采当前条", width="stretch"):
                    rt.recorder.discard_and_retake()
                    st.rerun()
            with discard_b:
                if st.button("丢弃并结束当前阶段", width="stretch"):
                    rt.recorder.discard_and_finish()
                    st.rerun()


def render_live() -> None:
    status = rt.recorder.status()
    recovered, recovery_msg = (False, "采集未运行")
    if status.state not in TERMINAL_STATES:
        recovered, recovery_msg = rt.recover_vr_stack_if_needed()

    if status.state == "AWAIT_SAVE_DECISION":
        render_decision_panel(status)
        st.divider()

    st.subheader("实时状态")
    m1, m2, m3, m4, m5 = st.columns(5)
    m1.metric("State", status.state)
    m2.metric("当前条", status.current_episode)
    m3.metric("已保存条数", status.saved_episodes)
    m4.metric("Elapsed", f"{status.episode_elapsed_s:.1f}/{status.episode_time_s:.1f}s")
    m5.metric("Reset left", f"{status.reset_remaining_s:.1f}s")

    if status.state in DATA_SAVE_STATES:
        st.warning("正在完成当前阶段的数据写入、视频编码、meta 更新和资源释放。状态变为 DONE 之前不要开始下一阶段。")
    elif status.state == "DONE":
        st.success("当前阶段数据已经保存完成，可以开始新的阶段采集。")

    if status.state == "COUNTDOWN" and status.countdown_remaining_s > 0:
        st.markdown(
            f"<div style='font-size:104px;font-weight:800;text-align:center'>{int(status.countdown_remaining_s) + 1}</div>",
            unsafe_allow_html=True,
        )
    if status.state == "RESETTING" and 0 < status.reset_remaining_s <= 5.0:
        st.markdown(
            f"<div style='font-size:96px;font-weight:800;text-align:center'>环境重置 {int(status.reset_remaining_s) + 1}</div>",
            unsafe_allow_html=True,
        )
    if status.state == "AWAIT_SAVE_DECISION":
        st.markdown(
            "<div style='font-size:48px;font-weight:700;text-align:center'>等待确认是否保存当前条</div>",
            unsafe_allow_html=True,
        )

    obs = status.latest_obs
    d1, d2, d3, d4 = st.columns(4)
    d1.metric("loop_dt_ms", f"{obs.get('loop_dt_ms', 0.0):.2f}")
    d2.metric("schedule_lag_ms", f"{obs.get('schedule_lag_ms', 0.0):.2f}")
    d3.metric("bridge_valid", f"{obs.get('bridge_valid', 0.0):.0f}")
    d4.metric("xr_age_ms", f"{obs.get('xr_age_ms', 0.0):.1f}")

    v1, v2, v3, v4 = st.columns(4)
    v1.metric("VR valid", f"{obs.get('vr_valid', 0.0):.0f}")
    v2.metric(
        "Action semantic",
        f"{obs.get('action_semantic_valid', 0.0):.0f}",
        f"L hold {obs.get('left_action_from_hold', 0.0):.0f}, R hold {obs.get('right_action_from_hold', 0.0):.0f}",
    )
    v3.metric("Bridge failures", f"{obs.get('bridge_failure_count', 0.0):.0f}")
    v4.metric("Bridge lost", f"{obs.get('bridge_lost_duration_ms', 0.0):.0f}ms")
    if status.state not in TERMINAL_STATES and (obs.get("bridge_valid", 0.0) <= 0.0 or obs.get("vr_valid", 0.0) <= 0.0):
        st.warning(f"VR/Bridge 状态异常，系统会尝试重连 socket 并在进程缺失时重启服务。最近恢复检查: {recovery_msg}")
    elif recovered and rt.recovery_logs:
        st.caption(f"最近恢复记录: {rt.recovery_logs[-1]}")

    st.subheader("三路相机")
    st.caption("采集页面显示录制线程的最新三路图像，前端按 30fps 采集视角展示；LeRobot 数据集真实保存也是 30fps。")
    img_cols = st.columns(3)
    images = status.latest_images
    for idx, name in enumerate(get_camera_bindings()):
        with img_cols[idx]:
            st.caption(f"{CAMERA_LABELS[name]} ({name})")
            image = images.get(name)
            if image is not None:
                st.image(image, channels="RGB", output_format="JPEG", width="stretch")
                st.caption(
                    f"display_fps={RECORDING_FPS}, save_fps={RECORDING_FPS}, "
                    f"age_ms={obs.get(f'cam_{name}_age_ms', 0.0):.1f}, "
                    f"valid={obs.get(f'cam_{name}_valid', 0.0):.0f}"
                )
            else:
                frame = rt.preview.latest(name)
                if frame and frame.valid:
                    st.image(frame.image_rgb, channels="RGB", output_format="JPEG", width="stretch")
                    st.caption(f"preview_fps={frame.fps:.1f}/20, recording_save_fps={RECORDING_FPS}")
                else:
                    st.warning("waiting")

    st.subheader("左右臂 action 实时曲线")
    history = rt.telemetry.history()
    if history:
        rows = []
        t0 = history[-1].stamp_s
        for sample in history:
            if t0 - sample.stamp_s > 10:
                continue
            row = {"t": round(sample.stamp_s - t0, 2)}
            for i, label in enumerate(ACTION_NAMES):
                row[f"L_{label}"] = sample.left_action[i]
                row[f"R_{label}"] = sample.right_action[i]
            rows.append(row)
        df = pd.DataFrame(rows).set_index("t") if rows else pd.DataFrame()
        lcol, rcol = st.columns(2)
        with lcol:
            st.caption("Left arm action")
            if not df.empty:
                st.line_chart(df[[f"L_{label}" for label in ACTION_NAMES]])
        with rcol:
            st.caption("Right arm action")
            if not df.empty:
                st.line_chart(df[[f"R_{label}" for label in ACTION_NAMES]])
        latest = history[-1]
        if not latest.action_live:
            st.info("当前 IK action topic 暂无实时命令；新采集数据会在这种情况下写入 hold action。若正在按住手柄控制，请检查 /left/control/move_j 和 /right/control/move_j。")
    else:
        st.warning("waiting for telemetry")

    if status.error:
        st.error(status.error)
    if status.dataset_path:
        st.caption(f"Dataset: {status.dataset_path}")


if hasattr(st, "fragment"):
    @st.fragment(run_every=0.5)
    def live_fragment() -> None:
        render_live()

    live_fragment()
else:
    render_live()
