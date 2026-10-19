#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socketserver
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_msgs.msg import GripperStatus
from xr_msgs.msg import Custom


def stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    value = int(sec) * 1_000_000_000 + int(nanosec)
    return value if value > 0 else None


def header_stamp_to_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    return stamp_to_ns(getattr(header, "stamp", None))


def fseq(values: Any) -> list[float]:
    return [float(v) for v in values]


def serialize_joint_state(msg: JointState) -> dict[str, Any]:
    return {
        "header_stamp_ns": header_stamp_to_ns(msg),
        "name": list(msg.name),
        "position": fseq(msg.position),
        "velocity": fseq(msg.velocity),
        "effort": fseq(msg.effort),
    }


def serialize_pose_stamped(msg: PoseStamped) -> dict[str, Any]:
    return {
        "header_stamp_ns": header_stamp_to_ns(msg),
        "pose": {
            "position": [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            "orientation": [
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ],
        },
    }


def serialize_gripper_status(msg: GripperStatus) -> dict[str, Any]:
    return {
        "header_stamp_ns": header_stamp_to_ns(msg),
        "width": float(msg.width),
        "force": float(msg.force),
        "voltage_too_low": bool(msg.voltage_too_low),
        "motor_overheating": bool(msg.motor_overheating),
        "driver_overcurrent": bool(msg.driver_overcurrent),
        "driver_overheating": bool(msg.driver_overheating),
        "sensor_status": bool(msg.sensor_status),
        "driver_error_status": bool(msg.driver_error_status),
        "driver_enable_status": bool(msg.driver_enable_status),
        "homing_status": bool(msg.homing_status),
    }


def serialize_controller(ctrl: Any) -> dict[str, Any]:
    return {
        "axis_x": float(ctrl.axis_x),
        "axis_y": float(ctrl.axis_y),
        "axis_click": bool(ctrl.axis_click),
        "gripper": float(ctrl.gripper),
        "trigger": float(ctrl.trigger),
        "primary_button": bool(ctrl.primary_button),
        "secondary_button": bool(ctrl.secondary_button),
        "menu_button": bool(ctrl.menu_button),
        "pose": fseq(ctrl.pose),
        "status": int(ctrl.status),
    }


def serialize_xr(msg: Custom) -> dict[str, Any]:
    return {
        "source_timestamp_ns": int(msg.timestamp_ns),
        "input": int(msg.input),
        "head": {
            "pose": fseq(msg.head.pose),
            "status": int(msg.head.status),
        },
        "left_controller": serialize_controller(msg.left_controller),
        "right_controller": serialize_controller(msg.right_controller),
    }


@dataclass
class Sample:
    recv_stamp_ns: int
    source_stamp_ns: int | None
    data: dict[str, Any]


class TopicStore:
    def __init__(self, maxlen: int):
        self._samples: deque[Sample] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, sample: Sample) -> None:
        with self._lock:
            self._samples.append(sample)

    def latest_before(self, frame_time_ns: int) -> Sample | None:
        with self._lock:
            samples = list(self._samples)
        for sample in reversed(samples):
            if sample.recv_stamp_ns <= frame_time_ns:
                return sample
        return samples[-1] if samples else None

    def count(self) -> int:
        with self._lock:
            return len(self._samples)


class PicoPiperBridge(Node):
    def __init__(self, ring_size: int):
        super().__init__("pico_piper_lerobot_bridge")
        self.stores = {
            "xr_pose": TopicStore(ring_size),
            "left_feedback_joint_states": TopicStore(ring_size),
            "right_feedback_joint_states": TopicStore(ring_size),
            "left_feedback_tcp_pose": TopicStore(ring_size),
            "right_feedback_tcp_pose": TopicStore(ring_size),
            "left_feedback_gripper_status": TopicStore(ring_size),
            "right_feedback_gripper_status": TopicStore(ring_size),
            "left_control_move_j": TopicStore(ring_size),
            "right_control_move_j": TopicStore(ring_size),
            "left_control_joint_states": TopicStore(ring_size),
            "right_control_joint_states": TopicStore(ring_size),
        }
        self._subscribe("xr_pose", "/xr_pose", Custom, serialize_xr, lambda msg: int(msg.timestamp_ns) or None)
        self._subscribe(
            "left_feedback_joint_states",
            "/left/feedback/joint_states",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )
        self._subscribe(
            "right_feedback_joint_states",
            "/right/feedback/joint_states",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )
        self._subscribe(
            "left_feedback_tcp_pose",
            "/left/feedback/tcp_pose",
            PoseStamped,
            serialize_pose_stamped,
            header_stamp_to_ns,
        )
        self._subscribe(
            "right_feedback_tcp_pose",
            "/right/feedback/tcp_pose",
            PoseStamped,
            serialize_pose_stamped,
            header_stamp_to_ns,
        )
        self._subscribe(
            "left_feedback_gripper_status",
            "/left/feedback/gripper_status",
            GripperStatus,
            serialize_gripper_status,
            header_stamp_to_ns,
        )
        self._subscribe(
            "right_feedback_gripper_status",
            "/right/feedback/gripper_status",
            GripperStatus,
            serialize_gripper_status,
            header_stamp_to_ns,
        )
        self._subscribe(
            "left_control_move_j",
            "/left/control/move_j",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )
        self._subscribe(
            "right_control_move_j",
            "/right/control/move_j",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )
        self._subscribe(
            "left_control_joint_states",
            "/left/control/joint_states",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )
        self._subscribe(
            "right_control_joint_states",
            "/right/control/joint_states",
            JointState,
            serialize_joint_state,
            header_stamp_to_ns,
        )

    def _subscribe(
        self,
        key: str,
        topic: str,
        msg_type: Any,
        serializer: Callable[[Any], dict[str, Any]],
        source_stamp: Callable[[Any], int | None],
    ) -> None:
        def callback(msg: Any) -> None:
            recv_ns = time.monotonic_ns()
            try:
                data = serializer(msg)
                src_ns = source_stamp(msg)
                self.stores[key].append(Sample(recv_ns, src_ns, data))
            except Exception as exc:
                self.get_logger().warning(f"Failed to serialize {topic}: {exc}")

        self.create_subscription(msg_type, topic, callback, 50)
        self.get_logger().info(f"Subscribed {topic} as {key}")

    def snapshot(self, frame_time_ns: int) -> dict[str, Any]:
        sources: dict[str, Any] = {}
        ages_ms: dict[str, float | None] = {}
        for key, store in self.stores.items():
            sample = store.latest_before(frame_time_ns)
            if sample is None:
                sources[key] = {"valid": False, "recv_stamp_ns": None, "source_stamp_ns": None, "data": None}
                ages_ms[key] = None
                continue
            age_ms = (frame_time_ns - sample.recv_stamp_ns) / 1_000_000.0
            sources[key] = {
                "valid": True,
                "recv_stamp_ns": sample.recv_stamp_ns,
                "source_stamp_ns": sample.source_stamp_ns,
                "data": sample.data,
            }
            ages_ms[key] = float(age_ms)
        return {
            "ok": True,
            "server_monotonic_ns": time.monotonic_ns(),
            "frame_time_ns": int(frame_time_ns),
            "sources": sources,
            "ages_ms": ages_ms,
        }

    def health(self) -> dict[str, Any]:
        return {
            "ok": True,
            "server_monotonic_ns": time.monotonic_ns(),
            "counts": {key: store.count() for key, store in self.stores.items()},
        }


class JsonLineHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        bridge: PicoPiperBridge = self.server.bridge  # type: ignore[attr-defined]
        for raw in self.rfile:
            try:
                req = json.loads(raw.decode("utf-8"))
                cmd = req.get("cmd")
                if cmd == "ping":
                    res = {"ok": True, "server_monotonic_ns": time.monotonic_ns()}
                elif cmd == "snapshot":
                    res = bridge.snapshot(int(req["frame_time_ns"]))
                elif cmd == "health":
                    res = bridge.health()
                else:
                    res = {"ok": False, "error": f"unknown command {cmd!r}"}
            except Exception as exc:
                res = {"ok": False, "error": str(exc)}
            self.wfile.write((json.dumps(res, separators=(",", ":")) + "\n").encode("utf-8"))


class ThreadedTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler: type[JsonLineHandler], bridge: PicoPiperBridge):
        super().__init__(server_address, handler)
        self.bridge = bridge


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--ring-size", type=int, default=600)
    args = parser.parse_args()

    rclpy.init()
    bridge = PicoPiperBridge(ring_size=args.ring_size)
    server = ThreadedTcpServer((args.host, args.port), JsonLineHandler, bridge)
    server_thread = threading.Thread(target=server.serve_forever, name="pico-piper-json-server", daemon=True)
    server_thread.start()
    bridge.get_logger().info(f"Pico/Piper LeRobot bridge listening on {args.host}:{args.port}")
    try:
        rclpy.spin(bridge)
    finally:
        server.shutdown()
        server.server_close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

