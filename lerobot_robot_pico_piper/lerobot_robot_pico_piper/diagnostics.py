from __future__ import annotations

import argparse
import statistics
import time

from .config_pico_piper import PicoPiperConfig
from .pico_piper import PicoPiper


def _summary(name: str, values: list[float]) -> str:
    if not values:
        return f"{name}: no samples"
    return (
        f"{name}: n={len(values)} min={min(values):.2f} "
        f"mean={statistics.fmean(values):.2f} max={max(values):.2f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Dry-run Pico/Piper LeRobot 30Hz capture diagnostics.")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=8765)
    parser.add_argument("--allow-missing-bridge", action="store_true")
    args = parser.parse_args()

    cfg = PicoPiperConfig(
        id="diagnostic",
        fps=args.fps,
        bridge_host=args.bridge_host,
        bridge_port=args.bridge_port,
        require_bridge=not args.allow_missing_bridge,
    )
    robot = PicoPiper(cfg)

    loop_dt = []
    lag = []
    cam_ages = {name: [] for name in cfg.camera_paths}
    ros_ages = {
        "left_joint": [],
        "right_joint": [],
        "left_tcp": [],
        "right_tcp": [],
        "xr": [],
        "left_action": [],
        "right_action": [],
    }
    valid_counts = {name: 0 for name in cfg.camera_paths}

    robot.connect()
    try:
        deadline = time.monotonic() + args.seconds
        frames = 0
        while time.monotonic() < deadline:
            obs = robot.get_observation()
            frames += 1
            loop_dt.append(float(obs["loop_dt_ms"]))
            lag.append(float(obs["schedule_lag_ms"]))
            for name in cfg.camera_paths:
                cam_ages[name].append(float(obs[f"cam_{name}_age_ms"]))
                valid_counts[name] += int(float(obs[f"cam_{name}_valid"]) > 0.5)
            for name in ros_ages:
                ros_ages[name].append(float(obs[f"{name}_age_ms"]))
        elapsed = args.seconds
        print(f"frames={frames} seconds={elapsed:.2f} effective_fps={frames / elapsed:.2f}")
        print(_summary("loop_dt_ms", [v for v in loop_dt if v > 0.0]))
        print(_summary("schedule_lag_ms", lag))
        for name, values in cam_ages.items():
            print(_summary(f"cam_{name}_age_ms", values))
            print(f"cam_{name}_valid={valid_counts[name]}/{frames}")
        for name, values in ros_ages.items():
            finite = [v for v in values if v < 999_999.0]
            print(_summary(f"{name}_age_ms", finite))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()

