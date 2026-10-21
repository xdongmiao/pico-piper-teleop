# Pico × PiPER

Pico VR teleoperation and LeRobot data collection for dual AgileX PiPER arms, with a ROS 2 telemetry bridge and a Streamlit capture console.

[![Offline checks](https://github.com/xdongmiao/pico-piper-teleop/actions/workflows/ci.yml/badge.svg)](https://github.com/xdongmiao/pico-piper-teleop/actions/workflows/ci.yml)

[中文说明](README.zh-CN.md) · [Installation](docs/INSTALL.md) · [Operation guide (中文)](docs/OPERATIONS.md) · [Architecture](docs/ARCHITECTURE.md) · [Third-party notices](THIRD_PARTY_NOTICES.md)

![Capture console](docs/images/capture-console.jpg)

Offline presentation preview; no arms or cameras are connected.

## What this repository contains

- The existing Pico → inverse kinematics → ROS 2 → dual-arm control chain.
- A read-only ROS 2 → TCP/JSON bridge for timestamped observations and action targets.
- LeRobot robot and teleoperator plugins with a shared logical frame clock, three cameras, and IK/hold action semantics.
- A browser console for device startup and checks, recording, per-episode save/discard decisions, reset waiting, and dataset inspection with limited checks. The arm-homing button reports that homing is unimplemented.

One collection stage produces one LeRobot dataset containing multiple episodes. The flow is **countdown → record → reset wait → confirm → save/discard/continue/finish**. Recording targets **30 Hz**; the browser refreshes its live panels every **0.5 seconds**. LeRobot's `send_action` is a no-op: the ROS teleoperation chain drives the arms. Action fallback depends on target age and available feedback; releasing a controller does not itself switch the recorder to hold. See the [behavior details](docs/ARCHITECTURE.md#observations-and-actions).

The public distribution preserves recording state transitions, action/observation fields, timing, topic names, robot commands, and IK calculations. The console theme and explanatory copy are maintained separately from that behavior. An unused `arm_interfaces` build dependency was removed; its messages were never used by this IK node. The AgileX driver and `agx_arm_msgs` remain included.

## Getting started

The hardware stack targets **Linux x86_64 with ROS 2 Humble**. Collection uses **Python 3.12** and the bundled **LeRobot 0.5.1** source snapshot. The ROS bridge uses the ROS installation's system Python. XRoboToolkit PC Service/SDK, native IK dependencies, FFmpeg/`ffprobe`, cameras, and CAN adapters must be prepared separately; see [the complete setup instructions](docs/INSTALL.md). The launch scripts use fixed `/projects` paths. A native Windows deployment or bundled hardware simulator is not provided.

In the prepared robot environment, mount this checkout at `/projects` and create the collection environment:

```bash
cd /projects
python3.12 -m venv lerobot_v051_py312
source lerobot_v051_py312/bin/activate
python -m pip install -e ./lerobot
python -m pip install -r requirements-ui.txt
python -m pip install -e ./lerobot_robot_pico_piper -e ./lerobot_teleoperator_pico_vr
if [ ! -e pico_piper_streamlit_app/camera_config.json ]; then
    cp pico_piper_streamlit_app/camera_config.example.json pico_piper_streamlit_app/camera_config.json
fi
bash pico_piper_streamlit_app/run_streamlit.sh
```

Open [localhost:8501](http://localhost:8501), configure the three cameras, and use **设备启动与扫描 → 数据采集 → 采集结果**. Set camera bindings and the [CAN adapter mapping](docs/INSTALL.md#5-can-adapter-mapping) for your rig before enabling equipment. Existing deployments retain their ignored local `camera_config.json`.

The Docker image export, SDK installer, calibration/device identities, recordings, model weights, and compiled libraries are intentionally not included. This is a source distribution, not an export of the original robot computer.

## Repository map

| Path | Role |
| --- | --- |
| `pico_piper_streamlit_app/` | Existing capture application, pages, and presentation theme |
| `lerobot_robot_pico_piper/` | LeRobot robot plugin and ROS 2 bridge |
| `lerobot_teleoperator_pico_vr/` | Pico teleoperator plugin |
| `pico_tele_agx/src/ik_solver/` | Pico input and inverse kinematics node |
| `pico_tele_agx/src/xr_msgs/` | XR message schemas, verified against licensed upstream |
| `pico_tele_agx/src/agx_arm_ros/` | AgileX ROS driver, messages, and robot models |
| `pyAgxArm/` | AgileX Python SDK source snapshot |
| `lerobot/` | LeRobot source snapshot; original licenses and headers retained |
| `tests/`, `scripts/` | Hardware-free checks and release verification |

The [architecture guide](docs/ARCHITECTURE.md) maps runtime responsibilities to source files. The [operation guide](docs/OPERATIONS.md) explains startup checks, recording decisions, quality indicators, and the limits of the results checker. Instructions inside bundled upstream trees describe those components; start with this repository's installation guide for the complete integration.

## Development checks

```bash
python -m pip install -r requirements-dev.txt
python scripts/check_release.py
python -m pytest tests -q
```

These checks cover source integrity, public-release hygiene, configuration resolution, and recording decision behavior without driving hardware. They do not certify real-robot operation. Hardware validation requires the configured ROS/XR/CAN/camera environment and an operator.

## License and attribution

The integration and presentation code is provided under [Apache-2.0](LICENSE). Bundled dependencies retain their respective licenses, including LeRobot (Apache-2.0), AgileX ROS packages (MIT/Apache-2.0), XR messages (MIT), and pyAgxArm (LGPL-3.0-only). See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the license files in each component.

See [CONTRIBUTING.md](CONTRIBUTING.md) for changes and [SECURITY.md](SECURITY.md) for vulnerability reporting.
