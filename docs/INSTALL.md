# Installation

## Reference environment

This project uses fixed `/projects` deployment paths. Use a Linux x86_64 workstation/container with ROS 2 Humble, a functioning display/session for XRoboToolkit, two PiPER arms with SocketCAN adapters, and three V4L2 cameras. The device console invokes Linux `bash`, `ps`, process-group and V4L2 APIs. Native Windows support, a bundled hardware simulator and a ready-made Docker image are not provided. Offline tests cover the UI and recording decisions without hardware; they do not establish a working robot deployment.

There are two Python environments:

| Environment | Purpose |
| --- | --- |
| ROS system Python, usually `/usr/bin/python3` | `rclpy`, bridge, AgileX ROS node and pyAgxArm |
| Python 3.12 virtual environment at `/projects/lerobot_v051_py312` | LeRobot 0.5.1, plugins and Streamlit |

Do not install a mismatched `rclpy` wheel into the LeRobot environment. The bridge is the boundary between them.

## 1. Source and external prerequisites

Clone this repository and make its **root**, containing `pico_piper_streamlit_app/` and `lerobot/`, available at `/projects` in the robot environment:

```bash
git clone https://github.com/xdongmiao/pico-piper-teleop.git
```

For a container, bind the checkout directory to `/projects`, provide the required CAN/camera access, and follow the XRoboToolkit networking and display requirements. For a host deployment, place the checkout at `/projects` or provide that path through a mount/symlink. Running from an arbitrary clone path does not satisfy the current scripts.

The Linux environment needs Bash, `ps`, `sudo`, `ip` (iproute2), `ethtool`, `modprobe` (kmod), FFmpeg and **`ffprobe`**. The results page launches `ffprobe` from `PATH`; installing Python video packages alone does not supply this system command. On an Ubuntu environment these tools can be installed with:

```bash
sudo apt-get update
sudo apt-get install -y bash procps sudo iproute2 ethtool kmod ffmpeg
ffmpeg -version
ffprobe -version
```

Provision ROS 2 Humble, colcon/rosdep, Python 3.12 with venv support, and native build tools separately. The following steps assume they are available.

Install ROS dependencies through your ROS distribution. Native dependencies required by `ik_solver/CMakeLists.txt` are:

- Pinocchio and Eigen: CMake requests versions **3.8.0** and **3.4**, respectively, without `EXACT`. Dependency versions must also be compatible with each other; newer versions are not automatically certified by this release.
- CasADi, qpOASES, OctoMap and hpp-fcl with their C++ headers/libraries.
- nlohmann-json, ament, rclcpp, tf2, standard sensor/geometry messages and services.
- [XRoboToolkit PC Service and Robot SDK](https://github.com/XR-Robotics/XRoboToolkit-PC-Service), compatible with the container's OS/ABI. The build expects `/opt/apps/roboticsservice/SDK/include` and `SDK/linux/64/libPXREARobotSDK.so`; the service startup expects `/opt/apps/roboticsservice/runService.sh`.

Install matching vendor packages or build these prerequisites according to their upstream instructions. This repository does not provide a prebuilt SDK or native dependency bundle.

The XR launch command in [process_manager.py](../pico_piper_streamlit_app/process_manager.py) prepends `/projects/third_party/libstdcxx_jammy_ppa/usr/lib/x86_64-linux-gnu` to `LD_LIBRARY_PATH`. If your SDK needs a newer libstdc++, provide a compatible library from your OS/package provider there or in the container's normal loader search path. See [third_party/README.md](../third_party/README.md). Required native libraries and SDK paths are defined in [ik_solver/CMakeLists.txt](../pico_tele_agx/src/ik_solver/CMakeLists.txt).

## 2. ROS-side packages

In a fresh shell outside the collection virtual environment, install the bundled AgileX SDK into the Python environment used by the ROS controller, then build the workspace. The bridge launcher explicitly uses `/usr/bin/python3`; keep ROS-side packages available to that interpreter:

```bash
cd /projects
/usr/bin/python3 -m pip install -e ./pyAgxArm
source /opt/ros/humble/setup.bash
cd pico_tele_agx
rosdep install --from-paths src --ignore-src --skip-keys=piper -r -y
colcon build --symlink-install
source install/setup.bash
```

Some native IK dependencies and the Pico SDK are not supplied by rosdep and must already be present. Match the SDK build to the selected OS. The unused historical `piper` and `arm_interfaces` declarations have been removed from the IK package. The bundled upstream controller still declares the legacy rosdep key `piper`; the command above skips that key because the actual SDK is the separately installed `pyAgxArm` source.

## 3. Collection environment

```bash
cd /projects
python3.12 -m venv lerobot_v051_py312
source lerobot_v051_py312/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ./lerobot
python -m pip install -r requirements-ui.txt
python -m pip install -e ./lerobot_robot_pico_piper -e ./lerobot_teleoperator_pico_vr
```

`lerobot/uv.lock` is the bundled upstream component's lockfile. It does not lock this repository's UI, plugins, ROS packages or native SDK; it is not a complete deployment lock. The commands above install the integration's own requirements as well. This release does not claim every upstream training policy or optional extra has been tested with this teleoperation stack.

## 4. Local camera configuration

```bash
cd /projects
if [ ! -e pico_piper_streamlit_app/camera_config.json ]; then
    cp pico_piper_streamlit_app/camera_config.example.json pico_piper_streamlit_app/camera_config.json
fi
```

The example has empty stable identities and fallback paths `/dev/video0`, `/dev/video2`, `/dev/video4`. These are examples, not device discovery results. Keep `/dev/v4l` and the target `/dev/videoN` devices visible inside the container.

The device page selects `auto`/`manual` mode and a manual/fallback path for top, left-wrist and right-wrist cameras. To establish stable bindings, edit `stable_path` (`/dev/v4l/by-path/...`) and/or `by_id` in the ignored local JSON. In auto mode, the UI resolver prefers stable path, then by-id, then fallback; when both stable path and by-id exist, it checks whether they refer to the same device. Manual mode uses the fallback directly. Inspect the effective paths shown on the device page before recording.

The UI passes its resolved paths into the recorder. A separately instantiated robot plugin reads the same local file but uses its own simpler path resolver. The **恢复默认 by-path 配置** control restores the code's reference USB-port paths, not the example JSON or a newly detected rig. See [camera_config.py](../pico_piper_streamlit_app/camera_config.py) and the [plugin configuration](../lerobot_robot_pico_piper/lerobot_robot_pico_piper/config_pico_piper.py).

## 5. CAN adapter mapping

The launch command expects **`can_left` and `can_right` at 1 Mbps**. The activation script [can_muti_activate.sh](../pico_tele_agx/src/agx_arm_ros/scripts/can_muti_activate.sh) contains this reference USB bus mapping:

| `ethtool -i` bus-info | Interface | Bitrate |
| --- | --- | --- |
| `1-9:1.0` | `can_left` | `1000000` |
| `1-4:1.0` | `can_right` | `1000000` |

Identify your adapters with `ip -details link show type can` and `ethtool -i <actual-interface>`, then update the script's `USB_PORTS` entries to match the physical left/right connections. These identifiers describe CAN adapters, independently of the V4L2 camera paths.

The UI skips activation if both interfaces already report UP, ERROR-ACTIVE and 1 Mbps. Otherwise it invokes the script with `--ignore`, which skips its module-count confirmation only; it does not discover left/right mappings. The script invokes `sudo`, `modprobe`, `ethtool` and `ip link`. Prepare the required device permissions and privileges before UI startup, or configure the interfaces from an operator terminal first; the browser cannot answer an interactive sudo prompt. Privilege configuration depends on the host/container environment.

## 6. Start the application

```bash
bash /projects/pico_piper_streamlit_app/run_streamlit.sh
```

Open [localhost:8501](http://localhost:8501). The script binds to `0.0.0.0:8501` and the application has no built-in login; limit access to the intended workstation/lab network. The bridge binds to `127.0.0.1:8765`. Opening an idle console starts telemetry and camera-preview workers. Device startup/shutdown and recording are explicit actions; during an active stage the collection page also attempts to recover missing bridge/XR services.

1. **设备启动与扫描**: configure cameras, enable devices, inspect live state and scan results.
2. **数据采集**: choose dataset root/name/task and timings; collect episodes and confirm save/discard.
3. **采集结果**: select the dataset actually produced, then inspect metadata, videos, timing and validity statistics. Its inherited example path is not a bundled dataset. See the [operation guide](OPERATIONS.md#结果页的检查范围) for the checker's limits.

The existing **机械臂复位** button is a placeholder: it reports that homing is not implemented and sends no robot command. The recording flow's reset wait is time for the operator to reset the environment; it is not an automatic arm-homing procedure.

Wait for saving/encoding and worker cleanup to finish before starting another stage. Device disable stops console-managed processes, leaves externally started processes running, and does not bring CAN interfaces down. Process optimization can stop externally started bridge/IK/arm processes, requires its confirmation checkbox, and is refused during an active stage. See [the operation guide](OPERATIONS.md).

## Existing deployments

Keep your local camera JSON, SDK installation, native dependencies and virtual environment. Mount the release source at the same `/projects` path. The source release excludes generated ROS `build/install/log`, so rebuild the workspace after switching the checkout. Original control and recording parameters remain unchanged.

## Offline development

```bash
python -m pip install -r requirements-dev.txt
python scripts/check_release.py
python -m pytest tests -q
```

These commands do not start the robot, CAN or VR service. Windows screenshots demonstrate the page presentation only; the complete device console remains a Linux/ROS application.
