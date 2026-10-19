# Installation

## Reference environment

This project preserves the original `/projects` deployment layout. Use a Linux x86_64 workstation/container with ROS 2 Humble, a functioning display/session for XRoboToolkit, two PiPER arms with SocketCAN adapters, and three V4L2 cameras. The UI can be inspected without hardware; teleoperation and recording require the complete stack.

There are two Python environments:

| Environment | Purpose |
| --- | --- |
| ROS system Python, usually `/usr/bin/python3` | `rclpy`, bridge, AgileX ROS node and pyAgxArm |
| Python 3.12 virtual environment at `/projects/lerobot_v051_py312` | LeRobot 0.5.1, plugins and Streamlit |

Do not install a mismatched `rclpy` wheel into the LeRobot environment. The bridge is the boundary between them.

## 1. Source and external prerequisites

Clone this repository and make it available at `/projects` in the robot environment. If using Docker, bind the checkout to `/projects`, give the container the required CAN/camera access, and follow the XRoboToolkit networking and display requirements. The original image archive is not distributed; prepare your own compatible image/environment.

Install ROS dependencies through your ROS distribution. Native dependencies required by `ik_solver/CMakeLists.txt` are:

- Pinocchio **3.8.0**, Eigen **3.4**, CasADi, qpOASES, OctoMap and hpp-fcl with their C++ headers/libraries.
- nlohmann-json, ament, rclcpp, tf2, standard sensor/geometry messages and services.
- [XRoboToolkit PC Service and Robot SDK](https://github.com/XR-Robotics/XRoboToolkit-PC-Service), compatible with the container's OS/ABI. The build expects `/opt/apps/roboticsservice/SDK/include` and `SDK/linux/64/libPXREARobotSDK.so`; the service startup expects `/opt/apps/roboticsservice/runService.sh`.

Install matching vendor packages or build these prerequisites according to their upstream instructions. This repository does not provide a prebuilt SDK or native dependency bundle.

The original XR launch command prepends `/projects/third_party/libstdcxx_jammy_ppa/usr/lib/x86_64-linux-gnu` to `LD_LIBRARY_PATH`. If your SDK needs a newer libstdc++, provide a compatible library from your OS/package provider there or in the container's normal loader search path. Do not copy arbitrary host libraries across incompatible distributions. See `third_party/README.md`.

## 2. ROS-side packages

Install the bundled AgileX SDK into the Python environment used by the ROS controller, then build the workspace:

```bash
cd /projects
python3 -m pip install -e ./pyAgxArm
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

`lerobot/uv.lock` retains the upstream dependency lock. Advanced deployments may provision the same virtual environment with `uv` using that lock instead of pip. This release does not claim every upstream training policy or optional extra has been tested with this teleoperation stack.

## 4. Local camera configuration

```bash
cd /projects
cp pico_piper_streamlit_app/camera_config.example.json pico_piper_streamlit_app/camera_config.json
```

Use the device page to select and save the top, left-wrist and right-wrist cameras. The original lookup order is stable `by-path` → `by-id` → fallback `/dev/videoN`, with a manual override. Fill the stable path and identity values for your own rig. Keep `/dev/v4l` and the target `/dev/videoN` devices visible inside the container. The local JSON is ignored by Git and is shared with the LeRobot plugin.

The reference arm interface names remain `can_left` and `can_right`, with the original 1 Mbps settings. Configure adapters and the ROS driver for your physical arms before enabling them.

## 5. Start the application

```bash
bash /projects/pico_piper_streamlit_app/run_streamlit.sh
```

Open [localhost:8501](http://localhost:8501). The existing script binds to `0.0.0.0`; keep access limited to the intended workstation/lab network. The bridge binds to loopback. Opening the page starts only the existing telemetry and camera-preview workers; device startup/shutdown and recording remain explicit UI actions.

1. **设备启动与扫描**: configure cameras, enable devices, inspect live state and scan results.
2. **数据采集**: choose dataset root/name/task and timings; collect episodes and confirm save/discard.
3. **采集结果**: inspect dataset metadata, videos, timing and validity statistics.

The existing **机械臂复位** button is a placeholder: it reports that homing is not implemented and sends no robot command. The recording flow's reset wait is time for the operator to reset the environment; it is not an automatic arm-homing procedure.

Do not start the next stage until saving/encoding has completed and the UI permits it. The process optimization control can stop external robot processes and retains its original explicit confirmation checkbox.

## Existing deployments

Keep your local camera JSON, SDK installation, native dependencies and virtual environment. Mount the release source at the same `/projects` path. The source release excludes generated ROS `build/install/log`, so rebuild the workspace after switching the checkout. Original control and recording parameters remain unchanged.

## Offline development

```bash
python -m pip install -r requirements-dev.txt
python scripts/check_release.py
python -m pytest tests -q
```

These commands do not start the robot, CAN or VR service. Windows screenshots demonstrate the page presentation only; the complete device console remains a Linux/ROS application.
