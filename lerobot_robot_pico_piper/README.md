# LeRobot Pico/PiPER robot plugin

Collect observations from the local ROS 2 JSON bridge and three cameras. This plugin preserves the shared logical frame clock and the original 30 Hz recording behavior. Direct LeRobot arm actuation is deliberately disabled (`send_action` is a no-op).

Requires Python 3.12, LeRobot 0.5.1 and the configured ROS/XR/camera environment. The ROS-side bridge script runs with the ROS distribution's Python, separately from this package.

See the [repository installation guide](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/INSTALL.md) and [architecture](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/ARCHITECTURE.md). Hardware-specific camera configuration is local and not distributed. Integration code uses Apache-2.0.
