# LeRobot Pico/PiPER robot plugin

Collect observations from the local ROS 2 JSON bridge and three cameras. The collection default targets 30 Hz and shares a logical frame time/snapshot with the Pico teleoperator. Logical scheduling diagnostics do not measure complete wall-clock loop duration or establish hardware clock synchronization. Direct LeRobot arm actuation is disabled (`send_action` is a no-op).

Requires Python 3.12, LeRobot 0.5.1 and the configured ROS/XR/camera environment. The ROS-side bridge script runs with the ROS distribution's Python, separately from this package.

The UI launcher uses `127.0.0.1:8765` and 900 buffered samples per ROS topic; the standalone bridge defaults to 600. A successful snapshot does not establish topic freshness. Camera and ROS age flags are diagnostics, not a hardware interlock.

See the [repository installation guide](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/INSTALL.md) and [architecture, source map and data fields](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/ARCHITECTURE.md). Local camera configuration overrides reference USB paths; original serial/by-id identities are not distributed. Integration code uses Apache-2.0.
