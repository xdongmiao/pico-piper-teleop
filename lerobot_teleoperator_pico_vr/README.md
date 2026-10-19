# LeRobot Pico VR teleoperator plugin

Provide the existing dual-PiPER action vector from the same timestamped bridge snapshot used for the robot observation. Fresh IK targets use the existing feedback-hold fallback when needed. Actual arm control stays in the ROS teleoperation chain.

Requires Python 3.12 and the Pico/PiPER robot plugin. See the [installation guide](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/INSTALL.md). Integration code uses Apache-2.0.
