# LeRobot Pico VR teleoperator plugin

Provide 14 action values from the same logical frame snapshot as the robot observation: left six joints and gripper, then right six joints and gripper. Actual arm control stays in the ROS teleoperation chain.

Each side selects a usable IK joint target with age ≤500 ms, otherwise attempts feedback. Gripper selection independently prefers a fresh control value, then feedback. Feedback fallback does not check feedback age; unavailable fields remain zero. Controller release does not directly trigger fallback. On bridge request exceptions, the last computed action is reused for up to 500 ms, then zeros. Age-derived observation flags are separate from this selection logic.

Requires Python 3.12 and the Pico/PiPER robot plugin. See the [installation guide](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/INSTALL.md) and [action/diagnostic details](https://github.com/xdongmiao/pico-piper-teleop/blob/main/docs/ARCHITECTURE.md#observations-and-actions). Integration code uses Apache-2.0.
