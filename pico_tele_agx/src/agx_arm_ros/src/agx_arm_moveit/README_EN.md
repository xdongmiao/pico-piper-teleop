# agx_arm_moveit

[中文](./README.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> **Note:** For installation issues, refer to [Section 4](#4-troubleshooting).

## Overview

`agx_arm_moveit` is a unified MoveIt2 configuration package for all AgileX robotic arms. Through parameterized design, it supports all arm types and end-effector combinations without maintaining separate packages for each configuration.

**Supported arm types:** `nero`, `piper`, `piper_h`, `piper_l`, `piper_x` 

**Supported end-effectors:** None (`none`), AgileX Gripper (`agx_gripper`), Revo2 Dexterous Hand (`revo2`)

**Planning groups and preset states:**

| Planning Group | Description | Preset States |
|----------------|-------------|---------------|
| `arm` | Robot arm body | `home` — zero position |
| `gripper` | AgileX Gripper (requires `effector_type:=agx_gripper`) | `gripper_open` — fully open<br>`gripper_half` — half open<br>`gripper_close` — fully closed |
| `hand` | Revo2 Dexterous Hand (requires `effector_type:=revo2`) | `hand_open` — open<br>`hand_half_close` — half close<br>`hand_close` — fist |

---

## 1. Install MoveIt 2

1) Binary Installation
[Reference](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-$ROS_DISTRO-moveit*
```

2) Build from Source
[Reference](https://moveit.ai/install-moveit2/source/)

---

## 2. Install Dependencies

After installing MoveIt 2, additional dependencies are required:

```bash
sudo apt-get install -y \
    ros-$ROS_DISTRO-control* \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-* \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-trajectory-msgs
```

**Locale Configuration:** If your system locale is not set to English, configure as follows:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Usage

### 3.1 Simulation Demo (No Real Arm Required)

Open a terminal and run:

```bash
cd ~/agx_arm_ws
source install/setup.bash
```

#### 3.1.1 Without End Effector

```bash
# Piper arm
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper

# Nero arm
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero

# Other arm types: piper_x, piper_l, piper_h
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_x
```

#### 3.1.2 With Gripper

```bash
# Piper + Gripper
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper

# Nero + Gripper
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

#### 3.1.3 With Dexterous Hand

```bash
# Piper + Left dexterous hand
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=revo2 revo2_type:=left

# Nero + Right dexterous hand
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=right
```

### 3.2 Control Real Robot Arm

#### Option 1: One-Click Launch (Recommended)

Start both the arm control node and MoveIt2 with a single command, automatically connecting joint feedback:

```bash
cd ~/agx_arm_ws
source install/setup.bash

# Piper + Gripper
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# Nero + Dexterous hand
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=nero effector_type:=revo2 revo2_type:=left

# Piper_X + namespace (multi-instance scenario)
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper_x namespace:=piper_x
```

> This launch supports all `agx_arm_ctrl` parameters (e.g. `tcp_offset`, `speed_percent`, `auto_enable`, etc.). See [agx_arm_ctrl Launch Parameters](../../README_EN.md#launch-parameters) for details.
> - `follow` defaults to `true`, so MoveIt automatically subscribes to `/feedback/joint_states` to track real arm state
> - `publish_gripper_joint` is automatically set to `false`, suppressing the `gripper` (opening width) joint that does not exist in the URDF, preventing MoveIt warnings
> - For multi-arm parallel use, you can set `namespace` for this launch (e.g. `namespace:=piper_x`)

#### Option 2: Step-by-Step Launch

**Step 1:** Start the arm control node. See: [agx_arm_ctrl](../../README_EN.md)

**Step 2:** Open an additional terminal and run MoveIt2:

```bash
cd ~/agx_arm_ws
source install/setup.bash

# Example: Piper + Gripper, controlling real arm
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=true

# Example: Nero + Dexterous hand, controlling real arm
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left follow:=true
```

### 3.3 Launch Parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `arm_type` | `piper` | Arm model | `nero`, `piper`, `piper_h`, `piper_l`, `piper_x` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `revo2_type` | `left` | Revo2 dexterous hand type | `left`, `right` |
| `namespace` | empty string | Namespace for the current MoveIt/control instance (recommended for multi-instance setups) | Any valid ROS namespace |
| `follow` | `false` | Follow real arm state (`true`: MoveIt subscribes to `/feedback/joint_states`; `false`: subscribes to `/control/joint_states`) | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP offset [x, y, z, rx, ry, rz] in meters/radians. When non-zero, the planning target and interactive marker align with the TCP position | - |
| `use_rviz` | `true` | Whether to launch RViz | `true`, `false` |
| `db` | `false` | Whether to start MoveIt warehouse database | `true`, `false` |

#### 3.3.1 Typical Usage Scenarios (follow combinations)

Based on the `follow` parameter, below are common MoveIt usage patterns:

- **Scenario A: Pure simulation, no real arm**  
  - Goal: Run MoveIt + RViz for visualization and planning only, without connecting to real hardware.  
  - Configuration: `follow:=false` (default). MoveIt subscribes to `/control/joint_states` and runs entirely in simulation.  
  - Example:  
    ```bash
    # Simulation only, no real arm
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=none
    ```

- **Scenario B: Real arm + control only, no follow** (MoveIt as high-level controller, not strictly synced to real feedback)  
  - Goal: Use MoveIt to plan and publish `/control/joint_states` to the real arm, while RViz mainly reflects the commanded state (not recommended for long-term precise use).  
  - Configuration: `follow:=false`. MoveIt still subscribes to `/control/joint_states`, and `agx_arm_ctrl` executes the commands on the real arm.  
  - Example:  
    ```bash
    # Terminal 1: start real arm control
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

    # Terminal 2: MoveIt plans and publishes /control/joint_states, does not subscribe to /feedback/joint_states
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=false
    ```

- **Scenario C: Real arm + control + follow (recommended for real hardware)**  
  - Goal: MoveIt plans and controls the real arm, while RViz stays synchronized with the real joint feedback.  
  - Configuration: `follow:=true`. MoveIt subscribes to `/feedback/joint_states` and uses real joint feedback as the primary state.  
  - Example 1 (recommended one-click launch, already described above):  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
    ```
  - Example 2 (step-by-step launch):  
    ```bash
    # Terminal 1: start real arm control
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

    # Terminal 2: MoveIt subscribes to /feedback/joint_states (control + follow)
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=true
    ```

> For real hardware, **Scenario C (`follow:=true`) is generally recommended** to keep MoveIt planning results consistent with the actual robot state. Scenario B is only suitable for simple tests where strict feedback consistency is not required.

### 3.4 RViz Operations

![piper_moveit](./assets/pictures/piper_moveit.png)

- Drag the interactive marker (6D ball) at the arm's end-effector to set target poses
- In the left **MotionPlanning → Planning** panel:
  - Use the **Planning Group** dropdown to switch between groups (`arm` / `gripper` / `hand`)
  - Use the **Goal State** dropdown to select preset states (e.g. `home`, `gripper_open`, `hand_close`, etc.)
  - Click **Plan & Execute** to plan and execute the trajectory

---

## 4. Troubleshooting

### 4.1 Error Running `demo.launch.py`

**Error:** Parameter expects a double but received a string.

**Solution:**

**Option A:** Configure locale permanently
```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

**Option B:** Prefix launch command
```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper
```
