# agx_arm_moveit

[English](./README_EN.md)

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> 注：安装使用过程中出现问题可查看[第4部分](#4-可能遇见的问题)

## 概述

`agx_arm_moveit` 是 AgileX 系列机械臂的统一 MoveIt2 配置包，通过参数化设计支持所有臂型和末端执行器的组合，无需为每种配置维护独立的功能包。

**支持的臂型：** `nero`、`piper`、`piper_h`、`piper_l`、`piper_x`、

**支持的末端执行器：** 无末端执行器 (`none`)、AgileX 夹爪 (`agx_gripper`)、Revo2 灵巧手 (`revo2`)

**规划组与预设动作：**

| 规划组 | 说明 | 预设状态 |
|--------|------|----------|
| `arm` | 机械臂主体 | `home` — 零位姿态 |
| `gripper` | AgileX 夹爪（需 `effector_type:=agx_gripper`） | `gripper_open` — 完全张开<br>`gripper_half` — 半开<br>`gripper_close` — 完全闭合 |
| `hand` | Revo2 灵巧手（需 `effector_type:=revo2`） | `hand_open` — 张开<br>`hand_half_close` — 半握<br>`hand_close` — 握拳 |

---

## 1 安装 MoveIt2

1）二进制安装，[参考链接](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-$ROS_DISTRO-moveit*
```

2）源码编译方法，[参考链接](https://moveit.ai/install-moveit2/source/)

---

## 2 安装依赖

安装完 MoveIt2 之后，需要安装一些依赖

```bash
sudo apt-get install -y \
    ros-$ROS_DISTRO-control* \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-* \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-trajectory-msgs
```

若系统语言区域设置不为英文区域，须设置

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

---

## 3 使用方法

### 3.1 仿真演示（无需真实机械臂）

打开终端，运行以下指令：

```bash
cd ~/agx_arm_ws
source install/setup.bash
```

#### 3.1.1 无末端执行器

```bash
# Piper 机械臂
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper

# Nero 机械臂
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero

# 其他臂型：piper_x、piper_l、piper_h
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_x
```

#### 3.1.2 带夹爪

```bash
# Piper + 夹爪
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper

# Nero + 夹爪
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

#### 3.1.3 带灵巧手

```bash
# Piper + 左手灵巧手
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=revo2 revo2_type:=left

# Nero + 右手灵巧手
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=right
```

### 3.2 控制真实机械臂

#### 方式一：一键启动（推荐）

一条命令同时启动机械臂控制节点和 MoveIt2，自动接入关节反馈：

```bash
cd ~/agx_arm_ws
source install/setup.bash

# Piper + 夹爪
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# Nero + 灵巧手
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=nero effector_type:=revo2 revo2_type:=left

# Piper_X + 命名空间（多实例场景）
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper_x namespace:=piper_x
```

> 该 launch 支持所有 `agx_arm_ctrl` 参数（如 `tcp_offset`、`speed_percent`、`auto_enable` 等），详见 [agx_arm_ctrl 启动参数](../../README.md#启动参数)。
> - `follow` 默认为 `true`，MoveIt 自动订阅 `/feedback/joint_states` 跟随真实臂状态
> - `publish_gripper_joint` 自动设为 `false`，不发布 `gripper`（夹爪宽度）关节，避免 URDF 中不存在该关节名导致的 MoveIt 告警
> - 需要多机械臂并行时，可为该 launch 指定 `namespace`（例如 `namespace:=piper_x`）

#### 方式二：分步启动

**步骤 1：** 启动机械臂控制节点，详见：[agx_arm_ctrl](../../README.md)

**步骤 2：** 额外启动一个终端，运行 MoveIt2：

```bash
cd ~/agx_arm_ws
source install/setup.bash

# 示例：Piper + 夹爪，控制真实机械臂
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=true

# 示例：Nero + 灵巧手，控制真实机械臂
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=revo2 revo2_type:=left follow:=true
```

### 3.3 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `arm_type` | `piper` | 机械臂型号 | `nero`, `piper`, `piper_h`, `piper_l`, `piper_x` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2` |
| `revo2_type` | `left` | Revo2 灵巧手类型 | `left`, `right` |
| `namespace` | 空字符串 | 当前 MoveIt/控制实例命名空间（多实例推荐设置） | 任意合法 ROS 命名空间 |
| `follow` | `false` | 跟随真实机械臂状态（`true` 时 MoveIt 订阅 `/feedback/joint_states`；`false` 时订阅 `/control/joint_states`） | `true`, `false` |
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | TCP 偏移 [x, y, z, rx, ry, rz]（米/弧度），非零时规划目标和交互标记移至 TCP 位置 | - |
| `use_rviz` | `true` | 是否启动 RViz | `true`, `false` |
| `db` | `false` | 是否启动 MoveIt warehouse 数据库 | `true`, `false` |

#### 3.3.1 典型使用场景（follow 组合）

结合 `follow` 参数，下面给出 MoveIt 侧的常见使用方式：

- **场景 A：纯模型，无真机（仿真演示）**  
  - 需求：只在 RViz+MoveIt 里看模型、规划轨迹，不连真机。  
  - 配置：`follow:=false`（默认），MoveIt 订阅 `/control/joint_states`，完全在仿真侧运行。  
  - 示例：  
    ```bash
    # 仅仿真，不接入真实机械臂
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=none
    ```

- **场景 B：真机 + 仅控制不跟随（MoveIt 作为“上位机控制器”，不实时显示真实反馈）**  
  - 需求：MoveIt 规划并通过 `/control/joint_states` 控制真机，但 RViz 中的状态主要由 MoveIt 自己维护，对真机反馈不敏感（一般不推荐长期这样用，仅用于简单测试）。  
  - 配置：`follow:=false`，MoveIt 仍订阅 `/control/joint_states`，由 `agx_arm_ctrl` 将控制结果执行到真机。  
  - 示例：  
    ```bash
    # 终端1：启动真机控制
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

    # 终端2：MoveIt 规划并发 /control/joint_states，不订阅 /feedback/joint_states
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=false
    ```

- **场景 C：真机 + 控制 + 跟随（推荐实机方案）**  
  - 需求：MoveIt 规划控制真实机械臂，同时 RViz 中的模型实时跟随 `/feedback/joint_states`。  
  - 配置：`follow:=true`，MoveIt 改为订阅 `/feedback/joint_states`，以真实关节反馈为主。  
  - 示例 1（推荐一键启动，已在本 README 顶部介绍）：  
    ```bash
    ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
    ```
  - 示例 2（分步启动）：  
    ```bash
    # 终端1：启动真机控制
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper

    # 终端2：MoveIt 订阅 /feedback/joint_states，控制 + 跟随
    ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper follow:=true
    ```

> 一般推荐在真机场景下使用 **场景 C（follow:=true）**，保证规划结果与真实机械臂保持一致；场景 B 仅适合对反馈要求不高的简单测试。

### 3.4 RViz 中的操作

![piper_moveit](./assets/pictures/piper_moveit.png)

- 拖动机械臂末端的交互标记（6D 球）来设定目标位姿
- 在左侧 **MotionPlanning → Planning** 面板中：
  - **Planning Group** 下拉菜单可切换规划组（`arm` / `gripper` / `hand`）
  - **Goal State** 下拉菜单可选择预设动作（如 `home`、`gripper_open`、`hand_close` 等）
  - 点击 **Plan & Execute** 开始规划并执行

---

## 4 可能遇见的问题

### 4.1 运行 demo.launch.py 时报错

报错：参数需要一个 double，而提供的是一个 string
解决办法：
终端运行

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

或在运行 launch 前加上 `LC_NUMERIC=en_US.UTF-8`
例如

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper
```
