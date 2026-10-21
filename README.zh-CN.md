# Pico × PiPER

基于 Pico VR 的松灵 PiPER 双臂遥操与 LeRobot 数据采集项目，包含 ROS 2 桥接、三相机采集和 Streamlit 操作界面。

[English](README.md) · [安装说明](docs/INSTALL.md) · [操作指南](docs/OPERATIONS.md) · [系统结构与源码索引](docs/ARCHITECTURE.md) · [第三方许可](THIRD_PARTY_NOTICES.md)

![数据采集界面](docs/images/capture-console.jpg)

离线界面预览，未连接机械臂和相机。

## 功能

- Pico 手柄输入，经 IK 求解和 ROS 2 控制松灵 PiPER 双臂。
- 采集左右臂关节、末端位姿、夹爪、VR 输入及顶部／左右腕三路图像。
- observation 和 action 共用逻辑帧时间与桥接快照，目标录制频率为 30 Hz；网页实时面板每 0.5 秒刷新一次。
- 网页完成设备启动与扫描、阶段采集、逐条保存／丢弃／重采、环境重置等待和数据集检查。“机械臂复位”按钮保留原有占位行为，只提示尚未实现，不发送复位指令。

一个阶段对应一个 Dataset，可以包含多条 episode。流程为：**倒计时 → 录制 → 环境重置等待 → 人工确认 → 保存／丢弃／继续／结束**。录制 action 优先使用未超时的 IK 目标，超时或目标缺失时尝试反馈值回退；松开手柄不会直接触发切换。反馈回退本身未检查反馈年龄，需结合质量标记判断。LeRobot 的 `send_action` 保持 no-op，机械臂由 ROS 遥操链路驱动。

公开版本保留采集状态机、录制时序、话题、数据字段、机械臂指令及 IK 计算，维护视觉样式、说明文字和仓库文档。清除的 `arm_interfaces` 仅是未使用的历史头文件与构建依赖；松灵实际驱动及 `agx_arm_msgs` 均保留。

## 使用环境

完整系统需要 Linux x86_64、ROS 2 Humble、Pico/XRoboToolkit PC Service 与 SDK、松灵机械臂及 CAN、三路 USB 相机、FFmpeg（含 `ffprobe`）和原生 IK 依赖。LeRobot 采集环境使用 Python 3.12；ROS 桥接使用 ROS 对应的系统 Python。启动脚本中的 `/projects` 是固定路径，需将仓库放置或挂载到该位置。未提供原生 Windows 运行支持或硬件模拟模式。

具体安装及启动命令见 [安装说明](docs/INSTALL.md)。首次使用时创建本地 `camera_config.json`；设备页可选择 auto/manual 和手动兜底路径，稳定 `by-path`、`by-id` 绑定需编辑 JSON。已有部署保留自己的本地配置。CAN 激活脚本中的 USB 端口映射也必须按实际设备核对。

页面采用深色侧栏、浅色内容区与蓝绿色强调色。按钮、输入项及保存确认流程沿用原有实现。

使用时还需注意：

- 30 Hz 是采集目标，不是网页显示帧率或硬实时保证；通过 `schedule_lag_ms`、图像年龄和有效性字段判断实际情况。
- 环境重置等待不会自动归位或暂停机械臂；关闭设备只停止界面管理的进程，外部进程可能仍在运行。
- 单条质量提示供人工判断，不会禁止保存；结果页的“通过”只代表现有有限检查未报错，不代表数据完整性全部通过。
- 默认保存到本地。界面使用的 `zme/<dataset_name>` 是继承的 LeRobot 标识，不会上传到 GitHub 或 Hugging Face。

完整流程和已知检查边界见 [操作指南](docs/OPERATIONS.md)。桥接话题、action 回退规则及对应实现见 [系统结构](docs/ARCHITECTURE.md)。仓库内 LeRobot、AgileX 等上游目录的 README 面向各自组件，整套项目的安装以根目录文档为入口。

## 发布范围

仓库提供应用、桥接与插件、ROS 源码、机器人模型和必要的上游源码快照。Docker 镜像备份、SDK 安装包、编译产物、采集数据、模型权重和原机器相机标识不进入 Git。

```bash
python -m pip install -r requirements-dev.txt
python scripts/check_release.py
python -m pytest tests -q
```

离线验证不会驱动机械臂，不能替代实际设备联调。贡献方式见 [CONTRIBUTING.md](CONTRIBUTING.md)。集成与主题代码采用 Apache-2.0，上游组件保留各自原许可，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
