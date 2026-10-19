# Third-party notices and source provenance

The root Apache-2.0 license covers the integration and new presentation work; it does not replace licenses on bundled components or assert ownership over upstream code.

| Component / path | Origin | License / notice |
| --- | --- | --- |
| `lerobot/` | [Hugging Face LeRobot](https://github.com/huggingface/lerobot), version 0.5.1, local source based on commit `1396b9fab7aecddd10006c33c47a487ffdcb54b4` | Apache-2.0; original `LICENSE` and source headers retained. Some upstream files include additional notices; these remain applicable. |
| `pyAgxArm/` | [AgileX pyAgxArm](https://github.com/agilexrobotics/pyAgxArm), source snapshot from the original deployment | LGPL-3.0-only, original `LICENSE`; accompanying GPL-3.0 text in `licenses/GPL-3.0.txt`. No new SDK binary is distributed. |
| `pico_tele_agx/src/agx_arm_ros/` | [AgileX agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros), source snapshot | MIT at the driver repository root; the controller also includes its own Apache-2.0 license. Preserve all nested licenses. |
| `…/agx_arm_description/agx_arm_urdf/` | [AgileX agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf), source/model snapshot | MIT, original license included. |
| `pico_tele_agx/src/xr_msgs/` | [XR Robotics XRoboToolkit-Teleop-ROS](https://github.com/XR-Robotics/XRoboToolkit-Teleop-ROS/tree/a516bb28061030a0fb5a73367241bcaa883eaf02/ros2/xr_msgs) | MIT, upstream root license copied alongside the package. All three message schemas were compared with this pinned upstream commit and match. The original placeholder declaration is corrected to MIT. |
| XRoboToolkit PC Service / SDK | [XR Robotics PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) | External dependency; its own license and distribution terms apply. Not bundled. |

Upstream snapshots were retained instead of fetching a moving branch at installation. The LeRobot snapshot contains existing local documentation and GR00T policy-file differences; this release does not claim to be an unmodified upstream checkout. The recording path is retained from the source deployment.

Upstream test weight/data artifacts are omitted from this source-only release. The root `tests/` suite is the supported offline release check; running the full upstream training/policy test suite requires its upstream assets and optional environments.

The old `arm_interfaces` package explicitly lacked a license. It is not distributed. Its only consumers in this project were two unused includes and CMake/package dependency declarations, which have been removed without changing the IK implementation or ROS communication.

Not distributed: the original Docker image archive, PC Service `.deb`, runtime libstdc++ binaries, personal camera configuration/identities, old local operations notes, generated files, credentials, datasets and model checkpoints. Install external runtime components from their respective providers.

New changes are described in `CHANGELOG.md`. Where upstream history was absent from the source backup, no exact upstream commit is invented.
