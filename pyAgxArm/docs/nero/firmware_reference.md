# Nero Firmware Reference

> Supplement to [nero_api.md](nero_api.md#nero-api-documentation). Full version matrices and behavioral details. The main API manual keeps a short firmware section.

[Switch to 中文](#nero-固件参考)

---

## Version Evolution (newest first)

Read top to bottom: each row is **what changed vs the previous SDK driver**.

| SDK driver | Constant | Arm firmware | Changes vs previous driver |
| --- | --- | --- | --- |
| `v112` | `NeroFW.V112` | ≥ 1.12 | **New:** full CPV APIs (joints 1–7, CAN `0x181`–`0x187`); `set_motion_mode('cpv')`. **Changed:** `get_leader_joint_angles` uses bundled CAN `0x155` / `0x156` / `0x157` + `0x170`. **Compat:** `set_normal_mode()` is SDK no-op (firmware removed; call does not error). Inherits `v111` (`calibrate_joint`, 12-bit MIT, pose fix). |
| `v111` | `NeroFW.V111` | 1.11 | **New:** `calibrate_joint`. **Changed:** `move_mit` 12-bit `t_ff` (±16 N·m all joints); MIT frame without CRC; motion mode encoding; `move_p` / `get_flange_pose` without 1.10 pose workaround; `get_motor_states` only forces `velocity = 0` (no `current` flip). |
| `default` | `NeroFW.DEFAULT` | ≤ 1.10 | **Baseline:** 8-bit `t_ff`; per-joint MIT torque ranges; 1.10 pose workaround on `move_p` / `get_flange_pose`; `get_motor_states` forces `velocity = 0` and negates `current`; leader feedback on `0x501`–`0x507`. No `calibrate_joint`, no CPV public APIs. |

---

## How to Choose (quick lookup)

| Your firmware (from `get_firmware()`) | `firmeware_version` | Constant |
| --- | --- | --- |
| 1.12 or later | `"v112"` | `NeroFW.V112` |
| 1.11 | `"v111"` | `NeroFW.V111` |
| 1.10 or earlier | `"default"` (or omit) | `NeroFW.DEFAULT` |

---

## APIs with Firmware Requirements

All other public APIs in [nero_api.md](nero_api.md#nero-api-documentation) are available on **every** supported driver unless noted in their section.

| API / group | Minimum SDK driver | Notes |
| --- | --- | --- |
| `calibrate_joint` | `NeroFW.V111` | Not on `DEFAULT` |
| CPV: `move_cpv_*`, `get_cpv_*`, `set_cpv_*` | `NeroFW.V112` | Joints 1–7 only |
| `set_motion_mode('cpv')` | `NeroFW.V112` | `OPTIONS.MOTION_MODE.CPV` only on `V112`; not listed on `DEFAULT` / `V111` |
| `move_mit` | All (behavior differs) | See [MIT parameters by version](#mit-move_mit-parameters-by-version) |
| `set_normal_mode` | All (behavior differs on `V112`) | `V112`: SDK no-op |

---

## MIT (`move_mit`) Parameters by Version

| Driver | Joints | `t_ff` input range (N·m) | `t_ff` bits | Other |
| --- | --- | --- | --- | --- |
| `DEFAULT` | 1–7 | 1–2: ±24; 3–4: ±16; 5–7: ±8 | 8 | Per-joint scaling before encode |
| `V111` | 1–7 | All: ±16 | 12 | `v_des` sign flip except joint 6 (firmware workaround) |
| `V112` | 1–7 | Same as `V111` | 12 | Inherits `v111` `move_mit` |

---

## API Availability Matrix (full)

Legend: **✅** supported · **—** not in driver · **⚠️** supported with version-specific behavior.

| API / capability | `DEFAULT` (≤ 1.10) | `V111` (1.11) | `V112` (≥ 1.12) |
| --- | :---: | :---: | :---: |
| Connect / disconnect, `enable` / `disable`, `reset` | ✅ | ✅ | ✅ |
| `move_j` / `move_js` | ✅ | ✅ | ✅ |
| `move_p` / `move_l` / `move_c` | ✅ | ✅ | ✅ |
| `get_joint_angles`, `get_driver_states`, limits / crash APIs, etc. | ✅ | ✅ | ✅ |
| `move_mit` | ✅ | ✅ | ✅ |
| `calibrate_joint` | — | ✅ | ✅ |
| CPV (`move_cpv_*`, `get/set_cpv_*`) | — | — | ✅ |
| `get_leader_joint_angles` | ✅ | ✅ | ✅ |
| `set_leader_mode` / `set_follower_mode` | ✅ | ✅ | ✅ |
| `set_normal_mode` | ✅ | ✅ | ⚠️ |

---

## Version-Specific Behavior (full)

| Topic | `DEFAULT` (≤ 1.10) | `V111` (1.11) | `V112` (≥ 1.12) |
| --- | --- | --- | --- |
| **`move_p` / `get_flange_pose`** | 1.10 pose workaround (send + receive) | No workaround | Inherits `V111` |
| **`get_motor_states`** | `velocity = 0`; `current` negated | `velocity = 0` only | Inherits `V111` |
| **`get_leader_joint_angles` CAN** | `0x501`–`0x507` per joint | Same as `DEFAULT` | `0x155` / `0x156` / `0x157` + `0x170` |
| **`set_normal_mode`** | Sends on CAN | Same as `DEFAULT` | SDK no-op |
| **CPV** | Not implemented | Not implemented | Full stack; `0x181`–`0x187` |

---

## Per-Version Quick Reference

### Firmware ≥ 1.12 → use `NeroFW.V112`

- Use CPV and `set_motion_mode('cpv')`.
- Leader angles from `0x155` / `0x156` / `0x157` / `0x170`.
- `set_normal_mode()` does nothing (safe to call).
- `calibrate_joint` and 12-bit MIT same as 1.11.

### Firmware 1.11 → use `NeroFW.V111`

- `calibrate_joint` available.
- 12-bit MIT; fixed flange pose (no 1.10 workaround).
- No CPV APIs.

### Firmware ≤ 1.10 → use `NeroFW.DEFAULT`

- 8-bit MIT; pose workaround on Cartesian APIs.
- No `calibrate_joint`, no CPV.

---

# Nero 固件参考

> [nero_api.md](nero_api.md#nero-机械臂-api-使用文档) 的补充文档：完整版本矩阵与行为差异。主手册仅保留简短固件说明。

[Switch to English](#nero-firmware-reference)

---

## 版本演进（从新到旧）

自上而下阅读：每行表示**相对上一档 SDK 驱动的变化**。

| SDK 驱动 | 常量 | 机械臂固件 | 相对上一版的变化 |
| --- | --- | --- | --- |
| `v112` | `NeroFW.V112` | ≥ 1.12 | **新增：** 完整 CPV API（1–7 轴，CAN `0x181`–`0x187`）；`set_motion_mode('cpv')`。**变更：** `get_leader_joint_angles` 改为 `0x155` / `0x156` / `0x157` + `0x170` 成组帧。**兼容：** `set_normal_mode()` 在 SDK 侧为 no-op（固件已移除；调用不报错）。继承 `v111`（`calibrate_joint`、12-bit MIT、位姿修正）。 |
| `v111` | `NeroFW.V111` | 1.11 | **新增：** `calibrate_joint`。**变更：** `move_mit` 12-bit `t_ff`（全关节 ±16 N·m）；MIT 帧无 CRC；运动模式编码变更；`move_p` / `get_flange_pose` 无 1.10 位姿 workaround；`get_motor_states` 仅将 `velocity` 置 0（不再对 `current` 取反）。 |
| `default` | `NeroFW.DEFAULT` | ≤ 1.10 | **基线：** 8-bit `t_ff`；分关节 MIT 力矩范围；`move_p` / `get_flange_pose` 1.10 位姿 workaround；`get_motor_states` 将 `velocity` 置 0 且 `current` 取反；主臂反馈 `0x501`–`0x507`。无 `calibrate_joint`、无 CPV 公开 API。 |

---

## 如何选择（速查）

| 固件版本（`get_firmware()`） | `firmeware_version` | 常量 |
| --- | --- | --- |
| 1.12 及更新 | `"v112"` | `NeroFW.V112` |
| 1.11 | `"v111"` | `NeroFW.V111` |
| 1.10 及更早 | `"default"`（或不填） | `NeroFW.DEFAULT` |

---

## 有固件要求的 API

[nero_api.md](nero_api.md#nero-机械臂-api-使用文档) 中其余公开 API 在**各支持驱动上均可用**（除非该 API 小节另有说明）。

| API / 分组 | 最低 SDK 驱动 | 说明 |
| --- | --- | --- |
| `calibrate_joint` | `NeroFW.V111` | `DEFAULT` 无 |
| CPV：`move_cpv_*`、`get_cpv_*`、`set_cpv_*` | `NeroFW.V112` | 仅 1–7 轴 |
| `set_motion_mode('cpv')` | `NeroFW.V112` | 仅 `V112` 的 `OPTIONS.MOTION_MODE` 含 `cpv`；`DEFAULT` / `V111` 不含 |
| `move_mit` | 均有（行为不同） | 见 [MIT 分版本参数](#mitmove_mit分版本参数) |
| `set_normal_mode` | 均有（`V112` 行为不同） | `V112`：SDK no-op |

---

## MIT（`move_mit`）分版本参数

| 驱动 | 关节 | `t_ff` 输入范围（N·m） | `t_ff` 位数 | 其它 |
| --- | --- | --- | --- | --- |
| `DEFAULT` | 1–7 | 1–2：±24；3–4：±16；5–7：±8 | 8 | 编码前分关节缩放 |
| `V111` | 1–7 | 全关节 ±16 | 12 | 除 6 轴外 `v_des` 取反（固件 workaround） |
| `V112` | 1–7 | 与 `V111` 相同 | 12 | 继承 `v111` `move_mit` |

---

## API 支持矩阵（完整）

图例：**✅** 支持 · **—** 驱动未实现 · **⚠️** 支持但行为因版本而异。

| API / 能力 | `DEFAULT`（≤ 1.10） | `V111`（1.11） | `V112`（≥ 1.12） |
| --- | :---: | :---: | :---: |
| 连接 / 断开、`enable` / `disable`、`reset` | ✅ | ✅ | ✅ |
| `move_j` / `move_js` | ✅ | ✅ | ✅ |
| `move_p` / `move_l` / `move_c` | ✅ | ✅ | ✅ |
| `get_joint_angles`、`get_driver_states`、限位 / 碰撞等 | ✅ | ✅ | ✅ |
| `move_mit` | ✅ | ✅ | ✅ |
| `calibrate_joint` | — | ✅ | ✅ |
| CPV（`move_cpv_*`、`get/set_cpv_*`） | — | — | ✅ |
| `get_leader_joint_angles` | ✅ | ✅ | ✅ |
| `set_leader_mode` / `set_follower_mode` | ✅ | ✅ | ✅ |
| `set_normal_mode` | ✅ | ✅ | ⚠️ |

---

## 版本差异说明（完整）

| 主题 | `DEFAULT`（≤ 1.10） | `V111`（1.11） | `V112`（≥ 1.12） |
| --- | --- | --- | --- |
| **`move_p` / `get_flange_pose`** | 1.10 位姿 workaround（收发） | 无 workaround | 继承 `V111` |
| **`get_motor_states`** | `velocity = 0`；`current` 取反 | 仅 `velocity = 0` | 继承 `V111` |
| **`get_leader_joint_angles` CAN** | `0x501`–`0x507` 逐轴 | 与 `DEFAULT` 相同 | `0x155` / `0x156` / `0x157` + `0x170` |
| **`set_normal_mode`** | 经 CAN 下发 | 与 `DEFAULT` 相同 | SDK no-op |
| **CPV** | 未实现 | 未实现 | 完整；`0x181`–`0x187` |

---

## 分版本用户速查

### 固件 ≥ 1.12 → `NeroFW.V112`

- 使用 CPV 及 `set_motion_mode('cpv')`。
- 主臂关节角来自 `0x155` / `0x156` / `0x157` / `0x170`。
- `set_normal_mode()` 无实际效果（可调用、不报错）。
- `calibrate_joint`、12-bit MIT 与 1.11 相同。

### 固件 1.11 → `NeroFW.V111`

- 可用 `calibrate_joint`。
- 12-bit MIT；笛卡尔位姿无 1.10 workaround。
- 无 CPV API。

### 固件 ≤ 1.10 → `NeroFW.DEFAULT`

- 8-bit MIT；笛卡尔 API 带位姿 workaround。
- 无 `calibrate_joint`、无 CPV。
