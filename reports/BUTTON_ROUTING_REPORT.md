# 按键路由收紧结果报告

日期：2026-07-09

本文档记录按键路由三阶段收紧后的代码边界、输入契约、模式路由和行为变化。检查范围包括：

- `lib/controller/control_input.h`
- `lib/controller/input_router.h`
- `lib/controller/input_router.cpp`
- `lib/controller/controller.cpp`
- `lib/controller/actions.cpp`
- `lib/controller/actions/action_common.cpp`
- `lib/controller/actions/action_sit.cpp`
- `lib/controller/actions/action_kick.cpp`
- `lib/drivers/xbox.cpp`
- `lib/controller/host_comm.cpp`

## 总结

按键处理已经从分散在 controller 和各动作文件中的物理位判断，收紧为三层结构：

```text
Xbox / UART0 原始快照
    -> input_router 输入归一化与组合键解释
    -> control_input 语义请求
    -> actions.cpp 集中模式路由
    -> action_xxx.cpp 动作内部阶段
```

实施后的主要结果：

1. 按钮位由 Xbox 驱动管理，轴索引由输入路由私有管理，controller 只管理语义输入类型。
2. `input_router` 是唯一解释物理按钮组合的位置。
3. Xbox 和 UART0 输入增加 250 ms freshness 判断。
4. 输入来源变化或失效恢复时不会生成伪上升沿。
5. BALANCE 同周期多动作键改为显式唯一优先级。
6. STOP、BALANCE到动作、STOP回BOOT、KICK模式切换集中到 `actions.cpp`。
7. 动作文件不再 include `xbox.h`，也不再访问物理按钮位。
8. 摄像头、腿高和 roll 使用连续语义方向，不再直接检查方向键。

## 文件职责

### `control_input.h`

定义 controller 语义输入契约：

- `input_source`：`NONE`、`XBOX`、`HOST`。
- `action_request`：离散动作请求。
- `control_input`：动作层可见的语义输入。

`control_input` 不暴露 `raw_buttons`、`buttons` 或 `pressed_buttons`，动作层无法重新解释物理按钮。

### `input_router.cpp`

负责：

1. 选择 Xbox 或 UART0 Host 输入。
2. 检查输入快照是否过期。
3. 生成按键上升沿。
4. 在来源变化时抑制首帧上升沿。
5. 处理 `SELECT` 修饰键。
6. 生成摄像头、腿高和 roll 连续方向。
7. 应用摇杆死区并生成线速度/yaw rate。
8. 根据当前模式生成唯一 `action_request`。

### `controller.cpp`

当前只负责组织：

```text
读取 balance_core 状态
-> input_router::update()
-> 合并外部中位校准请求
-> 更新摄像头
-> actions_update()
-> 翻译 balance_request
```

`controller.cpp` 不再读取 Xbox/Host 队列，不再保存 `last_buttons`，不再解释组合键。

### `actions.cpp`

集中处理：

- 任意非 STOP 模式进入 STOP。
- STOP 回到 BOOT。
- BALANCE 进入 SIT/JUMP/KICK。
- BALANCE 动作状态重置。
- BALANCE 进入中位校准。
- KICK_PLACE/KICK_RUN 互相切换。
- KICK 启动退出等待。

### `action_xxx.cpp`

动作文件只消费语义：

- `action_common.cpp`：`roll_direction`、`leg_height_direction`。
- `action_sit.cpp`：`disable_leg_torque`、`exit_action`。
- `action_kick.cpp`：踢球视觉、机构和阶段，不解释组合键。
- `action_jump.cpp`：跳跃阶段，不解释按钮。

## 输入来源与 freshness

输入优先级保持不变：

```text
xbox_dev::connected() == true
    -> XBOX

xbox_dev::connected() == false
    -> HOST
```

两个来源统一使用：

```text
INPUT_TIMEOUT_US = 250000
```

满足以下任一情况时，按钮和轴值按零处理：

- 队列不存在。
- 队列没有有效快照。
- `timestamp_us == 0`。
- 快照年龄超过 250 ms。

这解决了 UART0 停止发送后旧按钮和摇杆值永久保留的问题。

## 来源切换与边沿

`input_router` 内部保存：

```text
last_source
last_buttons
last_fresh
```

只有满足以下条件才生成上升沿：

```text
当前来源 == 上一来源
上一周期 fresh
当前周期 fresh
```

因此：

- Xbox连接或断开后的第一帧不会产生伪动作。
- 过期输入恢复后的第一帧不会产生伪动作。
- 已按住的按钮需要释放并重新按下，才会触发离散动作。
- BOOT/STOP/SIT 使用的 held 语义仍然可以识别已经按住的 RB/LS。

## 物理按钮映射

统一按钮位作为 `xbox` 类的公开静态常量，由 `xbox.h` 管理。Xbox驱动负责把手柄库字段映射到该编码；UART0 Host继续使用相同的16位协议位图。`input_router` 向下依赖 Xbox 驱动接口并将其翻译为 controller 语义，Xbox 驱动不依赖 controller。

| 按钮 | 位值 | 语义 |
| --- | --- | --- |
| `A` | `0x0001` | 后跳 |
| `B` | `0x0002` | 右转跳；修饰键下重置平衡或退出踢球 |
| `X` | `0x0004` | 左转跳；修饰键下原地踢球 |
| `Y` | `0x0008` | 前跳；修饰键下运动踢球 |
| `SHARE` | `0x0010` | 保留 |
| `START` | `0x0020` | 全局 STOP |
| `SELECT` | `0x0040` | 修饰键 |
| `XBOX` | `0x0080` | 保留 |
| `LB` | `0x0100` | SIT |
| `RB` | `0x0200` | BOOT确认、STOP恢复、动作退出 |
| `LS` | `0x0400` | 腿部状态复位、SIT扭矩关闭 |
| `RS` | `0x0800` | 原地跳 |
| `UP/DOWN` | `0x1000/0x8000` | 腿高；修饰键下摄像头 |
| `LEFT/RIGHT` | `0x2000/0x4000` | roll调整 |

## 连续控制语义

### 摇杆

| 轴 | 输出 |
| --- | --- |
| `AXIS_LINEAR = 3` | 线速度，5%死区，负方向乘 `0.8f` |
| `AXIS_YAW = 0` | yaw rate，5%死区并取反 |

其他四路轴继续保留，当前不进入控制逻辑。

### 摄像头

```text
SELECT + UP   -> camera_direction = +1
SELECT + DOWN -> camera_direction = -1
```

`controller::update_camera()` 将方向转换为 `+/-120 deg/s`，保留原有 `0.05s` 速度低通和角度限幅。

### 腿高和 roll

只有过滤后按钮位图恰好等于一个方向键时才生成：

```text
仅 RIGHT -> roll_direction = +1
仅 LEFT  -> roll_direction = -1
仅 UP    -> leg_height_direction = -1
仅 DOWN  -> leg_height_direction = +1
```

`action_common.cpp` 每个控制周期按方向调整 `0.025f`，保持原有连续按住行为。

## 离散动作请求

`action_request` 当前包括：

```text
STOP
BOOT
BOOT_CONFIRM
RESET_BALANCE
SIT
JUMP_IN_PLACE
JUMP_FORWARD
JUMP_BACKWARD
JUMP_LEFT
JUMP_RIGHT
KICK_PLACE
KICK_RUN
KICK_EXIT
```

动作层看到的是这些语义，不知道请求来自哪个物理按钮。

## 路由优先级

### 全局

所有非 STOP 模式：

```text
START 上升沿 -> STOP
```

该请求优先于当前模式内的其他离散动作。

### BOOT / STOP

```text
BOOT + RB held -> BOOT_CONFIRM
STOP + RB held -> BOOT
```

BOOT确认仍只在 `WAIT_SIGNAL` 阶段推进启动流程。

### BALANCE

先生成独立的 `reset_leg`，再生成唯一模式请求。

修饰键优先级：

```text
SELECT+X -> KICK_PLACE
SELECT+Y -> KICK_RUN
SELECT+B -> RESET_BALANCE
```

同时出现多个修饰组合时，按上表从上到下选择一个。

普通动作优先级显式定义为：

```text
B  -> JUMP_RIGHT
X  -> JUMP_LEFT
A  -> JUMP_BACKWARD
Y  -> JUMP_FORWARD
RS -> JUMP_IN_PLACE
LB -> SIT
```

这个顺序保留了旧代码多个独立 `if` 最终“后写覆盖前写”的实际结果，但不再依赖语句副作用。

### SIT / MIDDLE_CALIBRATION

SIT：

```text
LS held -> disable_leg_torque
RB held -> exit_action
```

两个语义可以同周期同时成立，保留旧代码先关闭扭矩、再开始退出的可能性。

MIDDLE_CALIBRATION：

```text
RB held -> exit_action
```

外部中位校准请求仍只在 BALANCE/SIT 被 controller 消费。

### KICK_PLACE / KICK_RUN

```text
SELECT+B -> KICK_EXIT
KICK_PLACE + SELECT+Y -> KICK_RUN
KICK_RUN + SELECT+X -> KICK_PLACE
```

`KICK_EXIT` 优先于模式切换。处于 `EXIT_PREPARE` 时忽略模式切换，继续原有延时退出流程。

模式切换和退出发生的当前周期仍返回踢球基础平衡请求，避免出现一周期电机请求中断。

## 行为保持与有意变化

### 保持不变

- Xbox连接时优先使用Xbox，断开时使用Host。
- 线速度和yaw轴映射、死区及限幅。
- `SELECT+UP/DOWN`摄像头控制。
- 方向键腿高和roll连续调整。
- `START`进入STOP。
- BALANCE、SIT、JUMP、KICK按钮含义。
- KICK退出延时和两个KICK模式切换顺序。
- 中位校准HTTP请求路径。
- 动作阶段、舵机参数和平衡请求语义。

### 有意变化

- 输入超过250 ms未更新时自动归零。
- 输入来源变化时首帧不产生上升沿。
- 过期输入恢复时首帧不产生上升沿。
- 同周期多个BALANCE动作键采用显式唯一优先级，不再连续切换多次模式。

## 仍然保留的边界问题

### 摄像头所有权

摄像头手动事件仍在所有模式生成。KICK模式中，controller可能先执行手动摄像头更新，动作逻辑随后执行视觉摄像头更新。

这次没有改变该行为，以免同时修改踢球控制语义。后续可以增加：

```text
camera_owner = MANUAL | KICK_VISION
```

### Host按钮协议仍是裸位图

UART0 Host仍直接发送controller按钮位图。虽然动作层已经与物理位解耦，但协议仍与当前按钮布局绑定。等UART0设计确定后，可再决定是否改成语义命令协议。

## 验收结果

代码层目标：

```text
动作文件不再 include xbox.h
动作文件不再出现 BUTTON_* 或 BTN_*
物理组合键只在 input_router.cpp 解释
模式切换集中在 actions.cpp
```

构建检查：

```text
/home/bgzerol/.platformio/penv/bin/pio run
```

最终构建通过：

```text
RAM:   18.0%（58844 / 327680 bytes）
Flash: 36.6%（1150197 / 3145728 bytes）
```

`git diff --check` 通过；动作层物理按钮引用扫描通过。

## 结论

最终边界为：

```text
输入设备/协议
    -> controller输入契约
    -> input_router物理映射
    -> actions语义路由
    -> 动作内部阶段
    -> balance_request
```

以后调整手柄布局、组合键和同时按键优先级，主要修改 `input_router.cpp`；新增动作模式主要修改 `actions.cpp` 和对应 `action_xxx.cpp`；底层 `balance_core` 不感知输入设备、按钮或动作模式。
