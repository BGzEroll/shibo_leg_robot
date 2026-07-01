# Controller 逻辑链路说明

本文档说明 `lib/controller` 当前各模块、函数和 RTOS 任务之间如何连接。核心原则是：上层只表达意图，底层 `balance_core` 负责把意图和传感器状态变成电机目标。

## 总体链路

启动后系统形成三条并行链路：

```text
Xbox / host remote
        |
        v
controller::update()
        |
        v
actions_update()  -- 舵机动作 -->  sts3032
        |
        v
balance_core setters
        |
        v
balance_core::control_task_entry()
        |
        v
motor::target_queue()
        |
        v
balance_core::core_task_entry()
        |
        v
motor::left.move() / motor::right.move()
```

```text
motor::left/right encoder
        |
        v
balance_core::core_task_entry()
        |
        v
motor::encoder_queue()
        |
        v
balance_core::control_task_entry()
```

```text
mpu6050_dev::imu
        |
        v
balance_core::core_task_entry()
        |
        v
mpu6050_dev::queue()
        |
        v
balance_core::control_task_entry()
```

所有跨任务队列都按“最新快照”使用，长度为 1，写入时覆盖旧值。

## 启动链路

入口在 `start_init_all()`：

1. `led_dev::init()` 初始化板载 LED。
2. `xbox_dev::init()` 初始化 Xbox 输入队列和手柄对象。
3. `controller::init()` 初始化控制器内部链路：
   - `host_comm::init()` 初始化 UART0 和 host remote 输入队列。
   - `balance_core::init()` 初始化平衡核心状态队列，并初始化 `sts3032`、`mpu6050_dev`、`motor`。
   - 初始化上层动作状态机和腿部运行状态。
4. `task_list()` 创建所有任务。

任务分布：

- `led_dev::task_entry`：低优先级 LED 闪烁。
- `xbox_dev::task_entry`：20 ms 采样一次手柄，发布 `xbox_dev::data`。
- `balance_core::core_task_entry`：core 1 高频 IO 任务，负责 FOC、move、encoder、IMU。
- `balance_core::control_task_entry`：core 0 1 kHz 控制任务，负责上层更新和平衡算法。
- `host_comm::task_entry`：1 kHz host 串口收发。

## 高频 IO 任务

`balance_core::core_task_entry()` 是唯一直接碰电机运动接口的地方。

每轮循环：

1. 执行 `motor::left.loopFOC()` 和 `motor::right.loopFOC()`。
2. 从 `motor::target_queue()` 读最新 `motor::target_data`。
3. 调用 `motor::left.move(target.left_torque)` 和 `motor::right.move(target.right_torque)`。
4. 每 1000 us 读取 `motor::left/right` 的 shaft angle 和 velocity，发布到 `motor::encoder_queue()`。
5. 每 5000 us 调用 `mpu6050_dev::imu.update()`，发布姿态、角速度、加速度到 `mpu6050_dev::queue()`。
6. `taskYIELD()` 让出调度。

这里不运行模式逻辑、不运行 LQI、不读取摇杆输入。它只做尽可能短的硬件 IO。

## 1 kHz 控制任务

`balance_core::control_task_entry()` 每 1 ms 执行：

```cpp
controller::update(1);
control_step(1);
```

这意味着同一个 1 kHz 周期内先更新上层意图，再用最新意图跑底层控制。

### 上层更新

`controller::update()` 做四件事：

1. `balance_core::get_status(status)` 读取上一轮平衡核心发布的状态。
2. `sample_input()` 采样输入：
   - 优先读取 `xbox_dev::queue()`。
   - 手柄未连接时读取 `host_comm::remote_queue()`。
   - 生成 `control_input`，包括按钮、边沿按钮、摇杆目标速度。
3. `update_camera()` 处理 `SELECT + UP/DOWN` 摄像头舵机控制。
4. `actions_update()` 根据当前模式生成 `balance_request`。

最后把 `balance_request` 翻译为底层接口调用：

```cpp
balance_core::set_target(request.target);
balance_core::set_command(request.command);
```

`balance_core` 不再保存上层模式；动作模式只留在 `actions.*`。

### 动作层

`actions_update()` 根据 `mode_id` 分发：

- `BOOT`：等待 `RB`，随后立腿、准备、recover，成功后进入 `BALANCE`。
- `BALANCE`：正常平衡，摇杆给 `target.linear_vel` 和 `target.yaw_rate`；按钮触发坐下或跳跃。
- `SIT`：坐下、坐下保持、退出坐下、recover。
- `JUMP`：跳跃准备、推蹬、飞行、落地、恢复。
- `STOP`：停止输出；非 STOP 模式按 `START` 进入，STOP 下按 `RB` 回到 `BOOT`。

动作层可以做：

- 设置 `balance_request`。
- 调 `sts3032::set()` / `sts3032::move()` 控制腿部舵机。
- 调 `sts3032::set_torque_switch()` 切换舵机扭矩。

动作层不做：

- 不访问 `motor::left/right`。
- 不写 `motor::target_queue()`。
- 不修改 LQI 状态、积分项和反馈增益。

## 平衡核心控制步骤

`control_step()` 是底层平衡核心的主路径：

1. 读取 `controller::update()` 刚写入的内部请求状态。
2. `read_sensor()` 组装内部传感快照：
   - 每 20 ms 调 `sts3032::get_position_and_load()` 更新腿部位置。
   - 从 `mpu6050_dev::queue()` 读 IMU 快照。
   - 从 `motor::encoder_queue()` 读 encoder 快照。
   - 用 `servo_count_to_height()` 把舵机位置转换成腿长。
3. `update_state()` 把 IMU 和 encoder 快照写入 `lqi::state`。
4. `update_gain()` 根据平均腿长更新 LQI feedback gain。
5. 根据命令处理 reset：
   - `reset_reference` 清空线速度/yaw 参考和积分。
   - `reset_yaw_integral` 只清 yaw 积分。
6. 如果 `enable_balance` 为 true：
   - `update_linear_reference()` 更新线速度参考和线速度积分。
   - `update_yaw_reference()` 更新 yaw rate 参考和 yaw 积分。
7. 如果 `enable_balance` 为 false，清空参考和积分。
8. `solve_output()` 计算 LQR 输出或处理 direct output。
9. 发布 `status_snapshot` 到 `status_queue`。

## balance_request 语义

`balance_request` 是动作层内部表达意图的结构；它由 `balance_core::target` 和 `balance_core::command` 组成，`controller::update()` 会把两部分分别传给 `balance_core::set_target()` 和 `balance_core::set_command()`。

- `target.linear_vel`：上层希望的线速度目标。
- `target.yaw_rate`：上层希望的 yaw rate 目标。
- `target.direct_left/right`：direct output 模式下的左右电机目标值。
- `command.enable_motor`：false 时输出 0 到电机目标队列。
- `command.enable_balance`：false 时不做 LQI 平衡输出，并清空运动参考。
- `command.enable_steering`：false 时 yaw 参考和 yaw 积分归零。
- `command.reset_reference`：清空线速度、yaw 参考和积分。
- `command.reset_yaw_integral`：只清 yaw 积分。
- `command.direct_output`：true 时跳过 LQI，直接发布 `target.direct_left/right`。
- `command.suppress_linear_feedback`：计算反馈向量时忽略线速度误差。
- `command.suppress_yaw_feedback`：计算反馈向量时忽略 yaw rate 误差和 yaw 积分。
- `command.suppress_yaw_integral`：不使用或清除 yaw 积分。
- `command.recover_active`：恢复阶段只保留 pitch 角和 pitch 角速度反馈。
- `command.output_blend`：恢复阶段用来平滑放大输出。

## status_snapshot 语义

`status_snapshot` 是底层向上层和 host 输出的最新状态。

- `pitch_angle`、`pitch_rate`、`avg_linear_pos`、`avg_linear_vel`、`yaw_angle`、`yaw_rate`：LQI 当前反馈状态。
- `reference_linear_vel`、`reference_yaw_rate`：当前线速度和 yaw rate 参考。
- `input`：当前命令目标值。
- `feedback_vector`：进入 LQI 求解的 6 维反馈向量。
- `output`：左右电机目标输出。
- `roll_angle`、`leg_height`、`avg_leg_height`：动作层和 host 调试使用。

`controller::update()` 读取的是上一轮 `control_step()` 发布的状态，所以动作决策天然滞后一轮 1 ms。这是有意的快照模型，避免跨任务直接访问底层内部状态。

## Host 链路

`host_comm::task_entry()` 每 1 ms 执行：

1. `update_rx()` 从 UART0 读数据。
2. `parse_rx()` 查找 `0xFF 0xAA` 帧头。
3. type 为 `0x01` 的帧进入 `parse_xbox()`。
4. `parse_xbox()` 把按钮和 6 路轴值写入 `host_comm::remote_queue()`。
5. `send_status()` 每 20 ms 从 `balance_core::get_status()` 取状态并发送调试帧。

当 Xbox 已连接时，`controller::sample_input()` 使用 Xbox 输入；当 Xbox 未连接时，使用 host remote 输入作为后备。

## 模式切换按钮

当前按钮约定：

- `RB`：BOOT/STOP/SIT 退出等待中的启动或恢复信号。
- `START`：任意非 STOP 模式进入 STOP。
- `LB`：BALANCE 中进入 SIT。
- `RS`：BALANCE 中原地跳。
- `Y`：BALANCE 中前跳。
- `A`：BALANCE 中后跳。
- `X`：BALANCE 中左转跳。
- `B`：BALANCE 中右转跳。
- `LS`：BALANCE 中低速时复位腿部运行状态；SIT DONE 中关闭舵机扭矩。
- `SELECT + UP/DOWN`：摄像头舵机上下转动。

## 扩展新动作的接入点

新增动作时优先改 `actions.*`：

1. 在 `mode_id` 中增加新模式。
2. 在 `action_state` 中加入动作需要的少量运行状态。
3. 写一个 `update_xxx()`，返回 `balance_request`。
4. 在 `actions_update()` 中添加分支。
5. 从现有模式中按按钮或条件调用 `begin_mode()` 切换过去。

新动作仍然只通过 `balance_request` 影响平衡核心；如果需要腿部姿态，直接调用 `sts3032` 模块；不要访问电机和 LQI 内部状态。
