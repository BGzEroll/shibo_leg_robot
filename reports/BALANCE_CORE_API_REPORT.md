# balance_core API 收紧结果报告

日期：2026-07-08

本文档基于当前源码检查 `lib/controller/balance_core.h`、`lib/controller/balance_core.cpp`、`lib/controller/controller.cpp` 以及动作层调用点，记录本次 `balance_core` API 收紧后的公共边界和调用方式。

## 总结

`balance_core` 已经从原来的 `target` / `command` / `status_snapshot` 宽接口，收紧为普通控制、特权动作、状态读取、调试读取四类 API：

1. 普通动作只通过 `motion_control` 设置电机、平衡、转向和运动目标。
2. 坐下、恢复、跳跃等特殊动作通过独立的特权控制结构表达直出、恢复和反馈覆盖。
3. 动作层只能读取 `motion_status`，不再看到 `feedback_vector`、`output` 等内部求解/调试字段。
4. 上位机调试保留 `debug_snapshot`，现有调试帧字段和顺序保持不变。

本次调整不改控制周期、任务创建、硬件初始化顺序、电机输出方向、LQI 参数、动作时序和上位机协议字节。

## 当前公开 API

正式公开位置：`lib/controller/balance_core.h`

### 控制类结构

| 类型 | 主要字段 | 用途 |
| --- | --- | --- |
| `motion_control` | `enable_motor`、`enable_balance`、`enable_steering`、`reset_reference`、`reset_yaw_integral`、`linear_vel`、`yaw_rate` | 普通平衡控制入口，表达电机/平衡/转向开关和目标运动量。 |
| `direct_output_control` | `enable`、`left`、`right` | 坐下等特殊流程使用的左右电机直出。 |
| `recover_control` | `enable`、`output_blend` | 恢复阶段只保留核心姿态反馈并渐进放大输出。 |
| `feedback_override` | `enable_linear_feedback`、`enable_yaw_feedback`、`enable_yaw_integral` | 跳跃等动作临时关闭部分反馈或积分。 |

### 状态类结构

| 类型 | 主要字段 | 用途 |
| --- | --- | --- |
| `motion_status` | `timestamp_us`、`pitch_angle`、`pitch_rate`、`avg_linear_vel`、`yaw_angle`、`yaw_rate`、`roll_angle`、`avg_leg_height` | 动作层稳定依赖的状态。 |
| `debug_snapshot` | 姿态、速度、参考、输入、反馈向量、输出、roll、左右腿高 | 上位机/调试使用，不作为普通动作层稳定依赖。 |
| `info` | `max_linear_vel`、`max_steer_vel` | 上层输入缩放和动作限幅信息。 |

### 函数

| 函数 | 当前调用者 | 作用 |
| --- | --- | --- |
| `apply_motion_control(const motion_control &control)` | `controller.cpp` | 写入普通运动控制请求。 |
| `apply_direct_output(const direct_output_control &control)` | `controller.cpp` | 写入直出电机请求。 |
| `apply_recover_control(const recover_control &control)` | `controller.cpp` | 写入恢复控制请求。 |
| `apply_feedback_override(const feedback_override &override_control)` | `controller.cpp` | 写入反馈覆盖请求。 |
| `get_motion_status(motion_status &out)` | `controller.cpp` | 给动作层读取稳定运动状态。 |
| `get_debug_snapshot(debug_snapshot &out)` | `host_comm.cpp` | 给上位机调试帧读取内部诊断状态。 |
| `get_info()` | `controller::init()` | 读取平衡核心限幅信息。 |
| `init()` | `controller::init()` | 初始化 LQI、舵机、IMU、电机和状态队列。 |
| `core_task_entry(void *arg)` | `start.cpp` | 高频 IO 任务入口。 |
| `control_task_entry(void *arg)` | `start.cpp` | 控制任务入口。 |

## 调用链现状

启动链路：

```text
start_init_all()
    -> controller::init()
        -> host_comm::init()
        -> balance_core::init()
        -> balance_core::get_info()
    -> task_list()
        -> balance_core::core_task_entry
        -> balance_core::control_task_entry
```

控制链路：

```text
balance_core::control_task_entry()
    -> controller::update(period_ms)
        -> balance_core::get_motion_status()
        -> actions_update()
        -> apply_balance_request()
            -> balance_core::apply_motion_control()
            -> balance_core::apply_direct_output()
            -> balance_core::apply_recover_control()
            -> balance_core::apply_feedback_override()
    -> control_step(period_ms)
        -> read_sensor()
        -> update_state()
        -> update_gain()
        -> solve_output()
        -> xQueueOverwrite(status_queue)
```

IO 链路：

```text
balance_core::core_task_entry()
    -> motor::left/right.loopFOC()
    -> motor::left/right.move()
    -> motor::encoder_queue()
    -> mpu6050_dev::queue()
```

## 上层请求边界

动作层不再直接依赖 `balance_core` 的底层命令结构。`controller::balance_request` 现在是 controller 自己的语义请求：

- `BALANCE`：普通平衡、手柄线速度、yaw rate、踢球视觉 yaw 叠加。
- `DIRECT_OUTPUT`：坐下流程中的左右电机小输出。
- `RECOVER`：BOOT、SIT 退出等恢复流程。
- `STOP`：关闭电机输出。

`controller.cpp` 内部的 `apply_balance_request()` 负责把动作语义请求翻译成四类 `balance_core` API 调用。这样动作层只表达“想做什么”，不直接表达“LQI 内部怎么改”。

## 收紧效果

已经移除的公共暴露：

1. `balance_core::target`
2. `balance_core::command`
3. `balance_core::status_snapshot`
4. `set_target()`
5. `set_command()`
6. `get_status()`
7. `info::wheel_radius`

仍然保留在 `balance_core.cpp` 内部的诊断信息包括输入、反馈向量、输出、参考值和左右腿高。这些信息只通过 `debug_snapshot` 给调试路径使用，不再进入动作层常规决策接口。

## 验证

已完成以下检查：

```text
git diff --check
/home/bgzerol/.platformio/penv/bin/pio run
```

第三阶段最终编译通过，固件资源占用约为：

```text
RAM:   18.3%
Flash: 36.6%
```
