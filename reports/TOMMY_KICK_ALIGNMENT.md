# Tommy 踢球功能对齐报告

本文档对照 `/home/bgzerol/esp32-arduino/shibo-tommy-code/src/main.cpp` 中的踢球逻辑，说明当前项目 `lib/controller/actions/action_kick.cpp` 的行为是否已经对齐。

结论：当前项目已经把原地踢球和运动踢球的核心行为语义对齐到 Tommy 逻辑，包括 `dy` 控相机、`dx` 控车身转向、挡板准备/踢球时机、丢球后相机低头和关闭挡板。由于当前项目已经拆成 `host_comm -> actions -> balance_core` 的接口框架，运动踢球里的前进、转向、踢后回正和后退是按当前 `balance_core::target_t` 做了等效映射，不是逐行照搬 Tommy 的 `joyx/joyy` 全局变量写法。

## 输入协议链路

当前项目使用 MaixCam 新二进制协议：

```text
FF AA 02 04 dx:int16 dy:int16 checksum
```

解析位置：

- `host_comm::parse_rx()` 识别 `0xFF 0xAA` 帧头。
- `cmd == 0x01` 仍走原有 Xbox remote 解析，不动。
- `cmd == 0x02` 走视觉帧解析。
- `host_comm::vision_latest()` 向动作层提供最新视觉快照。

视觉丢失条件：

- MaixCam 发 `dx == 32767` 或 `dy == 32767`。
- 或视觉快照超过 `VISION_TIMEOUT_MS = 350` ms。

这和 Tommy 代码中 `dx == 9999 || dy == 9999` 表达的是同一个语义，只是丢失哨兵值从 ASCII 旧协议的 `9999` 换成当前二进制协议的 `32767`。

## 模式入口

当前项目入口位于 `actions.cpp`：

- `SELECT + X`：进入 `KICK_PLACE`，对应 Tommy 的 `kick_place_mode`。
- `SELECT + Y`：进入 `KICK_RUN`，对应 Tommy 的 `kick_run_mode`。
- `SELECT + B`：从踢球模式取消并回到 `BALANCE`。

进入踢球模式后，`prepare_kick()` 会做：

- 清空 `kick_runtime`。
- 记录当前 yaw 为 `target_yaw`，用于运动踢球后回正。
- 摄像头设置到 `CAM_INITIAL_ANGLE = 90`。
- 前挡板设置到 `FRONTIER_READY_ANGLE = 100`。

Tommy 的 `SetKickMode()` 会开启 `trace_mode`、设置 `kick_mode`、记录 `kick_yaw_zero`。当前项目没有全局 `trace_mode/kick_mode` 字符串，而是用 `mode_id::KICK_PLACE/KICK_RUN` 表达同一件事。

## 共同基础行为

踢球模式的基础请求由 `base_command()` 生成：

- `enable_balance = true`
- `enable_motor = true`
- `enable_steering = true`
- 继续运行腿部 roll 控制

这对应 Tommy 中踢球模式仍然运行 LQR 平衡，只是把 `dx_to_joyx/dy_to_joyy/yaw_to_joyx` 接入平衡控制输入。

当前项目不直接写电机和 LQR 内部状态，而是只返回：

- `target.linear_vel`
- `target.yaw_rate`
- `command_t`

这符合当前项目“动作层表达意图，`balance_core` 负责底层控制”的接口框架。

## 丢失目标行为

Tommy 行为：

```cpp
dx_to_joyx = 9999;
dy_to_joyy = 9999;
trace_ob_flag = false;
last_dy = 0;
last_dy_time = 0;
setCarmeraFixServoAngles(60);
setFrontierServoAngles(0);
```

当前项目行为：

```cpp
state.kick.cam_error = 0.0f;
state.kick.cam_rate = 0.0f;
state.kick.yaw_rate = 0.0f;
state.kick.last_dy = 0;
state.kick.last_dy_time = 0;
set_camera(state, CAM_LOST_ANGLE);
set_frontier(state, FRONTIER_KICK_ANGLE);
```

常量对齐：

- `CAM_LOST_ANGLE = 60`
- `FRONTIER_KICK_ANGLE = 0`

对齐状态：已对齐。

差异说明：Tommy 在丢球分支后没有立刻 `return`，但后续 `cam_aim_ball()` 和 `joyx_aim_ball()` 对 `9999` 会直接忽略。当前项目中 `vision_latest()` 返回 false 后直接返回基础平衡请求，行为效果更干净，等价于“不继续追踪、不前进、不转向”。

## 相机 dy 跟踪

Tommy 行为：

```cpp
if(abs(dy) > 10)
{
    dt = max(1, current_time - last_dy_time);
    d_term = pid_cam_d * (dy - last_dy) / dt * 10;
    p_term = pid_cam_p * dy;
    pd_output = constrain(p_term + d_term, -pid_trace_joyy_output_limit, pid_trace_joyy_output_limit);
    new_s2 = current_s2 - (int)pd_output;
    setCarmeraFixServoAngles(new_s2);
}
else
{
    last_dy = 0;
}
```

Tommy 参数：

- `pid_cam_p = 0.07`
- `pid_cam_d = 0.05`
- `pid_trace_joyy_output_limit = 10`
- 死区 `abs(dy) <= 10`

当前项目行为：

- `CAM_PD_P = 0.07`
- `CAM_PD_D = 0.05`
- `CAM_PD_STEP_LIMIT = 10`
- `CAM_AIM_DEADBAND = 10`
- `set_camera(state, state.kick.cam_angle - pd_output)`

对齐状态：已对齐。

差异说明：Tommy 把 `pd_output` 转成 `int` 后更新舵机，当前项目保留 float 状态后再写 `uint16_t` 舵机角。实际舵机输出仍然是整数角度，行为等效。

## dx 转向跟踪

Tommy 行为：

```cpp
if(abs(dx) < 50)
{
    trace_ob_flag = true;
    dx_to_joyx = 9999;
}
else
{
    trace_ob_flag = false;
}

if(!trace_ob_flag)
{
    int joyx_direction = (dx > 0) ? -1 : 1;
    dx_to_joyx = pid_trace_joyx(abs(dx)) * joyx_direction;
}
```

Tommy 参数：

- `pid_trace_joyx(0.5, 0, 0, 100000, 100)`
- `abs(dx) < 50` 时认为车头已经瞄准
- `dx > 0` 时输出负方向，`dx < 0` 时输出正方向

当前项目行为：

```cpp
if(abs(dx) < 50){return 0.0f;}

direction = (dx > 0) ? -1.0f : 1.0f;
joy_rate = constrain(abs(dx) * 0.5f, 0.0f, 100.0f) * direction;
yaw_rate = joy_rate * (YAW_RATE_LIMIT / 100.0f);
```

对齐状态：语义对齐，输出单位做了接口映射。

差异说明：Tommy 输出的是模拟摇杆 `dx_to_joyx`，再由原工程 LQR 里的 yaw 控制使用；当前项目动作层直接输出 `target.yaw_rate`。因此这里把 Tommy 的 `[-100, 100]` joyx 输出线性映射到当前 `[-0.9, 0.9]` yaw rate。

## 原地踢球模式

Tommy 原地踢球主逻辑：

```cpp
cam_aim_ball(dy);
joyx_aim_ball(dx);
if(current_s2 < place_ball_s2 && dy > place_kick_ball_dy && dy < 120)
{
    kick_ball_core();
}
else
{
    ready_kick_ball();
}
```

Tommy 参数：

- `place_ball_s2 = 40`
- `place_kick_ball_dy = -10`
- `dy < 120`
- `ready_kick_angel = 100`
- `kick_ball_core()` 将前挡板置 `0`

当前项目原地踢球逻辑：

- 每轮读取视觉快照。
- 有目标时先执行 `aim_camera(vision.dy)`。
- 再根据 `vision.dx` 输出 `target.yaw_rate`。
- 当 `cam_angle < 40 && dy > -10 && dy < 120` 时触发 `trigger_kick()`，挡板置 `0`。
- 否则执行 `ready_kick()`，挡板置 `100`。

对齐状态：已对齐。

需要注意的行为含义：

- 球在画面上方时 `dy < 0`，相机会按 Tommy PD 逻辑调整。
- 球逐渐进入车身下方附近时，摄像头角度会变小。
- 只有摄像头已经低到阈值以下，并且 `dy` 落在 `(-10, 120)` 区间内，才认为球到了踢球区域。
- 未到踢球区域时，挡板保持打开到后方的准备角 `100`。
- 到踢球区域时，挡板打回 `0`，让球进入射门区域。

## 运动踢球模式

Tommy 运动踢球主逻辑：

1. `cam_aim_ball(dy)` 持续相机跟踪。
2. 若已经踢过球，进入 `kick_over_count` 阶段：
   - 先 `align_yaw_zero()` 回正。
   - 回正后 `dy_to_joyy = -50` 后退。
   - 计数结束后清 `kick_over_count`。
3. 若还没踢球：
   - `joyx_aim_ball(dx)` 转向瞄准。
   - 若 `chased_flag` 已置位，则踢球并进入踢后阶段。
   - 否则 `chase_ball(dy)` 追球。

Tommy `chase_ball(dy)`：

```cpp
if(current_s2 < chase_ball_s2 && dy > run_kick_ball_dy && dy < 120)
{
    chased_flag = true;
}
else
{
    ob_y = ob_ball_dy - dy;
    dy_to_joyy = pid_trace_joyy(ob_y);
    ready_kick_ball();
}
```

Tommy 参数：

- `chase_ball_s2 = 30`
- `run_kick_ball_dy = -5`
- `ob_ball_dy = 120`
- `pid_trace_joyy(0.2, 0, 0, 100000, 100)`
- `kick_over_count_max = 50`

当前项目运动踢球逻辑：

- 相机跟踪和 dx 转向同 Tommy 语义。
- 当 `cam_angle < 30 && dy > -5 && dy < 120` 时置 `chased = true`。
- 下一轮 `chased` 为 true 时触发踢球，进入 `post_kick`。
- 未追到球时：
  - `ob_y = 120 - dy`
  - `linear_vel = ob_y * CHASE_LINEAR_KP`
  - 限制到 `min(ctx.max_linear_vel, RUN_FORWARD_MAX)`
  - 挡板进入 ready 角 `100`
- 踢球后：
  - 先按进入踢球模式时记录的 `target_yaw` 回正。
  - yaw 误差小于 `10 deg` 后开始以 `RUN_BACK_VEL = -0.12` 后退。
  - `RUN_AFTER_KICK_MS = 700` ms 后退出踢后阶段，重新允许追球。

对齐状态：核心状态机对齐，速度/时间表达做了当前框架适配。

差异说明：

- Tommy 用 `dy_to_joyy = pid_trace_joyy(ob_y)` 输出摇杆前进量；当前项目直接输出 `target.linear_vel`，用 `CHASE_LINEAR_KP = 0.002` 将 `ob_y` 映射为线速度。
- Tommy 踢后阶段用 `kick_over_count` 循环计数，最大 `50`；当前项目用 `RUN_AFTER_KICK_MS = 700` ms 表达持续时间。两者都是“踢后短时间回正/后退”，但不是逐周期完全一致。
- Tommy 回正使用 `pid_yaw_align(0.8, 0, 0.2, 100000, 100)` 输出 `yaw_to_joyx`；当前项目用 `YAW_ALIGN_KP = 1.2` 直接输出 yaw rate。语义相同，单位和控制链路不同。

## 挡板动作时机

Tommy：

- `ready_kick_ball()`：未满足踢球条件时，前挡板设置到 `100`。
- `kick_ball_core()`：满足踢球条件时，前挡板设置到 `0`。
- 丢球或取消时，前挡板设置到 `0`。

当前项目：

- `ready_kick()`：前挡板设置到 `100`。
- `trigger_kick()`：前挡板设置到 `0`。
- `reset_lost_target()`：前挡板设置到 `0`。
- `cancel_kick()`：前挡板设置到 `0`。

对齐状态：已对齐。

差异说明：当前项目保留 `kicking` 状态和 `KICK_HOLD_MS`，但目前 `KICK_HOLD_MS = 0`，效果接近 Tommy 的“本轮触发，下一轮由条件决定继续踢球或重新 ready”。

## 当前对齐度总结

已对齐：

- 视觉输入含义：`dx` 横向偏差，`dy` 纵向偏差，丢球哨兵值由协议层转换为无效快照。
- 原地踢球条件：`cam_angle < 40 && dy > -10 && dy < 120`。
- 运动踢球追到球条件：`cam_angle < 30 && dy > -5 && dy < 120`。
- 相机 dy 跟踪：`P=0.07`、`D=0.05`、单步限幅 `10`、死区 `10`。
- dx 转向语义：`abs(dx) < 50` 认为瞄准，否则按 `dx` 方向反向转车身。
- 挡板 ready/kick/lost/cancel 时机。
- 丢球后相机回 `60`，并清相机 PD 状态。

等效适配：

- Tommy 的 `dx_to_joyx` 被映射为当前 `target.yaw_rate`。
- Tommy 的 `dy_to_joyy` 被映射为当前 `target.linear_vel`。
- Tommy 的 `kick_over_count` 被映射为当前 `RUN_AFTER_KICK_MS` 时间窗。
- Tommy 的 yaw 回正 PID 被映射为当前 yaw rate 比例控制。

未完全逐行一致但符合当前架构：

- 当前项目不会在动作层直接写电机、LQR 内部状态或全局摇杆变量。
- 当前项目不保留 Tommy 的字符串模式变量 `kick_mode` 和全局 `trace_mode`，而是使用 `mode_id`。
- 当前项目的运动踢球前进速度上限受 `ctx.max_linear_vel` 和 `RUN_FORWARD_MAX` 约束，比 Tommy 的全局摇杆输入链路更受当前平衡核心限制。

## 建议实机验证点

1. 原地踢球进入后，相机应先在 `90` 度附近，丢球后回 `60` 度。
2. 球在画面上方或下方移动时，相机只由 `dy` 驱动，不受 `dx` 影响。
3. 球在画面左侧时，`dx < 0`，车身应向让球居中的方向转；球在右侧时反向。
4. 原地模式下，未满足 `cam_angle < 40 && -10 < dy < 120` 时，挡板应保持 `100`。
5. 原地模式下，满足上述条件后，挡板应打到 `0`。
6. 运动模式下，未追到球时应一边转向一边向前追，挡板保持 ready。
7. 运动模式下，满足 `cam_angle < 30 && -5 < dy < 120` 后应触发踢球，然后进入短暂回正/后退。
8. 视觉丢失时，应停止追踪输出，挡板关闭，相机回 `60`。

## 如果要进一步“严格 Tommy 化”

当前实现已经保留当前项目架构下的等效映射。如果后续希望更接近 Tommy 的数值行为，可以再做三件事：

1. 将运动踢球的 `RUN_AFTER_KICK_MS` 改为按控制循环计数，模拟 `kick_over_count_max = 50`。
2. 将 yaw 回正从简单比例改成带 D 项的局部 PD，对齐 Tommy `pid_yaw_align(0.8, 0, 0.2, 100000, 100)`。
3. 给 `CHASE_LINEAR_KP` 增加一个从 Tommy `pid_trace_joyy` 到当前 `linear_vel` 的明确标定关系，避免只靠经验比例。
