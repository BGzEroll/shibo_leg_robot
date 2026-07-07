# balance_core API 开放情况报告

日期：2026-07-07

本文档基于当前源码检查 `lib/controller/balance_core.h`、`lib/controller/balance_core.cpp` 以及调用点，说明 `balance_core` 当前实际开放了哪些 API、上层如何使用、哪些部分符合“底层平衡核心只开放必要能力”的目标，以及哪些地方仍然偏宽。

## 总结

当前 `balance_core` 的边界总体是可控的：电机 FOC、编码器、IMU、LQI 状态、反馈增益、积分项和电机目标队列都没有直接开放给动作层；上层主要通过 `set_target()`、`set_command()`、`get_status()`、`get_info()` 和两个任务入口与平衡核心交互。

但从 API 收口角度看，它还不是严格窄接口。主要问题有三个：

1. `command` 结构体完整公开，包含 `suppress_*`、`recover_active`、`output_blend`、`direct_output` 等偏底层控制细节。
2. `status_snapshot` 完整公开，除了上层动作真正需要的姿态、速度、腿高外，还公开了 `feedback_vector`、`output` 等调试/内部求解信息。
3. `balance_core::init()` 实际被 `controller.cpp` 调用，但没有写进 `balance_core.h`，目前靠 `controller.cpp` 里的局部前向声明使用，接口归属不够干净。

如果当前目标是继续调踢球、跳跃、坐下等动作，不建议立刻大改 API；这些公开字段对调行为很方便。若后续想做模块边界整理，优先处理 `init()` 声明一致性，再考虑把 `command` 和 `status_snapshot` 分层。

## 当前正式公开内容

正式公开位置：`lib/controller/balance_core.h`

### 公开数据结构

| 类型 | 当前字段 | 用途判断 |
| --- | --- | --- |
| `balance_core::target` | `linear_vel`、`yaw_rate`、`direct_left`、`direct_right` | 上层动作给底层的运动目标。`linear_vel` 和 `yaw_rate` 是正常平衡控制目标；`direct_left/right` 只在直通输出模式下使用。 |
| `balance_core::command` | `enable_motor`、`enable_balance`、`enable_steering`、`reset_reference`、`reset_yaw_integral`、`direct_output`、`suppress_linear_feedback`、`suppress_yaw_feedback`、`suppress_yaw_integral`、`recover_active`、`output_blend` | 上层动作给底层的控制命令。这里同时包含必要开关和底层算法细节，是当前最宽的一组 API。 |
| `balance_core::status_snapshot` | 姿态、速度、参考值、输入、反馈向量、输出、roll、腿高 | 底层向上层和调试输出提供状态快照。动作层实际只用其中一部分，host 调试代码会取更多字段。 |
| `balance_core::info` | `max_linear_vel`、`max_steer_vel`、`wheel_radius` | 底层向上层公开的常量/限制信息，目前用于手柄输入缩放和动作限幅。 |

### 公开函数

| 函数 | 当前调用者 | 作用 |
| --- | --- | --- |
| `set_target(const target &target)` | `controller::update()` | 写入线速度、yaw rate 或直通输出目标。 |
| `set_command(const command &command)` | `controller::update()` | 写入电机、平衡、转向、恢复、反馈抑制等控制命令。 |
| `get_status(status_snapshot &out)` | `controller::update()`、`host_comm::send_status()` | 从长度为 1 的状态队列读取最新状态快照，非阻塞。 |
| `get_info()` | `controller::init()` | 读取 LQI 限幅和轮半径等固定信息。 |
| `core_task_entry(void *arg)` | `start.cpp` | 高频 IO 任务入口，负责 FOC、move、编码器、IMU 采样发布。 |
| `control_task_entry(void *arg)` | `start.cpp` | 控制任务入口，目前 `period_ms = 2`，每周期先跑 `controller::update(2)`，再跑 `control_step(2)`。 |

## 当前实际但非正式公开内容

`balance_core::init()` 在 `balance_core.cpp` 中定义，并在 `controller.cpp` 中通过局部前向声明调用：

```cpp
namespace balance_core
{
    void init();
}
```

这说明它事实上是模块启动 API，但没有出现在 `balance_core.h`。这不是功能 bug，当前能编译运行；但从接口管理角度看，它是一个“半公开 API”。建议二选一：

- 若 `controller::init()` 仍负责初始化平衡核心，就把 `void init();` 补进 `balance_core.h`，让启动 API 正式化。
- 若希望 `balance_core` 完全由系统启动层管理，就把初始化调用迁到 `start.cpp` 附近，并调整调用顺序。不过这会碰启动链路，短期不建议为了洁癖动它。

当前更稳的选择是第一种：正式声明 `init()`。

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
        -> balance_core::get_status()
        -> actions_update()
        -> balance_core::set_target()
        -> balance_core::set_command()
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

这个结构的优点是清楚的：动作层不直接碰电机和 LQI 内部状态，`balance_core` 仍然是电机输出的唯一拥有者。

## 上层实际使用了哪些字段

### target

动作层当前使用：

- `linear_vel`：平衡、跳跃、踢球等模式写入前后速度目标。
- `yaw_rate`：平衡、跳跃、踢球等模式写入转向目标。
- `direct_left/right`：坐下流程里配合 `direct_output` 直接给左右电机小输出。

评价：`linear_vel`、`yaw_rate` 是核心必要接口；`direct_left/right` 是特殊动作需要的旁路接口，虽然偏底层，但目前有实际调用。

### command

动作层当前使用：

- `enable_motor`：决定电机是否使能和是否输出 0。
- `enable_balance`：决定是否运行 LQI 平衡输出。
- `enable_steering`：决定 yaw 参考和 yaw 积分是否工作。
- `reset_yaw_integral`：跳跃等动作阶段清 yaw 积分。
- `direct_output`：坐下流程直通左右电机输出。
- `suppress_linear_feedback`、`suppress_yaw_feedback`、`suppress_yaw_integral`：跳跃等动作阶段压制部分反馈。
- `recover_active`、`output_blend`：恢复阶段只保留 pitch 相关反馈，并渐进放大输出。

评价：这里是当前 API 最宽的地方。它不是“坏”，因为动作层确实在用这些能力；但它已经不是单纯的“开关平衡环/设置目标值”，而是允许上层选择 LQI 反馈向量、积分项和输出混合方式。

### status_snapshot

动作层当前主要使用：

- `pitch_angle`、`pitch_rate`：恢复、坐下、跳跃落地判断。
- `yaw_angle`：跳跃转向和踢球后回正。
- `roll_angle`：腿部 roll 补偿。

host 调试代码还读取：

- `avg_linear_pos`、`avg_linear_vel`
- `yaw_rate`
- `input[0]`、`input[1]`
- `feedback_vector[4]`
- `leg_height[0]`、`leg_height[1]`

不过 `host_comm::send_status()` 当前函数开头直接 `return;`，因此这些 host 状态帧代码目前是保留代码，不会实际发出。

评价：动作层真正必要的状态比 `status_snapshot` 当前公开字段少很多。`feedback_vector`、`output` 更像调试/诊断信息，不适合作为长期稳定 API 承诺。

### info

当前使用：

- `max_linear_vel`：手柄线速度缩放、动作限幅、判断手动线速度输入是否明显。
- `max_steer_vel`：手柄 yaw 缩放、踢球视觉转向叠加后的限幅。
- `wheel_radius`：当前没有看到上层实际使用。

评价：`max_linear_vel` 和 `max_steer_vel` 是必要信息；`wheel_radius` 暂时更像预留/调试信息。

## API 收口程度评价

### 做得比较好的地方

1. `motor::left/right` 没有开放给动作层，动作层不会直接 `move()`。
2. `lqi::state`、`lqi::ref`、`lqi::integral`、`feedback_gain` 都留在 `balance_core.cpp` 内部路径中。
3. 跨任务状态用长度为 1 的快照队列，`get_status()` 非阻塞读取，避免上层持有底层内部引用。
4. `controller::update()` 是动作层到 `balance_core` 的唯一写入口，调用点集中。

### 仍然偏宽的地方

1. `command` 是底层求解细节直通上层，动作层可以直接影响反馈向量和积分项。
2. `target` 同时承担正常运动目标和 direct output 目标，语义混在一个结构里。
3. `status_snapshot` 把控制诊断字段和动作决策字段放在一起，容易让上层逐渐依赖内部细节。
4. `init()` 未在头文件声明，但被外部模块使用，接口边界不一致。
5. 任务入口直接公开在 `balance_core.h` 是 FreeRTOS 项目里的常见写法，但它也意味着系统层知道 `balance_core` 的任务拆分方式。

## 建议

### 短期建议

短期只建议做一个小整理：

```cpp
void init();
```

把它补进 `balance_core.h`，让当前已经存在的调用变成正式 API。这个改动不改变行为，风险很低。

除此之外，当前踢球和动作调试还在进行，不建议马上收窄 `command` 或拆 `status_snapshot`。现在这些字段虽然偏宽，但能帮助快速调动作。

### 中期建议

等动作行为稳定后，可以考虑把 API 分成三层：

1. 运动控制层：`set_motion_target(linear_vel, yaw_rate)`、`set_balance_enabled()`、`set_motor_enabled()`。
2. 动作特权层：给跳跃、坐下、恢复使用的 `set_recover_mode()`、`set_direct_output()`、`reset_reference()` 等。
3. 调试层：单独的 `debug_snapshot`，包含 `feedback_vector`、`output` 等不承诺稳定的字段。

这样普通动作不会随手碰到底层求解细节，特殊动作仍然保留必要能力。

### 不建议现在做的事

1. 不建议把 `command` 一次性全拆掉。跳跃、坐下、恢复、踢球都已经依赖其中一部分字段，贸然拆容易引入行为偏差。
2. 不建议把 `status_snapshot` 立刻砍到只剩动作层字段。host 调试虽然现在没发，但这些字段对后续排查平衡和动作问题仍然有价值。
3. 不建议把任务入口藏得太深。`start.cpp` 现在直接创建两个任务，任务归属清楚，先保持这个结构更稳。

## 结论

如果问题是“当前 `balance_core` API 有没有开放出来、够不够上层用”，答案是：够用，而且调用点集中，当前项目的动作层已经能通过它完成平衡、坐下、跳跃、踢球和调试状态读取。

如果问题是“它是否已经达到理想的窄接口”，答案是：还没有。现在的 API 是“工程可用、调试友好、边界略宽”的状态。真正需要优先修的不是功能，而是接口归属一致性：`init()` 应该正式声明；之后再等动作稳定，逐步把 `command` 和 `status_snapshot` 分层。
