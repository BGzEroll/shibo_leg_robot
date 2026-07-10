# controller 动作框架虚接口化报告

日期：2026-07-10

本文档记录 controller 动作层从集中 `switch` 状态机迁移到静态虚接口动作框架后的结构、边界、数据流和扩展方式。检查范围包括：

- `lib/controller/actions.h`
- `lib/controller/actions.cpp`
- `lib/controller/actions/action_common.h`
- `lib/controller/actions/action_common.cpp`
- `lib/controller/actions/action_jump.cpp`
- `lib/controller/actions/action_sit.cpp`
- `lib/controller/actions/action_kick.cpp`
- `lib/controller/control_input.h`
- `lib/controller/controller.cpp`

## 总结

动作层已经从“一个 `actions_update()` 内部集中判断所有模式”，调整为“动作对象 + 统一 transition”的框架：

```text
input_router
    -> control_input / action_request
    -> 当前 action::update()
        -> action_result { balance_request, action_request }
    -> actions.cpp 统一 transition
    -> controller.cpp 翻译 balance_request
    -> balance_core
```

实施后的主要结果：

1. 每个动作模式拥有统一生命周期：`enter()`、`update()`、`exit()`。
2. 所有 action 对象静态实例化，不使用动态分配。
3. 动作内部只返回语义请求，不直接切换目标模式。
4. `actions.cpp` 集中维护 STOP、BOOT、低电坐下锁、动作切换等全局 transition。
5. `action_state` 从大运行状态收紧为调度状态。
6. `phase`、`timer`、`jump_runtime`、`kick_runtime` 已迁移到动作私有区域。
7. `controller::update()` 的外部调用形态和 `balance_request` 到 `balance_core` 的翻译保持不变。

## 当前框架类型

### `actions::action`

动作虚接口定义在 `actions.h`：

```cpp
class action
{
    public:
        virtual mode_id mode() const = 0;
        virtual void enter(action_io &ctx, mode_id previous,
            const action_enter_params &params) = 0;
        virtual action_result update(action_io &ctx, uint32_t tick_ms) = 0;
        virtual void exit(action_io &ctx, mode_id next) = 0;
};
```

约定：

- `enter()` 负责初始化当前动作私有状态。
- `update()` 负责推进动作阶段，并返回本周期平衡请求。
- `exit()` 预留给动作收尾，目前多数动作为空实现。
- action 不直接调用模式切换函数。

### `action_result`

动作更新结果包含两部分：

```text
balance_request balance
action_request request
```

`balance` 是本周期输出到 `balance_core` 的控制意图；`request` 是动作层向 transition 层提交的语义请求。transition 在本周期动作输出生成后执行，因此模式切换默认影响下一周期输出。

### `action_enter_params`

当前只携带跳跃方向参数：

```text
jump_command jump
```

普通动作进入时使用默认值。后续如果某个动作确实需要进入参数，应优先扩展该结构，而不是让 action 直接读取外部临时全局变量。

### `action_state`

当前 `action_state` 只保留调度级状态：

```text
mode
current action pointer
sit_exit_locked
```

旧的 `phase`、`timer`、`ready_timer`、`elapsed` 已移动到 `actions::action_runtime`，由具体 action 私有持有或共享。

## 动作对象边界

### `actions.cpp`

`actions.cpp` 现在承担框架内核职责：

- 持有 `boot_action`、`balance_action`、`stop_action` 静态对象。
- 根据 `mode_id` 查找静态动作对象。
- 执行 `switch_action()` 生命周期切换。
- 维护 `sit_exit_locked`。
- 处理全局 STOP 优先级。
- 统一解释 `action_request` 到模式切换。

### `action_jump.cpp`

`jump_action` 持有跳跃私有状态：

- `linear_dir`
- `turn_dir`
- `target_yaw`
- `linear_cmd`
- `yaw_cmd`
- `action_runtime`

跳跃恢复完成后返回 `ACTION_DONE`，由 transition 层切回 `BALANCE`。

### `action_sit.cpp`

`sit_action` 和 `middle_calibration_action` 复用同一个坐下流程 runtime，以保留原有“坐下过程中可进入中位校准”的行为。

坐下退出流程完成后返回 `ACTION_DONE`；如果 `sit_exit_locked` 已经被低电锁定，transition 不允许回到 `BALANCE`。当电池检测有效且恢复到非低电状态后，锁会自动解除。

### `action_kick.cpp`

踢球动作拆成：

- `kick_action_base`：视觉、摄像头、前挡舵机、踢球冷却、退出流程等共用逻辑。
- `kick_place_action`：原地踢球流程。
- `kick_run_action`：运动踢球流程。

`KICK_PLACE` 和 `KICK_RUN` 之间通过语义请求互相切换。`KICK_EXIT` 先进入踢球内部退出等待，等待完成后返回 `ACTION_DONE`，再由 transition 回到 `BALANCE`。

## transition 规则

当前 transition 集中在 `actions.cpp`：

| 当前模式 | 请求 | 结果 |
| --- | --- | --- |
| 任意非 STOP | `STOP` | 立即进入 `STOP`，本周期输出空请求 |
| `BOOT` | `ACTION_DONE` | 进入 `BALANCE` |
| `BALANCE` | `RESET_BALANCE` | 重新进入 `BALANCE` |
| `BALANCE` | `SIT` | 进入 `SIT` |
| `BALANCE` | `MIDDLE_CALIBRATION` | 进入 `MIDDLE_CALIBRATION` |
| `BALANCE` | `JUMP_*` | 带 `jump_command` 参数进入 `JUMP` |
| `BALANCE` | `KICK_PLACE` | 进入 `KICK_PLACE` |
| `BALANCE` | `KICK_RUN` | 进入 `KICK_RUN` |
| `SIT` | `MIDDLE_CALIBRATION` | 进入 `MIDDLE_CALIBRATION` |
| `SIT` / `MIDDLE_CALIBRATION` | `ACTION_DONE` | 未低电锁定时进入 `BALANCE` |
| `JUMP` | `ACTION_DONE` | 进入 `BALANCE` |
| `KICK_PLACE` | `KICK_RUN` | 进入 `KICK_RUN` |
| `KICK_RUN` | `KICK_PLACE` | 进入 `KICK_PLACE` |
| `KICK_PLACE` / `KICK_RUN` | `ACTION_DONE` | 进入 `BALANCE` |
| `STOP` | `BOOT` | 未低电坐下锁定时进入 `BOOT` |

这使得 action 文件只表达“发生了什么”，不表达“应该切到哪个目标模式”。

## 扩展新动作的方式

新增动作时推荐按以下步骤：

1. 在 `mode_id` 增加新模式。
2. 如果需要按键或外部触发，在 `action_request` 增加语义请求，并在 `input_router` 里映射。
3. 新建或扩展 `actions/action_xxx.cpp`，实现一个静态 `xxx_action_impl`。
4. 私有阶段、计时器和动作运行状态放在 action 类内部。
5. 对外只暴露 `actions::action &xxx_action()`。
6. 在 `actions.cpp` 的 `action_for_mode()` 注册该静态对象。
7. 在 `apply_transition()` 中集中写清该请求在各模式下是否允许切换。

不建议：

- action 内直接修改 `action_state::mode`。
- action 内直接调用其他 action 的 `enter()`。
- 为动作对象使用 `new/delete`。
- 将动作私有 runtime 塞回 `action_state`。

## 行为保持点

本次是框架重构，不主动调整以下行为：

- 控制周期。
- 动作时序常量。
- 舵机目标位置和速度参数。
- 踢球视觉参数。
- 跳跃推送、飞行、落地、恢复时间。
- 坐下直出电机输出。
- 低电 BOOT 禁止和坐下后禁止退出逻辑；电量恢复后自动解除坐下退出锁。
- `controller.cpp` 到 `balance_core` 的请求翻译。

需要注意的是，框架现在明确采用“当前 action 先生成本周期输出，transition 再影响下一周期”的模型。这个模型保留了原先 KICK 切换时“本周期仍使用 kick base command”的行为。

## 验证

已完成以下检查：

```text
rg -n "begin_mode\\(|state\\.phase|state\\.timer|state\\.jump|state\\.kick" lib/controller
rg -n "actions::update_sit|actions::update_middle_calibration|actions::update_jump|actions::update_kick|actions::kick_base_command|actions::begin_kick_exit" lib/controller
git diff --check
/home/bgzerol/.platformio/penv/bin/pio run
```

结果：

- 旧 `begin_mode()` 和旧集中状态字段访问无残留。
- 旧动作更新函数入口无残留。
- 空白检查通过。
- PlatformIO 构建通过。

最终固件资源占用约为：

```text
RAM:   18.3%
Flash: 38.2%
```

## 后续建议

后续如果继续收紧，可以考虑：

1. 把 `action_io::sit_exit_locked` 从可写字段改成只读语义，减少 action 修改全局锁的可能性。
2. 给 `action_request::EXIT_ACTION` 补上明确使用场景，或在确认长期不用后移除。
3. 如果动作数量继续增加，可以把 transition 表格化，但仍建议保持集中 transition，不做动态注册。
