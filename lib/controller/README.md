# Controller 架构

`lib/controller` 分为三层：

- `balance_core.*`：底层 LQR 平衡核心，只开放运动目标、平衡环开关和必要状态快照，在两个 RTOS 任务中完成 FOC IO 和 1 kHz 控制。
- `controller.*` + `actions.*`：上层输入、模式和动作调度，只生成平衡指令和舵机动作。
- `host_comm.*`：上位机输入和状态输出。

详细的函数连接关系、任务链路、数据流和模式切换见 [LOGIC_CHAIN.md](LOGIC_CHAIN.md)。

关键边界：

- 电机 `loopFOC()` 和电机 `move()` 只在 `balance_core::core_task_entry()` 中执行。
- 动作层不访问电机、不写 motor target queue、不修改 LQI 内部状态。
- 上层控制意图统一通过 `balance_core` 的目标/模式 setter 进入平衡核心。
- 队列均按最新快照使用，长度为 1，发布时覆盖旧值。
