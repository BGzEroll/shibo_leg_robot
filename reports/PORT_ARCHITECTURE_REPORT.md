# 平级模块端口架构重构报告

日期：2026-07-12

## 结果

项目已经从运行时分层调用结构切换为“平级模块、显式端口、集中组装、确定顺序执行”的架构。

本次为一次性切换，不保留旧队列 getter、旧 `balance_core` setter/getter 或旧 RTOS 任务入口兼容层。硬件引脚、通信协议字节、控制参数、动作时序、任务优先级、任务栈和核心分配保持不变。

## 基础设施

端口基础设施位于 `lib/framework/ports/`：

- `latest_value<T>`：保存一个强类型最新快照，提供独立 reader/writer；
- `event_queue<T, capacity>`：静态分配的不可阻塞事件队列；
- `actuator_port::services`：腿部、摄像头和前挡板使用的同步执行端口；
- `leg_contract`：动作和硬件适配器共同遵守的腿部机械范围契约。

`latest_value<T>` 不使用动态注册、字符串主题、`void *` 或公开 FreeRTOS 队列句柄。跨核读写由端口内部临界区保护。

## 集中组装

所有状态通道都由 `lib/system/system_app.cpp` 静态持有。只有该文件负责：

1. 创建端口存储；
2. 将 reader/writer 分配给模块；
3. 按硬件依赖顺序初始化模块；
4. 把执行器适配服务交给动作模块。

模块不再初始化其他独立模块。`start.cpp` 只调用 `system_app::init()`，然后创建任务。

## 当前数据流

```text
xbox_dev ---------\
esp_http_server ----> input_router -> controller/actions -> balance command
host_comm ---------/                              |              |
                                                   |              v
battery ---------------------------> safety/action |        balance_core
vision ----------------------------> kick action   |          |       |
leg status ------------------------> action/balance|          |       |
                                                   v          v       v
                                            actuator ports  motion  motor target
                                                                  |      |
                                                                  v      v
                                                             host_comm  motor

mpu6050_dev -------------------------------> balance_core
motor encoder -----------------------------> balance_core
```

## 任务和模块分离

模块不再通过拥有任务来决定架构关系。

`system_executor::fast_io_task_entry()` 负责：

- 持续执行左右电机 `loopFOC()`；
- 读取最新电机目标并处理电机 enable/disable；
- 每 1 ms 发布编码器状态；
- 每 5 ms 发布 IMU 状态。

`system_executor::control_task_entry()` 每 2 ms 严格按顺序执行：

1. 采样腿部状态（内部限频为 20 ms）；
2. `controller::update()` 运行输入路由和动作状态机，发布平衡命令；
3. `balance_core::step()` 消费最新命令并运行 LQI，发布电机目标与状态。

原来的 `balance_core::control_task_entry() -> controller::update()` 反向依赖已经删除。

## 已删除的旧接口

- `battery::queue()`；
- `xbox_dev::queue()`；
- `host_comm::remote_queue()`；
- `host_comm::vision_latest()`；
- `esp_http_server::remote_queue()`；
- `motor::encoder_queue()` / `motor::target_queue()`；
- `mpu6050_dev::queue()`；
- `balance_core::apply_*()`；
- `balance_core::get_motion_status()` / `get_debug_snapshot()`；
- `balance_core::core_task_entry()` / `control_task_entry()`；
- HTTP 服务对 controller 校准 API 的直接调用。

仓库内跨任务业务数据不再暴露 `QueueHandle_t`。

## 行为保持项

- 控制周期仍为 2 ms；
- 编码器采样仍为 1 ms；
- IMU 采样仍为 5 ms；
- 腿部状态采样仍为 20 ms；
- FOC IO 任务仍在 core 1，优先级 5，栈 4096；
- 控制任务仍在 core 0，优先级 5，栈 4096；
- 电池、LED、RGB、Xbox、Host、HTTP任务参数保持不变；
- 动作 `enter/update/exit`、集中跳转表和所有动作参数保持不变；
- UART、视觉、HTTP遥控协议格式保持不变。

## 后续扩展规则

新增模块时：

1. 类型放在真正拥有语义的模块或单一契约头文件中；
2. 模块声明 `input_ports` / `output_ports`；
3. 最新状态使用 `latest_value<T>`；
4. 不可丢事件使用 `event_queue<T, N>`；
5. 只在 `system_app.cpp` 创建和连接端口；
6. 实时模块提供 `step()`，由系统执行器规定调用顺序；
7. 不公开原始队列句柄，不主动查找其他模块。

## 验证

已执行：

```text
git diff --check
/home/bgzerol/.platformio/penv/bin/pio run
```

静态构建只能确认接口、链接和资源占用。电机使能切换、平衡、坐下、跳跃、踢球和低电量恢复仍需要按实际机器人顺序进行真机回归。
