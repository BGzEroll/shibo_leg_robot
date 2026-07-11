# 模块化架构重构说明

日期：2026-07-11

本文档说明当前代码为模块化和可扩展性完成的第一版架构重构。此次重构保留原有控制周期、任务优先级、任务 Core、硬件引脚、通信协议、控制参数和动作时序，重点调整模块装配与依赖方向。

## 1. 重构目标

本轮解决四个中心耦合问题：

1. `input_router` 直接依赖 Xbox、HTTP 和 Host，增加输入源必须修改路由器。
2. `actions` 通过中心 `switch` 查找动作，增加动作必须修改调度实现。
3. 动作实现直接调用 STS3032 和 PWM 舵机，行为逻辑无法脱离硬件测试。
4. `start.cpp` 同时保存模块初始化顺序和全部任务参数，机器人配置没有独立装配位置。

目标形态是“编译期组件注册 + 稳定端口 + robot profile 装配”，不引入动态插件加载和跨总线统一基类。

## 2. 当前模块关系

```text
start.cpp
    |
    v
application::robot_profile
    |
    +-- 注册 input_provider[]
    +-- 注册 actions::action[]
    +-- 注入 actuator_port
    +-- 注入 motion_port
    +-- 初始化具体模块
    +-- 提供 task_descriptor[]
             |
             v
        controller
        /        \
input_provider   actions registry
                     |
                     v
               actuator_intent
                /            \
       actuator_port       motion_port
             |                  |
             v                  v
      robot_actuator     balance_motion_adapter
             |                  |
      STS3032 / PWM        balance_core / LQI
```

依赖规则：

- `controller` 只依赖输入、运动和执行器契约。
- `application` 选择具体实现并完成装配。
- 具体设备和 `balance_core` 不再由动作模块直接访问。
- I2C/UART 继续保留各自的具体轻量封装。

## 3. Robot Profile

文件：

- `lib/application/robot_profile.h`
- `lib/application/robot_profile.cpp`

`robot_profile` 是当前机器人的 composition root，集中保存：

- 启用的输入源及其优先级；
- 启用的动作对象；
- 当前执行器适配器；
- 当前运动控制适配器；
- FreeRTOS 任务入口、名称、栈、优先级和 Core；
- 模块初始化顺序。

`start.cpp` 不再 include 电池、Xbox、HTTP、控制器等具体业务模块。它只调用：

```text
application::init()
application::task_list()
```

任务创建结果通过 `configASSERT()` 检查，避免创建失败后静默运行半套系统。

## 4. 输入 Provider

契约：`lib/controller/input_provider.h`

具体适配器：`lib/application/input_providers.cpp`

统一接口：

```text
source()
claims_control(now_us)
latest(input_sample)
```

当前注册顺序：

```text
Xbox > Web > Host
```

输入优先级行为保持不变：

- Xbox 已连接时占用控制权，即使其最后数据已经超时；
- Web 只有在最新输入仍处于 250 ms 有效期时才占用控制权；
- Host 是最后后备来源；
- 最终新鲜度判断仍由 `input_router` 统一完成。

新增输入源时，实现 `controller::input_provider`，然后在 `robot_profile.cpp` 的 `INPUT_PROVIDERS` 中注册即可，不需要修改 `input_router.cpp`。

## 5. 动作注册表

原 `action_for_mode()` 使用固定 `switch`。现在调度器遍历 profile 注册的动作对象，并通过 `action::mode()` 查找目标动作。

当前动作表位于 `robot_profile.cpp`：

```text
BOOT
BALANCE
SIT
MIDDLE_CALIBRATION
JUMP
KICK_PLACE
KICK_RUN
STOP
```

新增动作仍需要为其分配稳定 `mode_id`，但不需要再修改 `action_for_mode()`。动作是否进入某个机器人固件版本，由 profile 决定。

## 6. Actuator Intent 与执行器端口

契约：

- `lib/controller/actuator_intent.h`
- `lib/controller/actuator_port.h`

具体适配器：

- `lib/application/robot_actuator.h`
- `lib/application/robot_actuator.cpp`

动作层现在只写入：

```text
leg_pose_intent
leg_torque_intent
accessory_intent
calibrate_middle
```

动作层不再直接调用：

```text
sts3032::set()
sts3032::move()
sts3032::set_torque_switch()
ptk7350::cam_servo
ptk7350::frontier_servo
```

`robot_actuator` 将意图转换为当前机器人的具体设备调用。以后替换腿部舵机、增加仿真执行器或添加命令记录器时，可以更换 `actuator_port` 实现，而不修改动作代码。

机械范围参数被移动到 `controller::robot_model`，不再由 STS3032/PTK7350 设备头文件向动作层泄漏。

中位判断现在读取 `actuator_feedback`，由当前适配器提供 STS3032 最新状态。反馈来源与具体设备类型对动作层不可见。

## 7. Motion Port

契约：`lib/controller/motion_port.h`

具体适配器：

- `lib/application/balance_motion_adapter.h`
- `lib/application/balance_motion_adapter.cpp`

控制器只使用：

```text
latest_status()
info()
apply(balance_request)
init()
```

`balance_motion_adapter` 负责把稳定的 `controller::balance_request` 转换为：

```text
balance_core::motion_control
balance_core::direct_output_control
balance_core::recover_control
balance_core::feedback_override
```

这使 `controller.cpp` 不再 include `balance_core.h`。以后可以增加测试运动控制器、仿真控制器或另一套算法适配器，并由 profile 选择。

当前 `balance_core` 内部仍然持有 LQI、设备初始化和两个任务入口。这是下一阶段可以继续拆分的边界，本轮没有改动 LQI 计算和实时任务行为。

## 8. 新增扩展的操作方式

### 新输入源

1. 实现 `input_provider`；
2. 在 profile 中按优先级注册；
3. 不修改输入路由器。

### 新动作

1. 实现 `actions::action`；
2. 提供静态动作对象访问函数；
3. 添加稳定 `mode_id`；
4. 在 profile 的 `ACTIONS` 中注册；
5. 不修改动作对象查找器。

### 更换执行器

1. 实现 `actuator_port`；
2. 消费现有 `actuator_intent`；
3. 在 profile 注入新实现；
4. 不修改动作模块。

### 更换运动控制器

1. 实现 `motion_port`；
2. 接收 `balance_request` 并发布 `motion_status`；
3. 在 profile 注入新实现；
4. 不修改动作和上层控制器。

### 修改任务配置

只修改 profile 的 `TASKS` 表，不修改 `start.cpp`。

## 9. 行为保持边界

本轮没有修改：

- GPIO 和总线编号；
- UART/I2C 波特率及频率；
- 控制周期和任务优先级；
- LQI 参数和输出方向；
- 动作阶段、阈值和等待时间；
- 输入优先级和 250 ms 超时；
- HTTP 路由和协议帧；
- 电池阈值。

一处实现层变化：SIT 中位判断不再主动同步读取 STS3032，而是使用 `balance_core` 周期更新的最新执行器反馈，因此判断数据最多约 20 ms 旧。这样动作层不再拥有设备读取权，也减少了 SIT 准备阶段的高频 UART 轮询。

中位校准内部原有的阻塞 `delay()` 仍然存在，本轮只迁移调用边界，没有改变校准时序。

## 10. 后续演进

推荐后续按以下顺序继续：

1. 把 `balance_core` 的纯 LQI 求解提取为不依赖 FreeRTOS/硬件的算法对象；
2. 将 STS3032 收发移动到唯一所有者任务，执行器反馈改为带时间戳快照；
3. 将连续输入与离散动作事件分成不同契约；
4. 增加 safety supervisor 和不可绕过的最终输出许可；
5. 增加 native test profile，注入 fake input/motion/actuator；
6. 根据产品版本增加 `standard/debug/no_wifi` 等 profile。

当前版本已经建立真实可替换边界，但没有为了“看起来统一”给所有设备、总线和服务建立共同基类。
