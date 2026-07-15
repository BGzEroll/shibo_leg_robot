# 模块对外 API 现状报告

> 生成日期：2026-07-15<br>
> 扫描范围：`lib/controller`、`lib/devices`、`lib/drivers`、`lib/services`、`lib/system` 下全部公共头文件<br>
> 说明：本文描述当前源码中的实际接口和依赖关系，不代表列出的接口都需要立即重构。

## 1. 总体结论

当前工程的 C++ 模块边界已经形成五层：

1. `system` 负责组装模块和创建任务；
2. `services` 负责 HTTP/WebSocket 服务；
3. `controller` 负责输入仲裁、动作语义和控制调度；
4. `devices` 负责板级设备与数据采集；
5. `drivers` 负责总线及具体器件的底层访问。

当前对外 API 的主要特征如下：

- Xbox、Web、HOST 三个输入模块各自拥有 `input` 数据格式，对外均只提供 `peek_input(input &out)`；上层 `input_router` 显式转换，不存在公共输入契约头文件。
- 三路遥控输入内部均采用长度为 1 的最新值队列，生产端 `xQueueOverwrite()`，消费端 `xQueuePeek()`；队列句柄不对外暴露。
- `controller` 是动作语义的主入口，`balance_core` 通过不同请求结构接收运动、直接输出、恢复和反馈覆盖指令。
- `battery`、`motor`、`mpu6050_dev` 已隐藏内部队列，通过各模块自有的类型化 `publish/peek` API 交换最新快照。
- 多个设备模块仍暴露全局可变对象，例如电机、IMU、舵机和板载 LED；这些接口有效，但边界比函数式 API 更宽。
- `actions/action_*.h` 和 `actions/action_common.h` 虽位于公共 include 路径，实际属于动作子系统内部扩展接口，并非普通业务模块入口。

## 2. 当前模块关系

```text
src/main.cpp
    |
    v
system::start_init_all()
    |
    +-- devices / services / controller 初始化
    +-- 创建各 FreeRTOS 任务

xbox_dev::input ----+
web_server::input --+--> input_router --> controller --> actions --> balance_core
host_comm::input ---+                                      |
                                                           +--> motor / sts3032

battery ---------------------> controller / led_dev / rgb_dev
mpu6050_dev / motor encoder --> balance_core
host_comm <------------------- balance_core debug snapshot

web_server --> wifi_dev / xbox_dev / controller calibration API
```

API 按使用范围可分为三类：

| 类别 | 含义 | 典型模块 |
| --- | --- | --- |
| 应用组装 API | 供 `main` 或系统启动层调用 | `system::start_init_all()` |
| 子系统协作 API | 模块之间稳定传递状态或命令 | `peek_input()`、`controller::update()`、`balance_core::apply_*()` |
| 内部扩展 API | 仅供同一子系统实现文件使用，但当前通过头文件可见 | `actions::action`、`action_common`、各动作工厂函数 |

## 3. System 层

### 3.1 `system/start.h`

源码：[start.h](../lib/system/start.h)

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `void system::start_init_all()` | 按固定顺序初始化设备、服务和控制器，并创建全部运行任务 | `src/main.cpp::setup()` |

该模块是当前唯一的系统组装入口。各任务的栈、优先级和核心绑定也集中在其实现中，外部无需分别创建模块任务。

边界状态：**清晰**。公开面只有一个启动入口。

## 4. Services 层

### 4.1 `services/web_server.h`

源码：[web_server.h](../lib/services/web_server.h)

公开数据类型：

```cpp
struct input
{
    uint32_t stream_id;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint16_t held_buttons;
    uint16_t press_count[16];
    float axes[6];
    bool valid;
};
```

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `bool web_server::peek_input(input &out)` | 非破坏性读取当前最新 Web 遥控快照 | `controller::input_router` |
| `bool web_server::init()` | 初始化 WiFi、启动原生 ESP-IDF HTTP Server、注册 HTTP/WS 路由 | `system::start_init_all()` |
| `void web_server::task_entry(void *arg)` | 维护 WiFi 状态、WebSocket 超时和 Xbox 抢占 | System 创建的 Web 维护任务 |

输入语义：

- `stream_id` 标识一次 WebSocket 控制会话；
- `sequence` 标识该会话内的输入快照；
- `held_buttons` 保存当前按住状态；
- `press_count[16]` 锁存按钮按下次数，避免短按在控制周期之间丢失；
- `axes[6]` 保存服务端由 Web 协议 `-1000..1000` 整数转换出的 `-1.0..1.0` 浮点值；
- `valid` 表示当前快照可参与输入仲裁。

实现边界：内部使用私有长度 1 队列保存最新快照，外部只见 `peek_input()`，不会取得队列所有权。

边界状态：**清晰**。HTTP/WS 回调负责校验和发布输入，不直接执行控制动作。

## 5. Controller 层

### 5.1 `controller/controller.h`

源码：[controller.h](../lib/controller/controller.h)

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `bool controller::middle_calibration_success()` | 查询中位校准是否成功 | Web 状态接口 |
| `void controller::mark_middle_calibration_success()` | 标记中位校准完成 | 中位校准动作实现 |
| `bool controller::request_middle_calibration()` | 请求进入中位校准流程 | Web 管理接口 |
| `void controller::update(uint32_t tick_ms)` | 执行一次输入路由、动作状态机和控制请求更新 | `balance_core` 控制任务 |
| `void controller::init()` | 初始化输入路由及动作状态 | `system::start_init_all()` |

`controller` 是业务控制的门面：Web 管理代码只提交校准请求，不直接调用舵机动作；周期控制统一从 `update()` 进入。

边界状态：**清晰**。接口表达业务意图，没有暴露内部状态机对象。

### 5.2 `controller/input_router.h`

源码：[input_router.h](../lib/controller/input_router.h)

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `void controller::input_router::update(mode_id mode, float max_linear_vel, float max_steer_vel, control_input &out)` | 读取三路输入、执行优先级与按钮语义转换，并按当前速度上限产出统一控制意图 | `controller.cpp` |
| `void controller::input_router::init()` | 初始化路由器内部流序号、边沿和超时状态 | `controller::init()` |

输入优先级保持为：

```text
XBOX > WEB > HOST
```

该模块不会要求三个来源共享同一种数据包，而是分别接收：

- `xbox_dev::input`；
- `web_server::input`；
- `host_comm::input`。

随后在路由器内部转换为 `controller::control_input`。因此模块数据格式可以独立演进，而动作层仍只处理统一的业务语义。

边界状态：**合理但属于控制器内部 API**。当前只有 `controller.cpp` 应直接调用。

### 5.3 `controller/host_comm.h`

源码：[host_comm.h](../lib/controller/host_comm.h)

公开数据类型：

```cpp
struct input
{
    uint32_t stream_id;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint16_t buttons;
    uint16_t press_count[16];
    float axes[6];
    bool valid;
};

struct vision_measurement
{
    int16_t dx;
    int16_t dy;
    uint32_t timestamp_ms;
    uint32_t seq;
    bool valid;
};
```

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `bool host_comm::peek_input(input &out)` | 非破坏性读取 HOST 最新遥控输入 | `controller::input_router` |
| `bool host_comm::vision_latest(vision_measurement &out)` | 获取最新视觉目标偏差和时序信息 | 踢球动作 |
| `void host_comm::init()` | 初始化串口、解析状态和输入快照 | `controller::init()` |
| `void host_comm::task_entry(void *arg)` | 接收并解析 HOST 帧，同时发送所需反馈 | System 创建的 HOST 任务 |

遥控输入采用私有长度 1 队列；视觉快照通过模块内部临界区保护。两个通道均没有向上层暴露底层 UART 或同步原语。

边界状态：**清晰**。

### 5.4 `controller/control_input.h`

源码：[control_input.h](../lib/controller/control_input.h)

该头文件只定义控制器内部统一语义，不提供函数：

| 类型 | 内容 |
| --- | --- |
| `input_source` | `NONE`、`XBOX`、`WEB`、`HOST` |
| `action_request` | 停止、启动、坐下、中位校准、各方向跳跃、定点/助跑踢球、退出动作等请求 |
| `control_input` | 来源、时间戳、线速度、偏航、相机/腿高/横滚方向、动作请求及复位/卸力标志 |

它是 `input_router` 到 `controller/actions` 的语义契约，不是三种输入模块的公共数据包契约。

边界状态：**清晰**。类型位于实际翻译边界上。

### 5.5 `controller/actions.h`

源码：[actions.h](../lib/controller/actions.h)

主要公开类型：

- `mode_id`：启动、平衡、坐下、跳跃、停止、踢球、中位校准等模式；
- `jump_command`、`balance_drive_mode`：动作参数枚举；
- `leg_runtime`、`balance_request`、`action_io`：动作执行所需运行数据和 IO；
- `action_enter_params`、`action_result`、`action_state`：状态机输入、输出及持久状态；
- `actions::action`：动作实现的抽象基类；
- `actions::action_runtime`、`actions::phase`：动作内部运行状态。

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `void actions_init(action_state &state)` | 初始化动作状态机 | `controller::init()` |
| `mode_id actions_mode(const action_state &state)` | 查询当前动作模式 | `controller.cpp` |
| `balance_request actions_update(action_state &state, action_io &ctx, uint32_t tick_ms)` | 执行动作状态机的一次更新，并返回本周期平衡核心请求 | `controller::update()` |

边界状态：**功能完整，但公开类型较多**。这是控制器与动作实现之间的子系统契约，不建议设备或服务层直接依赖。

### 5.6 动作扩展头文件

源码：

- [action_common.h](../lib/controller/actions/action_common.h)
- [action_jump.h](../lib/controller/actions/action_jump.h)
- [action_kick.h](../lib/controller/actions/action_kick.h)
- [action_sit.h](../lib/controller/actions/action_sit.h)

`action_common.h` 提供姿态步骤、角度误差、力矩切换、腿部控制、恢复判断和姿态序列执行等共享辅助函数。其余头文件提供动作工厂：

| API | 作用 |
| --- | --- |
| `actions::jump_action()` | 返回跳跃动作单例 |
| `actions::kick_place_action()` | 返回定点踢球动作单例 |
| `actions::kick_run_action()` | 返回助跑踢球动作单例 |
| `actions::sit_action()` | 返回坐下动作单例 |
| `actions::middle_calibration_action()` | 返回中位校准动作单例 |

边界状态：**内部扩展接口**。它们当前可被任意源码包含，但实际只应由动作实现和动作注册代码使用。

### 5.7 `controller/balance_core.h`

源码：[balance_core.h](../lib/controller/balance_core.h)

命令类型：

| 类型 | 用途 |
| --- | --- |
| `motion_control` | 平衡行驶的速度、转向和姿态目标 |
| `direct_output_control` | 动作模式下直接控制轮电机和腿部输出 |
| `recover_control` | 平衡恢复阶段的控制请求 |
| `feedback_override` | 临时覆盖或指定控制反馈 |

状态类型：

| 类型 | 用途 |
| --- | --- |
| `motion_status` | 当前姿态、速度、腿高和运动反馈 |
| `debug_snapshot` | HOST 调试/遥测所需快照 |
| `info` | 核心运行信息和能力状态 |

| API | 作用 | 主要调用方 |
| --- | --- | --- |
| `apply_motion_control(...)` | 提交平衡运动请求 | `controller` |
| `apply_direct_output(...)` | 提交动作直接输出请求 | `controller/actions` |
| `apply_recover_control(...)` | 提交恢复控制请求 | `controller/actions` |
| `apply_feedback_override(...)` | 提交反馈覆盖请求 | `controller/actions` |
| `get_motion_status(...)` | 获取动作决策所需运动状态 | `controller/actions` |
| `get_debug_snapshot(...)` | 获取遥测快照 | `host_comm` |
| `get_info()` | 获取最大线速度和最大转向速度配置 | `controller` |
| `init()` | 初始化核心状态和底层设备 | `system::start_init_all()` |
| `core_task_entry(void *)` | 执行高频 IO/FOC 核心任务 | System 创建的 Core 1 任务 |
| `control_task_entry(void *)` | 执行平衡控制与控制器更新 | System 创建的 Core 0 任务 |

边界状态：**较清晰**。不同控制意图使用不同结构体，调用者不需要直接修改核心内部状态。

## 6. Devices 层

### 6.1 设备 API 总表

| 模块 | 公开类型/对象 | 公开函数 | 主要调用方 | 边界状态 |
| --- | --- | --- | --- | --- |
| [battery](../lib/devices/battery.h) | `battery::data` | `peek_data()`、`init()`、`task_entry()` | controller、LED、RGB、system | 队列私有，只读快照 API |
| [led_dev](../lib/devices/led_dev.h) | `extern led board_led` | `init()`、`task_entry()` | system | 暴露可变驱动对象 |
| [rgb_dev](../lib/devices/rgb_dev.h) | 无 | `init()`、`task_entry()` | system | 生命周期接口清晰 |
| [motor](../lib/devices/motor.h) | `encoder_data`、`target_data`、`left`、`right` | 编码器/目标 `publish`、`peek`、`init()` | balance_core | 队列私有，保留两个电机对象 |
| [mpu6050_dev](../lib/devices/mpu6050_dev.h) | `data`、`extern imu` | `publish_data()`、`peek_data()`、`init()` | balance_core | 队列私有，保留 IMU 对象 |
| [ptk7350](../lib/devices/ptk7350.h) | 相机/前舵机范围宏、两个舵机对象 | 无 | controller/actions | 直接暴露舵机对象 |
| [sts3032](../lib/devices/sts3032.h) | ID/位置范围宏、`status_data`、`status[2]` | 查询、力矩、设定、移动、校准、初始化 | balance_core/actions | 函数 API 与可变状态并存 |
| [wifi_dev](../lib/devices/wifi_dev.h) | 无 | 状态、配网、IP、低延迟、更新、初始化 | web_server | 面向 Web 服务的窄接口 |
| [xbox_dev](../lib/devices/xbox_dev.h) | `input`、`ble_device` | `peek_input()`、连接/地址/扫描/选择、初始化、任务入口 | input_router、web_server、system | 输入队列私有，管理 API 完整 |

### 6.2 `battery`

`battery::data` 包含时间戳、电压、有效标志和低电量标志。生产任务通过内部长度 1 队列发布最新值，对外只提供非阻塞读取：

```cpp
bool battery::peek_data(data &out);
```

调用者不再接触 `QueueHandle_t`、队列长度或 `xQueuePeek()`。队列未初始化或还没有数据时返回 `false`。

### 6.3 `motor`

公开两类模块数据：

- `encoder_data`：左右编码器角度和时间戳；
- `target_data`：左右电机目标值和时间戳。

数据交换接口为：

- `publish_encoder()`、`peek_encoder()`；
- `publish_target()`、`peek_target()`。

内部仍使用两条长度 1 队列执行最新值覆盖与非破坏性读取，但队列句柄不再暴露。`motor::left`、`motor::right` 两个 `BLDCMotor` 对象按当前设计继续公开，供 `balance_core` 执行 FOC、使能和移动操作。

### 6.4 `mpu6050_dev`

`mpu6050_dev::data` 提供温度、加速度、角速度和姿态角；`publish_data()` 发布最新采样，`peek_data()` 非破坏性读取最新快照。

内部采样队列已隐藏，`imu` 底层驱动实例按当前设计继续公开，由 Core 1 高频任务保持原有更新时序。

### 6.5 `wifi_dev`

公开 API 包括：

- `station_connected()`、`config_portal_active()`；
- `connect_and_save(...)`、`station_ip(...)`；
- `set_low_latency_mode(bool)`；
- `update()`、`init()`。

其主要调用者是 `web_server`。WebSocket 取得控制权时提交低延迟请求，由 WiFi 维护任务关闭 WiFi Sleep；释放控制权 3 秒后再由同一任务恢复原设置，期间重新建连会取消恢复。

### 6.6 `xbox_dev`

公开 `input` 是 Xbox 模块自己的数据包：

- `stream_id`、`sequence`、`timestamp_us`；
- `buttons`、`press_count[16]`；
- 归一化浮点轴值 `axes[6]`；
- `valid`。

除 `peek_input()` 外，还公开 BLE 扫描、目标地址读写和连接状态，供 Web 管理页与系统任务使用。

实现边界：遥控输入队列保持私有，管理能力通过明确函数暴露。

## 7. Drivers 层

### 7.1 总线封装

#### `drivers/bus/i2c_bus.h`

源码：[i2c_bus.h](../lib/drivers/bus/i2c_bus.h)

| API | 作用 |
| --- | --- |
| `i2c_bus(uint8_t bus_id)` | 绑定指定 I2C 控制器 |
| `init()` | 初始化总线 |
| `read_bytes(...)` | 执行带长度检查的读取 |
| `write_bytes(...)` | 执行带长度检查的写入 |
| `get_TwoWire_handle()` | 在必要时取得 Arduino `TwoWire*` 原始句柄 |

`i2c_result` 明确区分无效总线、未初始化、NACK、超时、短读写等结果。原始句柄是有意保留的逃生接口，供必须接入第三方 Arduino 驱动的场景使用。

#### `drivers/bus/uart_bus.h`

源码：[uart_bus.h](../lib/drivers/bus/uart_bus.h)

| API | 作用 |
| --- | --- |
| `uart_bus(uint8_t bus_id)` | 绑定指定 UART 控制器 |
| `init(...)` | 初始化波特率和引脚 |
| `read_bytes(...)` | 读取串口数据并返回实际长度 |
| `write_bytes(...)` | 写入串口数据并检查短写 |
| `get_HardwareSerial_handle()` | 取得 `HardwareSerial*` 原始句柄 |

`uart_result` 明确区分无效参数、未初始化和短写等错误。

边界状态：**清晰且保留必要逃生口**。

### 7.2 器件驱动

| 驱动 | 公开 API | 当前上层 |
| --- | --- | --- |
| [led](../lib/drivers/led.h) | 构造、`init()`、`on()`、`off()`、`toggle()` | `led_dev` |
| [mpu6050](../lib/drivers/mpu6050.h) | 构造、`init()`、`update()`，以及公开的温度/加速度/角速度/姿态字段 | `mpu6050_dev` |
| [pwm_servo](../lib/drivers/pwm_servo.h) | 构造、`set_angle()` | `ptk7350` |
| [xbox](../lib/drivers/xbox.h) | 初始化、更新、振动、连接状态，以及公开按钮/轴状态 | `xbox_dev`，部分按钮常量被 `input_router` 使用 |

值得注意的依赖：

- `mpu6050` 和 `xbox` 采用“调用 `update()` 后读取公开字段”的驱动风格；
- `pwm_servo` 在构造期间完成 PWM 配置，设备模块通过全局对象完成静态初始化；
- `input_router` 使用 `xbox` 驱动声明的按钮位常量，控制器语义仍对底层驱动常量存在一处直接依赖。

## 8. 三路输入 API 对比

| 项目 | Xbox | Web | HOST |
| --- | --- | --- | --- |
| 模块自有类型 | `xbox_dev::input` | `web_server::input` | `host_comm::input` |
| 读取 API | `peek_input()` | `peek_input()` | `peek_input()` |
| 内部保存 | 长度 1 队列 | 长度 1 队列 | 长度 1 队列 |
| 发布方式 | `xQueueOverwrite()` | `xQueueOverwrite()` | `xQueueOverwrite()` |
| 消费方式 | `xQueuePeek()` | `xQueuePeek()` | `xQueuePeek()` |
| 时序标识 | `stream_id + sequence` | `stream_id + sequence` | `stream_id + sequence` |
| 按键状态 | `buttons` | `held_buttons` | `buttons` |
| 短按保存 | `press_count[16]` | `press_count[16]` | `press_count[16]` |
| 轴值格式 | `float[6]` | 协议整数转换为 `float[6]` | 协议整数转换为 `float[6]` |
| 额外状态 | 连接状态通过 `connected()` 单独查询 | Web 会话状态由服务内部管理 | 视觉数据通过独立 API 查询 |
| 上层转换 | `input_router` | `input_router` | `input_router` |

三者统一的是“最新快照读取语义”，而不是数据结构。这样既保留了 `Overwrite/Peek` 的一致性，也允许各来源添加自身字段。

## 9. 边界状态汇总

### 9.1 已经收紧的接口

- `system` 只有单一组装入口；
- Xbox、Web、HOST 输入队列均为模块私有；
- 三路输入统一使用 `peek_input()`，但数据包归各模块所有；
- `input_router` 集中处理来源优先级、时序、按钮和轴映射；
- `controller` 通过请求函数承接 Web 校准操作；
- `balance_core` 使用请求结构和只读查询函数，而不是公开内部状态；
- `battery`、`motor`、`mpu6050_dev` 使用类型化 `publish/peek` API，内部队列句柄不再跨模块暴露；
- I2C/UART 总线封装返回明确的结果枚举。

### 9.2 当前仍较宽的接口

| 类型 | 位置 | 影响 |
| --- | --- | --- |
| 全局可变硬件对象 | `led_dev`、`motor`、`mpu6050_dev`、`ptk7350` | 调用者可绕过设备模块规则直接操作硬件 |
| 全局可变状态 | `sts3032::status` | 状态更新与读取责任依赖约定 |
| 驱动公开数据字段 | `mpu6050`、`xbox` | 上层需要了解“先 update、后读取字段”的生命周期 |
| 驱动常量向控制层泄漏 | Xbox 按钮位常量 | `input_router` 对具体 Xbox 驱动头文件存在直接依赖 |
| 内部动作接口全局可见 | `actions/action_*.h` | 编译层面无法阻止其他模块误用动作内部辅助函数 |

这些接口目前均有实际调用者，不能仅通过删除声明完成收紧；若后续调整，需要同步迁移对应消费代码。

## 10. 后续可选收紧顺序

若后续继续统一模块边界，建议按影响范围从小到大处理：

1. 若后续重新考虑对象暴露边界，再评估是否把电机目标应用和 IMU 快照构造下沉到设备模块；
2. 将 `ptk7350` 和 `sts3032` 的可变对象/状态改为动作意图函数或只读快照；
3. 将 Xbox 按钮位定义提升到 `xbox_dev` 自己的接口，解除 `input_router` 对具体驱动头文件的依赖；
4. 通过目录或私有头文件约定标明动作扩展 API，避免被服务层和设备层直接包含。

上述项目是边界优化选项，不是当前功能缺陷。三路输入 API 已经符合“模块自有数据格式、上层统一使用”的当前设计目标。

## 11. 结论

当前工程并不是所有模块都采用同一种 API 形态，而是处于两种边界并存的状态：

- 新整理的遥控输入、控制器和总线层，倾向于窄函数 API、模块自有类型和显式转换；
- 设备采集与执行层已经隐藏内部队列，但按当前设计保留全局硬件对象和部分公开状态。

从现有调用链看，最稳定的公共边界是：

```text
各输入模块 peek_input()
    -> input_router 语义转换
    -> controller/actions 决策
    -> balance_core 请求 API
```

后续如继续重构，建议保持这条主链和当前设备快照语义不变；是否进一步收紧全局硬件对象，应作为独立边界决策处理，而不是重新暴露队列或再次合并三路输入的数据结构。
