# Shibo Leg Robot 项目分析

日期：2026-07-11

本文档基于当前 `main` 分支源码和本机 PlatformIO 构建结果，分析项目定位、启动流程、实时任务、控制链路、模块边界、通信协议、硬件资源以及当前主要风险。结论只针对当前工作区：当前版本仍启用 HTTP/WiFi 控制服务，并未切换为 BLE GATT 配置服务。

## 1. 总体结论

这是一个基于 ESP32、Arduino、FreeRTOS 和 SimpleFOC 的双轮腿式机器人固件。项目已经不再是简单的单文件 Arduino 程序，而是形成了比较明确的四层结构：

```text
输入与服务层
Xbox BLE / 网页遥控 / UART 上位机 / MaixCam 视觉
                         |
                         v
控制与动作层
input_router -> controller -> actions 状态机
                         |
                         v
平衡核心层
balance_core：LQI、参考生成、恢复控制、输出求解
                         |
                         v
设备与驱动层
SimpleFOC 电机 / AS5600 / MPU6050 / STS3032 / PWM 舵机 / 电池 / LED
```

当前版本的主要优点是控制职责已经分层、动作状态机边界清楚、跨任务数据大多使用长度为 1 的覆盖队列，而且输入失联会在 250 ms 后归零。当前最需要补强的不是继续拆模块，而是建立真正的故障闭环：传感器数据新鲜度、总线错误上传、运行中低电保护、控制周期超时监测和网络控制鉴权。

## 2. 技术栈与构建配置

| 项目 | 当前配置 |
| --- | --- |
| MCU/开发板 | ESP32 Dev Module，240 MHz，双核，320 KB RAM，4 MB Flash |
| 框架 | Arduino on ESP-IDF |
| 构建系统 | PlatformIO |
| Platform | `espressif32@6.13.0` |
| Arduino Core | `framework-arduinoespressif32 3.20017`，对应 Arduino-ESP32 2.x 系列 |
| 电机控制 | SimpleFOC 2.2.3 |
| 手柄 | XboxSeriesXControllerESP32_asukiaaa 1.1.1 |
| RGB | FastLED 3.10.3 |
| 分区 | `huge_app.csv`，应用空间大，但通常不保留标准双 OTA 分区 |
| 串口监视 | 230400 baud，启用异常解码器 |

2026-07-11 本机执行 `/home/bgzerol/.platformio/penv/bin/pio run` 成功：

```text
RAM:   18.3% (59940 / 327680 bytes)
Flash: 38.2% (1201121 / 3145728 bytes)
```

资源余量目前充足。Flash 使用率偏低是因为采用了 `huge_app` 分区，不能据此认为 OTA 空间也可用。

## 3. 目录与模块职责

### 3.1 `src/` 与 `lib/system/`

- `src/main.cpp`：Arduino 入口，仅调用 `start_init_all()`，随后删除 Arduino loop 任务。
- `lib/system/start.cpp`：统一初始化设备、服务和控制器，并创建全部 FreeRTOS 任务。

这种入口很薄，系统真实行为不散落在 `setup()`/`loop()` 中，方向是正确的。

### 3.2 `lib/controller/`

- `input_router`：从 Xbox、网页和 UART 中选择输入源，做 250 ms 新鲜度判断、摇杆死区、按钮边沿和语义动作映射。
- `controller`：动作层和平衡核心之间的薄翻译层，同时处理摄像头控制和网页触发的中位校准请求。
- `actions`：动作对象调度器和公共模式切换。
- `actions/action_sit`：坐下、起身和中位校准。
- `actions/action_jump`：原地、前后和转向跳跃。
- `actions/action_kick`：定点踢球、追球踢球和视觉跟踪。
- `balance_core`：LQI 状态、参考生成、积分、腿高增益调度、恢复模式和电机目标发布。
- `host_comm`：UART0 上位机遥控与 MaixCam 视觉帧解析；调试状态发送代码当前被主动禁用。

### 3.3 `lib/devices/`

- `motor`：左右 BLDC、驱动器和 AS5600 编码器组合。
- `mpu6050_dev`：MPU6050 设备实例和最新数据队列。
- `sts3032`：腿部串行舵机协议、位置/负载读取、同步写和扭矩切换。
- `xbox_dev`：NimBLE 扫描、目标地址保存、Xbox 对象生命周期和输入发布。
- `wifi_dev`：STA/AP_STA 模式、NVS WiFi 凭据和配置热点。
- `battery`：ADC 多次采样、电压换算、低电确认和恢复迟滞。
- `led_dev` / `rgb_dev`：普通状态灯和低电红色双闪。
- `ptk7350`：摄像头舵机和踢球挡板 PWM 舵机实例。

### 3.4 `lib/drivers/`

- `bus/i2c_bus`：两个静态 I2C 资源，具有明确的 `i2c_result` 错误码，并保留 SimpleFOC 需要的 `TwoWire *` 句柄。
- `bus/uart_bus`：三个静态 UART 资源，读操作同时返回状态和实际长度。
- `mpu6050`、`xbox`、`pwm_servo`、`led`：具体底层驱动。

I2C 和 UART 保持协议特有的轻量封装是合适的；当前规模不需要跨总线继承层。

### 3.5 `lib/services/`

- `esp_http_server`：板载网页、WiFi 配置、Xbox 扫描选择、网页遥控和舵机中位校准入口。

该文件接近 1000 行，主要体积来自内嵌 HTML/JavaScript。业务边界仍可辨认，但后续继续增加页面时，应考虑把静态页面资源和请求处理分开，避免单文件持续增长。

## 4. 启动流程

当前启动顺序如下：

```text
setup()
  -> start_init_all()
      -> delay(1000)
      -> battery::init()
      -> led_dev::init()
      -> rgb_dev::init()
      -> xbox_dev::init()
      -> esp_http_server::init()
          -> wifi_dev::init()
          -> 注册并启动 WebServer
      -> controller::init()
          -> host_comm::init()
          -> balance_core::init()
              -> STS3032
              -> MPU6050 和零偏采样
              -> 双电机、驱动和编码器
          -> actions/input_router
      -> task_list()
  -> 删除 Arduino loop 任务
```

启动阶段存在多段同步等待：固定 1 秒、Xbox BLE 重建、WiFi 最长 8 秒连接、MPU6050 4000 次零偏预读/采样和 SimpleFOC 对准。设备正常时可以接受，但当前没有启动阶段状态输出、超时分类或失败降级，因此某个硬件异常时不容易判断停在了哪里。

## 5. FreeRTOS 任务模型

| 任务 | Core | 优先级 | 栈 | 周期/行为 | 主要职责 |
| --- | ---: | ---: | ---: | --- | --- |
| `balance_io_task` | 1 | 5 | 4096 | 无阻塞忙循环，仅 `taskYIELD()` | 双电机 FOC，1 ms 编码器快照，5 ms IMU 快照 |
| `balance_ctl_task` | 0 | 5 | 4096 | 2 ms | 输入、动作状态机、LQI 控制、舵机状态读取 |
| `xbox_dev_task` | 0 | 3 | 4096 | 通常 20 ms | Xbox BLE 更新与输入发布 |
| `host_comm_task` | 0 | 3 | 4096 | 1 ms | UART0 接收、帧解析 |
| `battery_task` | 0 | 2 | 2048 | 100 ms | 电池电压采样和低电判定 |
| `led_dev_task` | 0 | 2 | 1024 | 50 ms | 板载 LED |
| `rgb_dev_task` | 0 | 2 | 2048 | 50 ms | 低电红灯双闪 |
| `http_server_task` | 0 | 2 | 4096 | 50 ms | WiFi 维护和 HTTP 请求处理 |

任务划分的核心思路正确：FOC 高频 IO 固定在 core 1，上层控制和外围服务主要在 core 0。需要注意：`balance_io_task` 没有真实阻塞点，可能长期占满 core 1 并挤压该核心的 idle/系统任务；是否触发看门狗或造成系统抖动必须通过真机运行时间和 task runtime stats 验证。

`balance_ctl_task` 的算法 `dt` 固定为 2 ms，而不使用实际周期。只要 WiFi/BLE、UART 或舵机操作造成超期，积分和参考斜坡仍会按 2 ms 计算，实际动态会偏离设计值。当前也没有记录最大执行时间、deadline miss 次数或 stack high-water mark。

## 6. 控制与数据链路

### 6.1 主闭环

```text
Xbox / Web / Host UART
         |
         v
input_router::update()
         |
         v
actions_update() -----> STS3032 腿部动作
         |
         v
controller::apply_balance_request()
         |
         v
balance_core::control_step()
  读取 IMU/encoder/腿高
  更新 LQI 状态与增益
  生成左右电机目标
         |
         v
motor::target_queue()，长度 1，覆盖旧值
         |
         v
balance_io_task -> SimpleFOC move()/loopFOC()
```

反向反馈链路：

```text
AS5600 -> SimpleFOC shaft_angle/velocity -> encoder_queue --+
                                                          |
MPU6050 -> mpu6050_dev::queue ----------------------------+-> balance_core
                                                          |
STS3032 position/load ------------------------------------+
```

队列都按“最新状态”使用，长度为 1 并通过 `xQueueOverwrite()` 更新。这非常适合闭环状态快照，避免旧数据堆积。

### 6.2 输入优先级与失联行为

当前输入优先级是：

```text
Xbox 已连接
    > 网页遥控有效且 250 ms 内有新包
    > UART 上位机输入
```

- 任一输入超过 250 ms 未更新，按钮和轴值都会清零。
- 网页遥控额外有 500 ms 会话租约，避免两个网页同时控制。
- Xbox 报告已连接时，即使其数据队列已经超时，网页和 UART 也不会接管；输出会归零但输入源仍被 Xbox 占用。
- 输入失联只会使移动目标归零，机器人仍保持平衡，不会自动进入 `STOP`。对于短暂无线抖动，这是比直接断电更合理的默认行为。

### 6.3 动作状态机

主要模式：

| 模式 | 作用 |
| --- | --- |
| `BOOT` | 腿部初始化，等待确认，随后恢复站立 |
| `BALANCE` | 常规平衡、移动、转向和腿高/横滚调节 |
| `SIT` | 坐下、保持和起身恢复 |
| `MIDDLE_CALIBRATION` | 坐下后执行腿部舵机中位校准 |
| `JUMP` | 原地、前后和转向跳跃 |
| `KICK_PLACE` | 定点视觉踢球 |
| `KICK_RUN` | 追球、对准和踢球 |
| `STOP` | 关闭电机输出，等待重新进入 BOOT |

动作层只生成 `balance_request`，由 `controller` 翻译为 `balance_core` 的普通运动、直出、恢复和反馈覆盖 API。动作层不直接操作 BLDC，这是当前架构中最有价值的边界之一。

## 7. 通信与协议

### 7.1 UART0 上位机/MaixCam

公共帧头为 `FF AA`，随后是命令、负载长度、负载和校验。

- `0x01`：Xbox 风格输入，负载至少 14 字节，按钮 2 字节加 6 个 `int16_t` 轴值。
- `0x02`：MaixCam 视觉，负载固定 4 字节，分别为 `dx` 和 `dy`；`32767` 表示目标丢失。
- 视觉结果有效期为 350 ms，并带本地递增序号，踢球动作只消费新视觉步。
- 调试状态发送逻辑已实现，但 `send_status()` 开头存在无条件 `return`，当前不会真正发送。

接收解析器能够跳过噪声并保留不完整尾帧，但如果缓冲区即将溢出会直接清空全部旧数据，没有统计溢出、校验失败或重同步次数。

### 7.2 STS3032

UART2 使用 1 Mbps，支持同步读、同步写位置以及扭矩开关。动作层每次目标变化才调用同步写，平衡核心每 20 ms 请求一次位置/负载。

总线封装已经能报告 UART 错误，但 `sts3032` 的公共 API 仍然是 `void`，上层不知道读取是否成功，失败时会继续沿用旧的全局 `status`。

### 7.3 HTTP/WiFi

当前提供：

- WiFi 扫描与配置；
- Xbox 状态、BLE 扫描和目标地址选择；
- 手机网页遥控；
- 舵机中位校准。

WiFi 配置热点为 `SHIBO_LEG_ROBOT`，密码硬编码为 `12345678`。连接路由后切换为 STA 模式，但 HTTP 服务继续监听局域网 80 端口。

## 8. 硬件资源映射

| 功能 | 资源 |
| --- | --- |
| 左电机三相 PWM / EN | GPIO 32、33、25 / 22 |
| 右电机三相 PWM / EN | GPIO 26、27、14 / 12 |
| I2C0 左编码器 | SDA 19，SCL 18，400 kHz |
| I2C1 右编码器 + MPU6050 | SDA 23，SCL 5，400 kHz |
| UART0 上位机 | 230400 baud，默认串口引脚 |
| UART2 STS3032 | 1 Mbps，当前未在项目层显式指定引脚 |
| 摄像头 PWM 舵机 | GPIO 4，LEDC channel 4 |
| 踢球挡板 PWM 舵机 | GPIO 15，LEDC channel 15 |
| 板载 LED | GPIO 13 |
| WS2812 x2 | GPIO 21 |
| 电池 ADC | ADC1 channel 7，即 GPIO 35 |

UART2 依赖 Arduino 默认引脚映射，硬件迁移时容易遗漏。建议把 RX/TX 引脚纳入 `uart_dev` 配置并在一个板级配置文件中集中记录。

## 9. 做得较好的地方

1. **控制边界清晰**：`controller` 是动作意图到 `balance_core` 的唯一翻译层，动作模块不直接写电机。
2. **平衡核心 API 已收紧**：普通动作、直出、恢复、反馈覆盖和调试快照被区分，不再暴露一份可任意修改的巨型命令结构。
3. **最新快照队列适配实时控制**：传感器、输入、电池和目标输出均使用长度为 1 的覆盖队列。
4. **输入失联有基础保护**：250 ms 超时会清零按键和运动轴，网页还有单控制者租约。
5. **总线错误类型明确**：I2C/UART 能区分非法总线、未初始化、参数错误、NACK 和短读写等状态。
6. **电池判断有消抖和迟滞**：低电和恢复采用不同阈值及连续确认，避免阈值附近反复跳变。
7. **动作对象为静态实例**：控制热路径没有反复 `new/delete` 动作对象。
8. **项目代码风格整体统一**：命名、定宽类型、中文 Doxygen 和模块章节划分已经比较一致。

## 10. 风险与问题优先级

### P0：建议在继续调参前处理

#### 10.1 传感器失联不会触发安全退出

`balance_core::read_sensor()` 只要队列中曾经有过数据，就把它视为有效，没有检查 IMU 和编码器时间戳。MPU6050 驱动又忽略了 `i2c_result`，即使 I2C 短读/NACK，也会继续处理旧 `raw` 并用新的时间戳发布。

后果是传感器拔线、总线卡死或采样任务异常时，控制环可能继续使用冻结或错误状态输出电机，而外部看起来状态时间戳仍在更新。

建议：

1. 让 MPU6050 `update()` 返回明确状态，失败时不更新时间戳；
2. 在 `balance_core` 分别检查 IMU、encoder 和腿部反馈年龄；
3. 超过短阈值时冻结积分并输出零，超过长阈值时关闭电机并记录故障码；
4. 故障恢复必须显式重新进入 BOOT，不要自动瞬间恢复输出。

#### 10.2 STS3032 回包解析边界不足

`parse_sync_read()` 初始只检查 `offset + 7 <= len`，但随后会访问 `offset + 10`，并根据未校验的 `frame_len` 访问校验字节。异常短包或被破坏的长度字段可能引起栈缓冲区越界读取。

建议先验证：

- 至少存在完整头、长度和状态字段；
- `frame_len` 等于当前命令预期值；
- `offset + frame_len + 4 <= len`；
- 读取位置/负载前确认对应字段均在帧内。

解析结果应由 `get_position_and_load()` 返回给 `balance_core`，失败时不能静默继续把旧位置当作新位置。

#### 10.3 HTTP 控制接口没有鉴权

所有页面和 API 都是明文 HTTP，没有身份认证、请求签名或 CSRF 防护。只要进入同一局域网，就可以提交网页遥控、触发舵机校准、改 WiFi 凭据或重选 Xbox 地址；配置 AP 密码还是固定的 `12345678`。

对于能驱动电机和机械机构的设备，这属于控制面安全问题。至少应采用每台设备唯一凭据、受保护的控制会话和物理在场确认；如果后续改为 BLE，也应启用加密配对并限制高风险命令。

### P1：近期应处理

#### 10.4 中位校准阻塞 2 ms 控制任务约 1.1 秒

`sts3032::calibrate_middle()` 内部使用 `delay(100)` 和 `delay(1000)`，它从 `balance_ctl_task` 的动作状态机直接调用。校准发生在坐下并停止 BLDC 后，当前机械风险相对受控，但这会让控制任务整体停顿，所有该任务内的状态更新、故障检查和 deadline 统计都消失。

建议把扭矩切换拆成非阻塞状态机，由现有 `tick_ms` 推进。

#### 10.5 运行中低电只报警，不自动进入安全态

低电状态能阻止 BOOT 启动，也会在 SIT/MIDDLE_CALIBRATION 模式锁住起身；但机器人已经处于 BALANCE、JUMP 或 KICK 时，低电不会自动停止新动作、坐下或进入 STOP。当前实际行为主要是 LED/RGB 告警。

建议定义明确策略：禁止进入高功率动作、先限速，再在可控姿态下坐下；严重欠压则关闭电机。阈值和动作必须经过真机电池内阻/负载跌落实验，不能只凭静态电压决定。

#### 10.6 控制周期使用固定 dt 且无超时监控

控制任务固定传入 2 ms，不测量真实间隔。WiFi/BLE 系统任务、UART flush、STS3032 读写或临界区都可能造成抖动。建议记录实际周期、最大执行时间和 deadline miss，并用实际受限 `dt` 驱动积分和参考生成。

#### 10.7 Xbox 对象跨任务生命周期保护依赖时序

HTTP 任务可以扫描 BLE 或重建 `gamepad`，Xbox 任务同时调用 `gamepad->update()`。当前通过 `volatile ble_scan_active` 加 60 ms 延迟降低冲突概率，但这不是严格同步；任务通过标志检查后仍可能与另一个任务删除对象交错。

建议把扫描、换目标和连接重建变成发给 Xbox 任务的命令，由 Xbox 任务独占 NimBLE/Xbox 对象生命周期。

#### 10.8 任务和队列创建结果未检查

`xTaskCreatePinnedToCore()`、`xQueueCreate()`、总线初始化、FOC 初始化和设备配置大多没有检查返回值。低内存或硬件初始化失败时，系统可能带着部分模块继续运行。

建议启动时汇总 `init_result`，只有关键模块全部就绪才允许进入 BOOT 确认阶段。

### P2：维护性与可观测性

#### 10.9 高频 IO 任务可能饿死 core 1 idle

`balance_io_task` 是优先级 5 的无限循环，仅调用 `taskYIELD()`。应在真机检查 core 1 idle、任务看门狗、FOC 实际频率和 CPU runtime；必要时用精确微秒节拍或能保证系统任务运行的调度方式替代纯忙循环。

#### 10.10 总线错误码尚未贯穿设备层

总线层已经完成错误返回，但 `motor`、`mpu6050` 和多数 `sts3032` 写路径仍忽略结果。这使封装层的改进还没有变成系统级诊断能力。下一步应先贯穿读取路径和关键初始化，不必一次重写所有写调用点。

#### 10.11 动态字符串集中在网络配置路径

网页 JSON、WiFi 扫描、BLE 扫描和 NVS 地址使用 Arduino `String`。它们不在 2 ms 控制热路径中，短期风险低于传感器故障问题；但长期重复扫描和拼接可能造成堆碎片。可以用预留容量、固定上限和运行时最小空闲堆监控控制风险。

#### 10.12 调试发送是死代码

`host_comm::send_status()` 开头直接 `return`，后续完整打包逻辑永远不执行。建议改为编译开关或运行时诊断开关，避免代码表现为“已支持”但实际静默关闭。

#### 10.13 PWM 舵机在全局构造阶段初始化硬件

`pwm_servo` 构造函数直接调用 `ledcSetup()` 和 `ledcAttachPin()`，两个舵机对象又是全局对象。更稳妥的方式是构造函数只保存配置，在明确的 `init()` 阶段操作硬件，便于控制初始化顺序和报告失败。

#### 10.14 测试与文档存在缺口

- `test/` 只有占位 README，没有协议解析、状态机或输入路由测试。
- `reports/LOGIC_CHAIN.md` 仍写 1 ms 控制周期和旧 `set_target/set_command/get_status` API，与当前 2 ms 周期和收紧后的 API 不一致。
- 当前没有一份统一的板级引脚、协议版本和故障码文档。

建议优先给纯逻辑模块补主机测试：UART 帧解析、STS3032 帧解析、输入优先级/超时、动作状态迁移。控制参数本身再通过 HIL/真机验证。

## 11. 推荐推进顺序

### 第一阶段：建立安全底座

1. 修复 STS3032 解析边界并返回读取状态；
2. 贯通 MPU6050 读取错误和传感器新鲜度；
3. 增加统一 fault 状态，故障时零输出、关电机、显式恢复；
4. 将中位校准改成非阻塞状态机；
5. 明确并实现运行中低电降级策略。

### 第二阶段：提高实时可观测性

1. 记录控制周期实际 `dt`、最大执行时间和 deadline miss；
2. 记录各任务 stack high-water mark、最小空闲堆和队列/协议错误计数；
3. 评估 `balance_io_task` 的真实 FOC 频率和 core 1 idle 占用；
4. 用编译开关恢复二进制调试状态输出。

### 第三阶段：收紧外部控制面

1. 给网页/无线控制增加鉴权和物理在场确认；
2. 让 Xbox 任务独占 BLE 对象，网页只投递配置命令；
3. 决定是否需要 OTA；若需要，重新选择分区表；
4. 将内嵌网页资源从近千行服务实现中拆出。

### 第四阶段：补测试和整理文档

1. 给协议解析和状态机补自动化测试；
2. 更新 `LOGIC_CHAIN.md` 中的控制周期和 API；
3. 增加板级硬件映射、协议格式和故障处理文档；
4. 对 LQI 参数、跳跃时序和踢球阈值建立版本化真机测试记录。

## 12. 最终评价

当前项目的架构已经具备继续演进的基础：层次不乱、核心控制边界相对克制、动作扩展点明确、资源占用也健康。下一阶段不建议再优先做大规模目录或抽象重构，而应把现有错误码、时间戳和状态机真正连成安全闭环。

如果按重要性只选三件事，建议依次做：

1. 传感器错误与新鲜度故障保护；
2. STS3032 安全解析和非阻塞校准；
3. 外部控制鉴权与运行中低电策略。

完成这三项后，项目会从“结构清楚、功能完整的机器人固件”进一步接近“异常情况下也可预测、可诊断、可安全恢复的控制系统”。
