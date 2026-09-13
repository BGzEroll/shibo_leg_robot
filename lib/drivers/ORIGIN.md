# 驱动来源

- `foc_motor.cpp`：移植自 Skythinker616-foc-wheel-legged-robot 的 main `0a867e6`
  (`stm32/user_lib/devices/hw/motor.cpp` 与 `encoder.cpp`)。保留 Q15 查表、
  零序注入 SVPWM、3 ms 速度滤波；改为独立实例和 ESP32 32 位时间戳。
- `pid.*`、`lowpass_filter.*` 及 `motor_pwm.cpp` 的中心对齐定时器配置：
  基于本项目原 SimpleFOC 2.2.3 裁剪版本，许可证见 `SIMPLEFOC_LICENSE`。
  PID/低通运算顺序保留，时基直接使用 IDF，旧类名与电机兼容入口已移除。

## 当前硬件与运行约定

- 保留 PlatformIO espressif32 6.13.0 / Arduino 自带的 IDF 4.4.7。
  I2C 使用 IDF 同步事务，FOC 不访问总线；MCPWM 的 LL 写入绑定此 SDK。
- 单相电阻按星接线间 21.2 Ω / 2 = 10.6 Ω，Kt、Ke 均先按 0.0796，
  固定母线 8 V、7 极对。上层输出原数值视为 N·m，入口换算为 µN·m。
- 校准采用 STM32 的正反扫描和平均零点流程，施加 3 V 对齐电压；
  成功或失败退出时均关闭 EN，运行期等待有效控制命令。
- sensor_task：1 ms 周期，两颗 AS5600 每轮读取，MPU6050 每五轮读取。
  foc_loop：同为 Core 1 / 优先级 5，使用 taskYIELD；每份新样本更新一次
  速度，每轮按最多 1 ms 样本年龄预测角度并更新 PWM。
- 任一 AS5600 读取失败或超过 5 ms、控制命令超过 20 ms，关闭双电机 EN。
  仍使用单槽最新快照；故障后的有效新样本和有效命令允许恢复输出。
- 验证命令及硬件验证边界见 `../../test/foc_regression/README.md`。
