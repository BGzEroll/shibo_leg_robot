# foc_motor 来源与维护范围

本目录由当前机器人使用的 SimpleFOC 路径裁剪而来。

- 上游项目：Arduino-FOC/simplefoc。
- 上游包：askuric/Simple FOC，导入版本 2.2.3。
- 许可证：MIT，完整声明保留在 `LICENSE`。
- 本轮裁剪基线：本仓库提交 `9a3b8757acea816755bfd852b24da0cf9257b620`。

## 当前支持路径

AS5600 I2C 采样 → 应用单槽快照 → `sampled_sensor` → 电压力矩控制 → 居中 SVPWM → ESP32 MCPWM 3PWM，驱动使用一路公共使能。

- 运行期由应用提交传感器样本，`loopFOC()` 不读取 I2C；`initFOC()` 的显式读取保持原样。
- 保留电机方向和电角度零点校准，包括原采样顺序、角度扫描和等待时间。
- 提供相电阻时，`move()` 按 `target * phase_resistance + voltage_bemf` 计算 q 轴电压并限幅；保留 KV 反电动势补偿和轴速度滤波。
- 保留 `loopFOC()` 后调用 `move()` 的应用顺序：本次 PWM 使用上次计算的电压，新命令在下一采样周期输出。
- 停机时 `move()` 仍更新轴角度和速度反馈。
- 保留原 SVPWM 扇区公式、正负 Uq 处理、查表采样值、舍入规则和三相限压。
- 保留 MCPWM 原资源分配顺序：前两台电机共用 MCPWM0 的计时器，分别使用 A/B 输出；计时器频率、中心对齐和同步调用顺序保持不变。

## 已移除的能力

- 电流采样基类、dc_current/foc_current 电流环及其 PID、滤波和电流监视状态。
- 速度环、位置环、运行期开环、编码器索引搜索及运动降采样计数。
- SinePWM、梯形调制、非零 Ud 和非居中调制。
- 2PWM、4PWM、6PWM、死区配置分支及不同驱动类型间的槽位互斥逻辑。
- 未使用的串口监视和 SimpleFOCDebug；初始化结果仍可通过 `motor_status` 和 `initFOC()` 返回值检查。
- AS5048 配置、未使用的 I2C 构造重载、工厂声明和总线恢复接口。
- AVR/RP2040 兼容分支、未引用的数学工具和默认参数。

## 接口与代码风格

当前应用使用的 SimpleFOC 类名、反馈字段及主要 API 保留原名。保留的上游接口使用原参数类型，内部整数状态优先使用显式定宽类型；内部变量、辅助函数和 MCPWM 私有类型使用 snake_case。代码按 `.agents/skills/embedded-cpp-workflow` 整理为 4 空格缩进、换行括号和中文 Doxygen 注释。

该目录已是项目专用子集，不再声明完整的上游 API 兼容性：

- 删除 `controller`、`torque_controller`、`foc_modulation` 等模式选择字段和枚举，能力固定为上述路径；应用中的对应六行赋值已移除。
- `setPhaseVoltage(uq, angle_el)` 省去恒为零的 Ud 参数。
- `BLDCDriver3PWM(phase_a, phase_b, phase_c, enable_pin)` 仅保留公共使能引脚，不再提供逐相使能和 `setPhaseState()`。
- `Sensor` 保留实际使用的采样、单圈角、多圈角和速度接口。
- `PIDController`、`LowPassFilter` 仍供腿部动作和平衡模块使用，公共实现没有删除。
- `esp32_driver_mcpwm.h` 的内部结构收回 `.cpp`，后端头文件改为与源文件同名的 `esp32_mcu.h`。

## 独立的初始化修复

- `mcpwm_config_t` 使用零初始化，避免原来未初始化的 `cmpr_a/cmpr_b` 被传入 MCPWM 初始化接口。频率、极性、计时器和同步参数不变。
- 3PWM 槽位耗尽时返回 `SIMPLEFOC_DRIVER_INIT_FAILED`，避免原实现越界访问槽位。当前两台电机的分配顺序不变。

本轮未改任务、队列、超时、硬件引脚或上层控制参数；未重构快照适配器和电机继承关系。I2C 传输失败检测仍是原实现，后续应作为独立行为修改处理。

## 验证

可复现步骤和覆盖边界见 `../../test/foc_regression/README.md`。

2026-09-05 本地验证结果：

- 裁剪前后 33,479 行十六进制浮点及状态记录逐字节一致。
- 92 行 MCPWM 配置和写入记录一致；零占空比初始化和槽位耗尽断言通过。
- PlatformIO `esp32dev` 构建通过。
- 构建报告静态 RAM：58,668 → 57,908 字节；Flash：1,200,433 → 1,193,009 字节。
- 尚未烧录，未验证实际 PWM 波形、电机运行效果或周期耗时。
