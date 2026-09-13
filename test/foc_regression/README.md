# 定点 FOC 验证

```sh
python3 test/foc_regression/run.py
/home/bgzerol/.platformio/penv/bin/pio run
git diff --check
```

- 实际 `foc_motor.cpp` 对双精度三相参考扫描全部 65536 相位和七档有符号电压。
- 跨圈、32 位时间戳回绕、双实例隔离、力矩单位、预测上限和重复样本。
- 实际 sensor_task 执行 1000 轮，检查双 AS5600 各 1000 次、IMU 调度 200 次。
- 实际 IDF 总线驱动、固定 AS5600 读取、PWM 写入和 FOC 任务在硬件替身下检查校准、
  通信失败、编码器过期、命令过期和 NaN 命令停机；IMU 驱动错误传播。
- g++ UndefinedBehaviorSanitizer 检查整数等未定义行为。

硬件替身不证明真实 I2C/PWM 波形、RTOS 抢占、1 kHz 达成或带载校准效果。
原 SimpleFOC 等价比较测试随旧接口移除，不再宣称输出与旧模型等价。
