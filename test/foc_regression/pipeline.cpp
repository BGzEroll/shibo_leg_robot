#include "../../../src/devices/hw/motor.cpp"
#include "hal/mcpwm_ll.h"
#include "mpu6050.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <algorithm>

uint64_t fake_time_us = 1000;
int fake_gpio[40]{};
fake_mcpwm MCPWM0{};
static bool fail_bus[2]{};
static uint32_t read_count[2]{};
static uint32_t fail_after_read = UINT32_MAX;
static uint32_t imu_count = 0;
static bool simulate_motor = true;
static double last_angle[2]{}, mechanical[2]{};
static uint16_t raw_angle[2]{};
struct stop_task {};
void taskYIELD(){throw stop_task{};}
static uint32_t sensor_ticks = 0;
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    *last += period;
    fake_time_us = (uint64_t)*last * 1000;
    if(++sensor_ticks == 1000){throw stop_task{};}
}

namespace hw {namespace imu {void sample(){imu_count++;}}}

/** @brief 模拟独立两台理想转子，按输出电压矢量跟随，第二台编码器反向 */
int i2c_master_write_read_device(i2c_port_t port, uint8_t addr,
    const uint8_t *reg, size_t, uint8_t *data, size_t size, unsigned)
{
    read_count[port]++;
    fake_time_us += 100;
    if(fail_bus[port] || (port == 0 && read_count[0] >= fail_after_read)){return -1;}
    if(addr == 0x68)
    {
        std::fill(data, data + size, 0);
        return 0;
    }
    assert(addr == 0x36 && *reg == 0x0C && size == 2);
    if(simulate_motor && fake_gpio[port == 0 ? 22 : 12] &&
       fake_gpio_mux[port == 0 ? 22 : 12])
    {
        const double a = MCPWM0.compare[0][port];
        const double b = MCPWM0.compare[1][port];
        const double c = MCPWM0.compare[2][port];
        const double angle = std::atan2((b - c) / std::sqrt(3.0), (2*a - b - c) / 3);
        const double delta = std::remainder(angle - last_angle[port], 2 * std::acos(-1.0));
        mechanical[port] += delta / 7 * (port == 0 ? 1 : -1);
        last_angle[port] = angle;
        raw_angle[port] = (uint16_t)(int32_t)std::lround(mechanical[port] * 4096 / (2 * std::acos(-1.0))) & 4095;
    }
    data[0] = raw_angle[port] >> 8;
    data[1] = raw_angle[port] & 255;
    return 0;
}

/** @brief 执行实际 FOC 循环的一轮 */
static void step_foc()
{
    try {hw::motor::foc_loop(nullptr);} catch(const stop_task &) {}
}

/** @brief 覆盖校准失败、双实例校准、采样比例、失效停机和上层单位 */
int main()
{
    fake_gpio_mux[22] = true; // GPIO12 模拟上电后尚未切到 GPIO 功能。
    fail_bus[0] = true;
    assert(!hw::motor::init());
    assert(!fake_gpio[22] && !fake_gpio[12]);
    fail_bus[0] = false;
    assert(hw::motor::init());
    assert(motors[0].direction == 1 && motors[1].direction == -1);
    assert(!fake_gpio[22] && !fake_gpio[12]);
    // 已使能的校准过程中断线，init 的调用方路径必须关闭 EN。
    fail_after_read = read_count[0] + 20;
    assert(!hw::motor::init());
    assert(!ready && !fake_gpio[22] && !fake_gpio[12]);
    fail_after_read = UINT32_MAX;
    assert(hw::motor::init());
    simulate_motor = false;
    read_count[0] = read_count[1] = 0;
    try {hw::sensor::task_entry(nullptr);} catch(const stop_task &) {}
    assert(read_count[0] == 1000 && read_count[1] == 1000 && imu_count == 200);
    hw::motor::command input;
    input.enabled = true;
    input.left = 0.012345f;
    input.right = -0.023456f;
    input.timestamp_us = fake_time_us;
    assert(hw::motor::publish_command(input));
    torque_command command;
    assert(command_latest.get(command));
    assert(std::abs(command.torque_uNm[0] - 12345) <= 1);
    assert(std::abs(command.torque_uNm[1] + 23456) <= 1);
    step_foc();
    assert(fake_gpio[22] && fake_gpio[12]);
    fail_bus[0] = true;
    hw::motor::sample_encoders();
    step_foc();
    assert(!fake_gpio[22] && !fake_gpio[12]);
    hw::motor::encoder_state feedback;
    assert(hw::motor::latest_encoder(feedback) && !feedback.valid);
    fail_bus[0] = false;
    hw::motor::sample_encoders();
    step_foc();
    assert(fake_gpio[22] && fake_gpio[12]);
    rotor_sample aged;
    assert(rotor_latest.get(aged));
    aged.axes[0].timestamp_us = (uint32_t)fake_time_us - 5001;
    aged.axes[1].timestamp_us = (uint32_t)fake_time_us;
    aged.sequence++;
    rotor_latest.set(aged);
    step_foc();
    assert(!fake_gpio[22] && !fake_gpio[12]);
    fake_time_us += 5001;
    step_foc();
    assert(!fake_gpio[22] && !fake_gpio[12]);
    fake_time_us += 20000;
    hw::motor::sample_encoders();
    step_foc();
    assert(!fake_gpio[22] && !fake_gpio[12]);
    input.timestamp_us = fake_time_us;
    input.left = std::numeric_limits<float>::quiet_NaN();
    hw::motor::publish_command(input);
    step_foc();
    assert(!fake_gpio[22] && !fake_gpio[12]);
    // 同一 IDF 总线上的 IMU 读取失败必须向上传播。
    mpu6050 imu(hw::sensor::right_bus, 0x68, 0.02f);
    assert(imu.init(false) && imu.update());
    fail_bus[1] = true;
    assert(!imu.update());
    fail_bus[1] = false;
    assert(imu.update());
    std::puts("Actual sensor task: AS5600 1000/1000, MPU6050 200; calibration, IDF I2C failure, FOC stop: PASS");
}
