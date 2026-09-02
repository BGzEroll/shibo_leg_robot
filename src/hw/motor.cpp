#include "motor.h"

#include "bus/i2c_bus.h"
#include "esp_timer.h"
#include "util/latest.h"

static BLDCDriver3PWM left_driver(32, 33, 25, 22);
static BLDCDriver3PWM right_driver(26, 27, 14, 12);
static MagneticSensorI2C left_encoder(AS5600_I2C);
static MagneticSensorI2C right_encoder(AS5600_I2C);
static i2c_bus left_i2c(0);
static i2c_bus right_i2c(1);
static util::latest<hw::motor::encoder_state> encoder_latest;
static util::latest<control::motor_command> command_latest;

BLDCMotor hw::motor::left(7, 12.27166f, 100.0f);
BLDCMotor hw::motor::right(7, 12.27166f, 100.0f);

/**
 * @brief 读取 FOC task 发布的最新编码器快照
 *
 * @param out 编码器状态输出
 *
 * @return 已有编码器快照时返回 true
 */
bool hw::motor::latest_encoder(hw::motor::encoder_state &out)
{
    return encoder_latest.get(out);
}

/**
 * @brief 读取 control task 发布的最新电机命令
 *
 * @param out 电机命令输出
 *
 * @return 已有电机命令时返回 true
 */
bool hw::motor::latest_command(control::motor_command &out)
{
    return command_latest.get(out);
}

/**
 * @brief 发布最新电机编码器快照
 *
 * @param value 编码器状态
 *
 * @return 发布成功时返回 true
 */
bool hw::motor::publish_encoder(const hw::motor::encoder_state &value)
{
    return encoder_latest.set(value);
}

/**
 * @brief 发布最新电机控制命令
 *
 * @param value 电机命令
 *
 * @return 发布成功时返回 true
 */
bool hw::motor::publish_command(const control::motor_command &value)
{
    return command_latest.set(value);
}

/**
 * @brief 初始化电机、驱动器、编码器和跨任务快照
 */
void hw::motor::init()
{
    left_i2c.init();
    right_i2c.init();
    left_encoder.init(left_i2c.get_TwoWire_handle());
    right_encoder.init(right_i2c.get_TwoWire_handle());

    left.linkSensor(&left_encoder);
    right.linkSensor(&right_encoder);
    left.linkDriver(&left_driver);
    right.linkDriver(&right_driver);

    left.foc_modulation = FOCModulationType::SpaceVectorPWM;
    right.foc_modulation = FOCModulationType::SpaceVectorPWM;
    left_driver.voltage_power_supply = 8.0f;
    right_driver.voltage_power_supply = 8.0f;
    left_driver.init();
    right_driver.init();

    left.voltage_sensor_align = 6.0f;
    right.voltage_sensor_align = 6.0f;
    left.controller = MotionControlType::torque;
    right.controller = MotionControlType::torque;
    left.torque_controller = TorqueControlType::voltage;
    right.torque_controller = TorqueControlType::voltage;

    encoder_latest.init();
    command_latest.init();

    left.init();
    left.initFOC();
    right.init();
    right.initFOC();

    left.disable();
    right.disable();

    control::motor_command initial_command{};
    initial_command.timestamp_us = (uint32_t)esp_timer_get_time();
    initial_command.enabled = false;
    command_latest.set(initial_command);
}
