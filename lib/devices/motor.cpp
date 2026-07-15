#include "motor.h"

#include "bus/i2c_bus.h"

static BLDCDriver3PWM left_driver = BLDCDriver3PWM(32, 33, 25, 22);
static BLDCDriver3PWM right_driver = BLDCDriver3PWM(26, 27, 14, 12);
static MagneticSensorI2C left_encoder = MagneticSensorI2C(AS5600_I2C);
static MagneticSensorI2C right_encoder = MagneticSensorI2C(AS5600_I2C);
static i2c_bus left_i2c(0);
static i2c_bus right_i2c(1);

static QueueHandle_t encoder_data_queue = nullptr;
static QueueHandle_t motor_target_data_queue = nullptr;

BLDCMotor motor::left = BLDCMotor(7, 12.27166f, 100.0f);
BLDCMotor motor::right = BLDCMotor(7, 12.27166f, 100.0f);

/**
 * @brief 发布最新电机编码器数据
 *
 * @param value 电机编码器数据
 *
 * @return 队列存在且发布成功时返回 true
 */
bool motor::publish_encoder(const motor::encoder_data &value)
{
    return encoder_data_queue &&
           xQueueOverwrite(encoder_data_queue, &value) == pdTRUE;
}

/**
 * @brief 读取最新电机编码器数据
 *
 * @param out 电机编码器数据输出
 *
 * @return 队列存在且已有数据时返回 true
 */
bool motor::peek_encoder(motor::encoder_data &out)
{
    return encoder_data_queue &&
           xQueuePeek(encoder_data_queue, &out, 0) == pdTRUE;
}

/**
 * @brief 发布最新电机目标输出
 *
 * @param value 电机目标输出
 *
 * @return 队列存在且发布成功时返回 true
 */
bool motor::publish_target(const motor::target_data &value)
{
    return motor_target_data_queue &&
           xQueueOverwrite(motor_target_data_queue, &value) == pdTRUE;
}

/**
 * @brief 读取最新电机目标输出
 *
 * @param out 电机目标输出
 *
 * @return 队列存在且已有数据时返回 true
 */
bool motor::peek_target(motor::target_data &out)
{
    return motor_target_data_queue &&
           xQueuePeek(motor_target_data_queue, &out, 0) == pdTRUE;
}

/**
 * @brief 初始化电机、驱动器和编码器
 */
void motor::init()
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

    encoder_data_queue = xQueueCreate(1, sizeof(encoder_data));
    motor_target_data_queue = xQueueCreate(1, sizeof(target_data));

    left.init();
    left.initFOC();
    right.init();
    right.initFOC();
}
