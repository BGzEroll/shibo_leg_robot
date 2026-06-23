#include "motor.h"

static BLDCDriver3PWM left_driver = BLDCDriver3PWM(32, 33, 25, 22);
static BLDCDriver3PWM right_driver = BLDCDriver3PWM(26, 27, 14, 12);
static MagneticSensorI2C left_encoder = MagneticSensorI2C(AS5600_I2C);
static MagneticSensorI2C right_encoder = MagneticSensorI2C(AS5600_I2C);

static QueueHandle_t encoder_data_queue = nullptr;
static QueueHandle_t motor_target_data_queue = nullptr;

BLDCMotor motor::left = BLDCMotor(7, 12.27166f, 100.0f);
BLDCMotor motor::right = BLDCMotor(7, 12.27166f, 100.0f);

/**
 * @brief 获取电机编码器数据队列
 *
 * @return 队列句柄
 */
QueueHandle_t motor::encoder_queue()
{
    return encoder_data_queue;
}

/**
 * @brief 获取电机目标输出队列
 *
 * @return 队列句柄
 */
QueueHandle_t motor::target_queue()
{
    return motor_target_data_queue;
}

/**
 * @brief 初始化电机、驱动器和编码器
 */
void motor::init()
{
    i2c_bus_init(&i2c1);
    i2c_bus_init(&i2c2);

    left_encoder.init(i2c1.get_TwoWire_handle(&i2c1));
    right_encoder.init(i2c2.get_TwoWire_handle(&i2c2));

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
