#include "motor.h"

#include "bus/i2c_bus.h"

static BLDCDriver3PWM left_driver = BLDCDriver3PWM(32, 33, 25, 22);
static BLDCDriver3PWM right_driver = BLDCDriver3PWM(26, 27, 14, 12);
static MagneticSensorI2C left_encoder = MagneticSensorI2C(AS5600_I2C);
static MagneticSensorI2C right_encoder = MagneticSensorI2C(AS5600_I2C);
static i2c_bus left_i2c(0);
static i2c_bus right_i2c(1);

static motor::input_ports module_inputs;
static motor::output_ports module_outputs;

BLDCMotor motor::left = BLDCMotor(7, 12.27166f, 100.0f);
BLDCMotor motor::right = BLDCMotor(7, 12.27166f, 100.0f);

/**
 * @brief 读取最新电机力矩目标
 *
 * @param out 电机力矩目标输出
 *
 * @return 存在有效目标时返回 true
 */
bool motor::read_target(motor::target_data &out)
{
    return module_inputs.target.read(out);
}

/**
 * @brief 发布最新编码器状态
 *
 * @param data 编码器状态
 */
void motor::publish_encoder(const motor::encoder_data &data)
{
    module_outputs.encoder.publish(data);
}

/**
 * @brief 初始化电机、驱动器和编码器
 *
 * @param inputs 电机输入端口
 * @param outputs 电机输出端口
 */
void motor::init(const motor::input_ports &inputs, const motor::output_ports &outputs)
{
    module_inputs = inputs;
    module_outputs = outputs;
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

    left.init();
    left.initFOC();
    right.init();
    right.initFOC();
}
