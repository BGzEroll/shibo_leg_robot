#include "motor.h"

#include "bus/i2c_bus.h"
#include "util/latest.h"
#include "esp_timer.h"
#include <type_traits>

/* ---- 编码器采样与 FOC 缓存 ---- */

struct rotor_axis_sample
{
    uint32_t timestamp_us = 0;
    float mechanical_angle = 0.0f;
    float multi_turn_angle = 0.0f;
    float velocity = 0.0f;
    bool valid = false;
};

struct rotor_sample
{
    uint32_t sequence = 0;
    rotor_axis_sample left;
    rotor_axis_sample right;
};

static_assert(std::is_trivially_copyable<rotor_sample>::value,
    "rotor_sample must be trivially copyable");

// 向 SimpleFOC 提供由 FOC task 应用的最近编码器样本，不访问 I2C。
class sampled_sensor : public Sensor
{
    public:
        void set_sample(const rotor_axis_sample &sample)
        {
            current = sample;
        }

        void update() override
        {
        }

        float getMechanicalAngle() override
        {
            return current.mechanical_angle;
        }

        float getAngle() override
        {
            return current.multi_turn_angle;
        }

        float getVelocity() override
        {
            return current.velocity;
        }

    protected:
        float getSensorAngle() override
        {
            return current.mechanical_angle;
        }

    private:
        rotor_axis_sample current;
};

static BLDCDriver3PWM left_driver(32, 33, 25, 22);
static BLDCDriver3PWM right_driver(26, 27, 14, 12);
static MagneticSensorI2C left_encoder(AS5600_I2C);
static MagneticSensorI2C right_encoder(AS5600_I2C);
static sampled_sensor left_sampled_sensor;
static sampled_sensor right_sampled_sensor;
static i2c_bus left_i2c(0);
static i2c_bus right_i2c(1);
static util::latest<rotor_sample> rotor_latest;
static util::latest<hw::motor::encoder_state> encoder_latest;
static util::latest<control::motor_command> command_latest;
static uint32_t rotor_sequence = 0;
static uint32_t applied_rotor_sequence = 0;

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
 * @brief 从两个真实 AS5600 读取一组样本并覆盖编码器队列
 *
 * 左右编码器顺序采样并分别记录时间戳。运行期只允许 sensor task 调用。
 *
 * @return 样本成功写入单槽队列时返回 true
 */
bool hw::motor::sample_encoders()
{
    rotor_sample sample;
    sample.sequence = ++rotor_sequence;

    left_encoder.update();
    sample.left.timestamp_us = (uint32_t)esp_timer_get_time();
    sample.left.mechanical_angle = left_encoder.getMechanicalAngle();
    sample.left.multi_turn_angle = left_encoder.getAngle();
    sample.left.velocity = left_encoder.getVelocity();
    sample.left.valid = true;

    right_encoder.update();
    sample.right.timestamp_us = (uint32_t)esp_timer_get_time();
    sample.right.mechanical_angle = right_encoder.getMechanicalAngle();
    sample.right.multi_turn_angle = right_encoder.getAngle();
    sample.right.velocity = right_encoder.getVelocity();
    sample.right.valid = true;

    return rotor_latest.set(sample);
}

/**
 * @brief 将编码器队列中的新样本应用到两个 FOC 缓存传感器
 *
 * @param timestamp_us 本次样本中较晚的右编码器采样时间
 *
 * @return 存在尚未应用的新样本时返回 true
 */
bool hw::motor::apply_latest_encoder_sample(uint32_t &timestamp_us)
{
    rotor_sample sample;
    if(!rotor_latest.get(sample)){return false;}
    if(sample.sequence == applied_rotor_sequence){return false;}

    applied_rotor_sequence = sample.sequence;
    if(!sample.left.valid || !sample.right.valid){return false;}

    left_sampled_sensor.set_sample(sample.left);
    right_sampled_sensor.set_sample(sample.right);
    timestamp_us = sample.right.timestamp_us;
    return true;
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

    rotor_latest.init();
    encoder_latest.init();
    command_latest.init();

    left.init();
    left.initFOC();
    right.init();
    right.initFOC();

    sample_encoders();
    uint32_t initial_sample_us = 0;
    apply_latest_encoder_sample(initial_sample_us);
    left.linkSensor(&left_sampled_sensor);
    right.linkSensor(&right_sampled_sensor);

    left.disable();
    right.disable();

    control::motor_command initial_command{};
    initial_command.timestamp_us = (uint32_t)esp_timer_get_time();
    initial_command.enabled = false;
    command_latest.set(initial_command);
}
