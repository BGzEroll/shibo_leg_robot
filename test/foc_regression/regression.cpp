#include "BLDCMotor.h"

#include "drivers/BLDCDriver3PWM.h"
#include "common/pid.h"
#include "sensors/MagneticSensorI2C.h"
#include <cassert>
#include <cstdio>

uint32_t test_time_us = 1000;
uint32_t test_pwm_writes = 0;
bool test_enabled = false;

// 硬件写入替身；真实限压和占空比转换仍由 BLDCDriver3PWM 执行。
/** @brief 为电机数值测试提供占位的硬件资源句柄 */
void *_configure3PWM(long, int, int, int)
{
    static int params;
    return &params;
}

/** @brief 统计 PWM 写入次数以验证输出顺序 */
void _writeDutyCycle3PWM(float, float, float, void *)
{
    test_pwm_writes++;
}

class test_sensor : public Sensor
{
    public:
        float mechanical = 0.0f;
        float position = 0.0f;
        float speed = 0.0f;
        bool stationary = false;
        uint32_t reads = 0;

        void update() override
        {
            reads++;
            if(!stationary){mechanical += 0.001f;}
        }
        float getMechanicalAngle() override { return mechanical; }
        float getAngle() override { return mechanical + position; }
        float getVelocity() override { return speed; }
        float getSensorAngle() override { return mechanical; }
};

/** @brief 配置与机器人相同的力矩、电压和调制路径 */
static void configure(BLDCMotor &motor, BLDCDriver3PWM &driver, Sensor &sensor)
{
    driver.voltage_power_supply = 8.0f;
    driver.init();
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);
#ifndef FOC_TRIMMED
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
#endif
    motor.voltage_sensor_align = 6.0f;
    motor.init();
}

/** @brief 记录实际浮点状态、PWM 次数和使能状态 */
static void emit(const BLDCMotor &motor, const BLDCDriver3PWM &driver)
{
    std::printf("%a %a %a %a %a %a %a %a %a %u %d %u\n",
        motor.voltage.q, motor.voltage_bemf, motor.shaft_angle,
        motor.shaft_velocity, motor.electrical_angle,
        driver.dc_a, driver.dc_b, driver.dc_c, motor.zero_electric_angle,
        test_pwm_writes, test_enabled, test_time_us);
}

/** @brief 覆盖正负电压、六扇区边界、限幅、使能切换和一个周期命令延迟 */
static void test_runtime()
{
    // 静态存储期与固件中的电机对象一致。
    static BLDCMotor motor(7, 12.27166f, 100.0f);
    static BLDCDriver3PWM driver(32, 33, 25, 22);
    static test_sensor sensor;
    configure(motor, driver, sensor);
    motor.initFOC(0.3f, Direction::CCW);

    for(int32_t step = -720; step <= 720; step++)
    {
        float angle = step * _PI_3 / 60.0f;
        for(float uq : {-12.0f, -8.0f, -0.001f, 0.0f, 0.001f, 8.0f, 12.0f})
        {
#ifdef FOC_TRIMMED
            motor.setPhaseVoltage(uq, angle);
#else
            motor.setPhaseVoltage(uq, 0.0f, angle);
#endif
            emit(motor, driver);
        }
    }
    for(uint32_t step = 0; step < 3000; step++)
    {
        test_time_us += 1000;
        sensor.mechanical = _normalizeAngle(step * 0.03f);
        sensor.position = step * 0.02f;
        sensor.speed = (int32_t(step % 121) - 60) * 0.75f;
        if(step % 200 == 0){motor.disable();}
        if(step % 200 == 30){motor.enable();}
        uint32_t reads = sensor.reads;
        motor.loopFOC();
        assert(sensor.reads == reads); // 热路径不得触发传感器读取。
        emit(motor, driver);
        if(motor.enabled){motor.move((int32_t(step % 41) - 20) * 0.1f);}
        else{motor.move();}
        emit(motor, driver);
    }
}

/** @brief 覆盖自动校准、静止失败及方向和零点预置 */
static void test_calibration()
{
    for(uint32_t scenario = 0; scenario < 4; scenario++)
    {
        static BLDCMotor motor(7, 12.27166f, 100.0f);
        static BLDCDriver3PWM driver(32, 33, 25, 22);
        static test_sensor sensor;
        motor.sensor_direction = NOT_SET;
        motor.zero_electric_angle = NOT_SET;
        sensor.stationary = scenario == 1;
        configure(motor, driver, sensor);
        int result;
        if(scenario == 2){result = motor.initFOC(1.2f, Direction::CW);}
        else
        {
            if(scenario == 3){motor.sensor_direction = Direction::CCW;}
            result = motor.initFOC();
        }
        std::printf("calibration %u %d %d %u\n", scenario, result,
            motor.sensor_direction, sensor.reads);
        emit(motor, driver);
        assert((result != 0) == (scenario != 1));
    }
}

/** @brief 覆盖共用 PID 的积分限幅、斜率约束及滤波长间隔 */
static void test_filters()
{
    PIDController pid(8.0f, 30.0f, 0.1f, 100000.0f, 450.0f);
    LowPassFilter filter(0.005f);
    for(uint32_t step = 0; step < 1000; step++)
    {
        test_time_us += step % 100 == 0 ? 600000 : 2000;
        float input = (int32_t(step % 31) - 15) * 6.0f;
        std::printf("filter %a %a\n", pid(input), filter(input));
    }
}

/** @brief 覆盖 AS5600 全量计数、双向跨圈和速度最小更新间隔 */
static void test_encoder()
{
    MagneticSensorI2C sensor(AS5600_I2C);
    sensor.init(&Wire);
    for(int32_t direction : {1, -1})
    {
        for(int32_t step = 0; step < 8192; step++)
        {
            Wire.raw = direction > 0 ? step % 4096 : 4095 - step % 4096;
            test_time_us += step % 3 == 0 ? 50 : 1000;
            sensor.update();
            float velocity = sensor.getVelocity();
            std::printf("encoder %a %a %a %u\n", sensor.getMechanicalAngle(),
                sensor.getAngle(), velocity, Wire.requests);
        }
    }
}

/** @brief 运行裁剪前后的同输入回归序列 */
int main()
{
    test_runtime();
    test_calibration();
    test_filters();
    test_encoder();
}
