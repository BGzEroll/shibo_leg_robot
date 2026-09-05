#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Sensor.h"
#include "../defaults.h"
#include "../foc_utils.h"
#include "../lowpass_filter.h"

// 保留上游状态值，便于诊断初始化和校准结果。
enum FOCMotorStatus : uint8_t
{
    motor_uninitialized = 0x00,
    motor_initializing = 0x01,
    motor_uncalibrated = 0x02,
    motor_calibrating = 0x03,
    motor_ready = 0x04,
    motor_calib_failed = 0x0E,
    motor_init_failed = 0x0F
};

/**
 * @brief 仅用于电压力矩控制的电机公共状态
 *
 * 保留当前应用使用的 SimpleFOC 名称；不再提供速度、位置和电流环模式。
 */
class FOCMotor
{
    public:
        FOCMotor() = default;

    public:
        void linkSensor(Sensor *sensor);
        float shaftAngle();
        float shaftVelocity();
        float electricalAngle();
        virtual void enable() = 0;
        virtual void disable() = 0;
        virtual void loopFOC() = 0;
        virtual void move(float target = NOT_SET) = 0;
        virtual void setPhaseVoltage(float uq, float angle_el) = 0;
        virtual void init() = 0;
        virtual int initFOC(float zero_electric_offset = NOT_SET,
            Direction sensor_direction = Direction::CW) = 0;

    public:
        float target = 0.0f;
        float shaft_angle = 0.0f;
        float electrical_angle = 0.0f;
        float shaft_velocity = 0.0f;
        DQVoltage_s voltage{};
        float voltage_bemf = 0.0f;
        float voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;
        float phase_resistance = NOT_SET;
        int32_t pole_pairs = 0;
        float KV_rating = NOT_SET;
        float voltage_limit = DEF_POWER_SUPPLY;
        int8_t enabled = 0;
        FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized;
        LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};
        LowPassFilter LPF_angle{0.0f};
        float sensor_offset = 0.0f;
        float zero_electric_angle = NOT_SET;
        int32_t sensor_direction = NOT_SET;
        Sensor *sensor = nullptr;
};

#endif
