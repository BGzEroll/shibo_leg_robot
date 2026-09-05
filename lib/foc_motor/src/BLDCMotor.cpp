#include "BLDCMotor.h"

#include "common/time_utils.h"

/* ---- 电机参数与驱动 ---- */

/**
 * @brief 初始化电机物理参数
 *
 * @param pairs 极对数
 * @param resistance 相电阻，单位欧姆；NOT_SET 表示直接使用电压目标
 * @param kv 转速常数，单位 rpm/V；NOT_SET 表示不补偿反电动势
 */
BLDCMotor::BLDCMotor(int pairs, float resistance, float kv)
{
    pole_pairs = pairs;
    phase_resistance = resistance;
    KV_rating = kv;
}

/**
 * @brief 关联三相 PWM 驱动
 *
 * @param linked_driver 已配置的驱动对象
 */
void BLDCMotor::linkDriver(BLDCDriver *linked_driver)
{
    driver = linked_driver;
}

/** @brief 使能驱动并清零三相输出 */
void BLDCMotor::enable()
{
    driver->enable();
    driver->setPwm(0, 0, 0);
    enabled = 1;
}

/** @brief 清零三相输出并关闭驱动 */
void BLDCMotor::disable()
{
    driver->setPwm(0, 0, 0);
    driver->disable();
    enabled = 0;
}

/* ---- 电压力矩与 SVPWM ---- */

/**
 * @brief 用已有电压命令和最新电角度更新 PWM
 *
 * 应用保持先 loopFOC() 再 move() 的顺序，新电压命令下一采样周期生效。
 */
void BLDCMotor::loopFOC()
{
    if(!enabled){return;}
    electrical_angle = electricalAngle();
    setPhaseVoltage(voltage.q, electrical_angle);
}

/**
 * @brief 更新轴反馈并计算下一周期的 q 轴电压
 *
 * @param new_target 力矩控制目标；NOT_SET 表示沿用上次目标
 */
void BLDCMotor::move(float new_target)
{
    // 停机期间仍更新反馈，供上层平衡控制读取。
    shaft_angle = shaftAngle();
    shaft_velocity = shaftVelocity();
    if(!enabled){return;}
    if(_isset(new_target)){target = new_target;}

    if(_isset(KV_rating))
    {
        voltage_bemf = shaft_velocity / KV_rating / _RPM_TO_RADS;
    }
    if(!_isset(phase_resistance))
    {
        voltage.q = target;
    }
    else
    {
        voltage.q = target * phase_resistance + voltage_bemf;
    }
    voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
    voltage.d = 0;
}

/**
 * @brief 按原六扇区公式生成居中 SVPWM 三相电压
 *
 * @param uq q 轴电压，单位 V，允许正负值
 * @param angle_el 转子电角度，单位 rad
 */
void BLDCMotor::setPhaseVoltage(float uq, float angle_el)
{
    float u_out = uq / driver->voltage_limit;
    angle_el = _normalizeAngle(angle_el + _PI_2);
    int32_t sector = floor(angle_el / _PI_3) + 1;
    float t1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * u_out;
    float t2 = _SQRT3 * _sin(angle_el - (sector - 1.0f) * _PI_3) * u_out;
    float t0 = 1 - t1 - t2;
    float ta;
    float tb;
    float tc;
    switch(sector)
    {
        case 1:
            ta = t1 + t2 + t0 / 2;
            tb = t2 + t0 / 2;
            tc = t0 / 2;
            break;
        case 2:
            ta = t1 + t0 / 2;
            tb = t1 + t2 + t0 / 2;
            tc = t0 / 2;
            break;
        case 3:
            ta = t0 / 2;
            tb = t1 + t2 + t0 / 2;
            tc = t2 + t0 / 2;
            break;
        case 4:
            ta = t0 / 2;
            tb = t1 + t0 / 2;
            tc = t1 + t2 + t0 / 2;
            break;
        case 5:
            ta = t2 + t0 / 2;
            tb = t0 / 2;
            tc = t1 + t2 + t0 / 2;
            break;
        case 6:
            ta = t1 + t2 + t0 / 2;
            tb = t0 / 2;
            tc = t1 + t0 / 2;
            break;
        default:
            ta = 0;
            tb = 0;
            tc = 0;
            break;
    }
    Ua = ta * driver->voltage_limit;
    Ub = tb * driver->voltage_limit;
    Uc = tc * driver->voltage_limit;
    driver->setPwm(Ua, Ub, Uc);
}

/* ---- 启动与传感器校准 ---- */

/** @brief 检查驱动、约束电压并按原等待时间使能电机 */
void BLDCMotor::init()
{
    if(!driver || !driver->initialized)
    {
        motor_status = FOCMotorStatus::motor_init_failed;
        return;
    }
    motor_status = FOCMotorStatus::motor_initializing;
    if(voltage_limit > driver->voltage_limit){voltage_limit = driver->voltage_limit;}
    if(voltage_sensor_align > voltage_limit){voltage_sensor_align = voltage_limit;}
    _delay(500);
    enable();
    _delay(500);
    motor_status = FOCMotorStatus::motor_uncalibrated;
}

/**
 * @brief 校准传感器方向和电角度零点
 *
 * @param zero_electric_offset 预置电角度零点，NOT_SET 表示自动校准
 * @param direction 预置零点时使用的传感器方向
 *
 * @return 校准成功返回 1，失败返回 0
 */
int BLDCMotor::initFOC(float zero_electric_offset, Direction direction)
{
    int32_t success = 1;
    motor_status = FOCMotorStatus::motor_calibrating;
    if(_isset(zero_electric_offset))
    {
        zero_electric_angle = zero_electric_offset;
        sensor_direction = direction;
    }

    _delay(500);
    if(sensor)
    {
        success = align_sensor();
        sensor->update();
        shaft_angle = shaftAngle();
    }
    // 保留原无电流传感器路径的等待时间。
    _delay(500);
    if(success)
    {
        motor_status = FOCMotorStatus::motor_ready;
    }
    else
    {
        motor_status = FOCMotorStatus::motor_calib_failed;
        disable();
    }
    return success;
}

/**
 * @brief 扫描一个电周期识别方向，并在固定电角度下记录零点
 *
 * @return 检测到运动或已预置方向时返回 1，未检测到运动返回 0
 */
int32_t BLDCMotor::align_sensor()
{
    // AS5600 为绝对角度传感器，无需编码器索引搜索。
    if(!_isset(sensor_direction))
    {
        for(int32_t i = 0; i <= 500; i++)
        {
            float angle = _3PI_2 + _2PI * i / 500.0f;
            setPhaseVoltage(voltage_sensor_align, angle);
            sensor->update();
            _delay(2);
        }
        sensor->update();
        float mid_angle = sensor->getAngle();
        for(int32_t i = 500; i >= 0; i--)
        {
            float angle = _3PI_2 + _2PI * i / 500.0f;
            setPhaseVoltage(voltage_sensor_align, angle);
            sensor->update();
            _delay(2);
        }
        sensor->update();
        float end_angle = sensor->getAngle();
        setPhaseVoltage(0, 0);
        _delay(200);
        if(mid_angle == end_angle){return 0;}
        sensor_direction = mid_angle < end_angle ? Direction::CCW : Direction::CW;
    }

    if(!_isset(zero_electric_angle))
    {
        setPhaseVoltage(voltage_sensor_align, _3PI_2);
        _delay(700);
        sensor->update();
        zero_electric_angle = 0;
        zero_electric_angle = electricalAngle();
        _delay(20);
        setPhaseVoltage(0, 0);
        _delay(200);
    }
    return 1;
}
