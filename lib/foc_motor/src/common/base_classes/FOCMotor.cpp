#include "FOCMotor.h"

/**
 * @brief 关联校准传感器或运行期快照适配器
 *
 * @param linked_sensor 传感器对象
 */
void FOCMotor::linkSensor(Sensor *linked_sensor)
{
    sensor = linked_sensor;
}

/**
 * @brief 获取经过方向修正和滤波的多圈轴角度
 *
 * @return 轴角度，单位 rad
 */
float FOCMotor::shaftAngle()
{
    if(!sensor){return shaft_angle;}
    return sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset;
}

/**
 * @brief 获取经过方向修正和滤波的轴速度
 *
 * @return 轴速度，单位 rad/s
 */
float FOCMotor::shaftVelocity()
{
    if(!sensor){return shaft_velocity;}
    return sensor_direction * LPF_velocity(sensor->getVelocity());
}

/**
 * @brief 根据单圈机械角、极对数和校准零点计算电角度
 *
 * @return 归一化电角度，单位 rad
 */
float FOCMotor::electricalAngle()
{
    if(!sensor){return electrical_angle;}
    return _normalizeAngle((float)(sensor_direction * pole_pairs) *
        sensor->getMechanicalAngle() - zero_electric_angle);
}
