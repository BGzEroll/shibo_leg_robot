#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/BLDCDriver.h"

/** @brief 使用居中 SVPWM 的电压力矩电机 */
class BLDCMotor : public FOCMotor
{
    public:
        BLDCMotor(int pole_pairs, float resistance = NOT_SET, float kv = NOT_SET);

    public:
        void linkDriver(BLDCDriver *driver);
        void enable() override;
        void disable() override;
        // 运行期由应用先提交传感器样本，本方法不访问 I2C。
        void loopFOC() override;
        // 提供相电阻时，目标按电流乘电阻并叠加反电动势转换为电压。
        void move(float target = NOT_SET) override;
        // 仅支持 Ud = 0；省去无效的 d 轴参数。
        void setPhaseVoltage(float uq, float angle_el) override;
        void init() override;
        int initFOC(float zero_electric_offset = NOT_SET,
            Direction sensor_direction = Direction::CW) override;

    public:
        BLDCDriver *driver = nullptr;
        float Ua = 0.0f;
        float Ub = 0.0f;
        float Uc = 0.0f;

    private:
        int32_t align_sensor();
};

#endif
