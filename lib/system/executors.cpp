#include "executors.h"

#include "balance_core.h"
#include "actuator_adapter.h"
#include "controller.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "motor.h"
#include "mpu6050_dev.h"

/**
 * @brief 高频硬件 IO 执行器任务入口
 *
 * @param arg RTOS 任务参数
 */
void system_executor::fast_io_task_entry(void *arg)
{
    uint32_t last_encoder_us = (uint32_t)esp_timer_get_time();
    uint32_t last_imu_us = last_encoder_us;
    bool motor_enabled = true;

    while(true)
    {
        motor::left.loopFOC();
        motor::right.loopFOC();

        motor::target_data target;
        if(motor::read_target(target))
        {
            if(target.enabled && !motor_enabled)
            {
                motor::left.enable();
                motor::right.enable();
                motor_enabled = true;
            }
            if(!target.enabled && motor_enabled)
            {
                motor::left.move(0.0f);
                motor::right.move(0.0f);
                motor::left.disable();
                motor::right.disable();
                motor_enabled = false;
            }
            if(target.enabled)
            {
                motor::left.move(target.left_torque);
                motor::right.move(target.right_torque);
            }
        }

        uint32_t now_us = (uint32_t)esp_timer_get_time();
        if((uint32_t)(now_us - last_encoder_us) >= 1000)
        {
            last_encoder_us = now_us;
            motor::encoder_data encoder;
            encoder.timestamp_us = now_us;
            encoder.left_shaft_angle = motor::left.shaft_angle;
            encoder.left_shaft_velocity = motor::left.shaft_velocity;
            encoder.right_shaft_angle = motor::right.shaft_angle;
            encoder.right_shaft_velocity = motor::right.shaft_velocity;
            motor::publish_encoder(encoder);
        }

        if((uint32_t)(now_us - last_imu_us) >= 5000)
        {
            last_imu_us = now_us;
            mpu6050_dev::imu.update();

            mpu6050_dev::data imu;
            imu.timestamp_us = now_us;
            imu.temperature = mpu6050_dev::imu.temperature;
            for(uint8_t i = 0; i < 3; i++)
            {
                imu.acc[i] = mpu6050_dev::imu.acc[i];
                imu.gyro[i] = mpu6050_dev::imu.gyro[i];
                imu.angle[i] = mpu6050_dev::imu.angle[i];
            }
            mpu6050_dev::publish(imu);
        }

        taskYIELD();
    }
}

/**
 * @brief 确定顺序执行动作管理和平衡控制的任务入口
 *
 * @param arg RTOS 任务参数
 */
void system_executor::control_task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    static constexpr uint32_t CONTROL_PERIOD_MS = 2;

    while(true)
    {
        actuator_adapter::sample_leg_status((uint32_t)esp_timer_get_time());
        controller::update(CONTROL_PERIOD_MS);
        balance_core::step(CONTROL_PERIOD_MS);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
