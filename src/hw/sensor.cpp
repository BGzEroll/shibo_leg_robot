#include "sensor.h"

#include "imu.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr uint32_t SENSOR_PERIOD_MS = 1;
static constexpr uint32_t IMU_SAMPLE_DIVIDER = 5;

/**
 * @brief 按固定周期串行采样两个 AS5600 和 MPU6050
 *
 * 右侧 AS5600 与 MPU6050 共用 I2C1，因此所有运行期 I2C 读取均在本任务
 * 中完成。编码器每周期采样，IMU 每五个周期采样一次。
 *
 * @param arg RTOS 任务参数
 */
void hw::sensor::task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t imu_sample_divider = 0;

    while(true)
    {
        hw::motor::sample_encoders();

        imu_sample_divider++;
        if(imu_sample_divider >= IMU_SAMPLE_DIVIDER)
        {
            imu_sample_divider = 0;
            hw::imu::sample();
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
