#include "motor.h"

#include "as5600.h"
#include "motor_pwm.h"
#include "sensor.h"
#include "util/latest.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <math.h>

/* ---- 双电机固定配置与快照 ---- */

static constexpr float PHASE_RESISTANCE = 21.2f / 2.0f; // 星接线间测量值 / 2。
static constexpr float KT = 0.0796f;
static constexpr float KE = 0.0796f;
static constexpr float BUS_VOLTAGE = 8.0f;
static constexpr uint32_t ENCODER_TIMEOUT_US = 5000;
static constexpr uint32_t COMMAND_TIMEOUT_US = 20000;
static constexpr int32_t ALIGNMENT_UQ = 12288; // 3 V / 8 V，Q15。
static constexpr int32_t DIRECTION_STEPS = 500;
static constexpr int32_t DIRECTION_MIN_COUNT = 41;
static constexpr float COUNT_TO_RAD = 6.28318530718f / 4096.0f;

static as5600 encoders[] = {as5600(hw::sensor::left_bus), as5600(hw::sensor::right_bus)};
static foc_motor motors[] =
{
    foc_motor(PHASE_RESISTANCE, KT, KE, BUS_VOLTAGE),
    foc_motor(PHASE_RESISTANCE, KT, KE, BUS_VOLTAGE)
};
static motor_pwm outputs[] =
{
    motor_pwm(MCPWM_OPR_A, GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_22),
    motor_pwm(MCPWM_OPR_B, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12)
};

struct rotor_sample
{
    uint32_t sequence = 0;
    as5600::sample axes[2];
};

struct torque_command
{
    uint32_t timestamp_us = 0;
    int32_t torque_uNm[2]{};
    bool enabled = false;
};

static util::latest<rotor_sample> rotor_latest;
static util::latest<hw::motor::encoder_state> encoder_latest;
static util::latest<torque_command> command_latest;
static uint32_t sequence = 0;
static bool ready = false;

/* ---- 启动校准，仅在运行任务放行前读取 I2C ---- */

/** @brief 保持输出并读取新编码器样本，通信失败立即退出 */
static bool wait_encoder(uint8_t axis, uint32_t time_ms)
{
    for(uint32_t i = 0; i < time_ms; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        const as5600::sample value = encoders[axis].read();
        if(!value.valid){return false;}
        motors[axis].sample(value.raw, value.timestamp_us);
    }
    return true;
}

/** @brief 正反扫描判断方向，并平均 32 个新样本校准零电角 */
static bool calibrate(uint8_t axis)
{
    foc_motor &motor = motors[axis];
    motor_pwm &output = outputs[axis];
    if(!wait_encoder(axis, 1)){return false;}
    output.write(foc_motor::svpwm(ALIGNMENT_UQ, 0xC000));
    output.set_enabled(true);
    if(!wait_encoder(axis, 300)){return false;}
    const int64_t start = motor.full_count;
    for(int32_t i = 0; i <= DIRECTION_STEPS; i++)
    {
        output.write(foc_motor::svpwm(ALIGNMENT_UQ,
            (uint16_t)(0xC000 + i * 65536 / DIRECTION_STEPS)));
        if(!wait_encoder(axis, 2)){return false;}
    }
    const int64_t forward = motor.full_count;
    for(int32_t i = DIRECTION_STEPS; i >= 0; i--)
    {
        output.write(foc_motor::svpwm(ALIGNMENT_UQ,
            (uint16_t)(0xC000 + i * 65536 / DIRECTION_STEPS)));
        if(!wait_encoder(axis, 2)){return false;}
    }
    const int64_t forward_delta = forward - start;
    const int64_t reverse_delta = motor.full_count - forward;
    int8_t direction = 0;
    if(forward_delta >= DIRECTION_MIN_COUNT && reverse_delta <= -DIRECTION_MIN_COUNT)
    {
        direction = 1;
    }
    else if(forward_delta <= -DIRECTION_MIN_COUNT && reverse_delta >= DIRECTION_MIN_COUNT)
    {
        direction = -1;
    }
    else
    {
        return false;
    }
    if(!wait_encoder(axis, 300)){return false;}
    int64_t sum = 0;
    for(uint8_t i = 0; i < 32; i++)
    {
        if(!wait_encoder(axis, 1)){return false;}
        sum += motor.full_count * 16;
    }
    motor.align(direction, (uint16_t)(sum / 32));
    return true;
}

/* ---- 电机公共 API ---- */

/** @brief 读取 FOC 任务发布的最新轴反馈 */
bool hw::motor::latest_encoder(encoder_state &out)
{
    return encoder_latest.get(out);
}

/** @brief 上层 N·m 原数值转换为微牛米；非有限命令关闭两侧输出 */
bool hw::motor::publish_command(const command &value)
{
    torque_command next;
    next.timestamp_us = value.timestamp_us;
    next.enabled = value.enabled && isfinite(value.left) && isfinite(value.right);
    if(next.enabled)
    {
        // 仅保护 float 到 int32 的转换范围，实际输出由 SVPWM 线性区限幅。
        next.torque_uNm[0] = (int32_t)(fmaxf(-1000.0f, fminf(1000.0f, value.left)) * 1.0e6f);
        next.torque_uNm[1] = (int32_t)(fmaxf(-1000.0f, fminf(1000.0f, value.right)) * 1.0e6f);
    }
    return command_latest.set(next);
}

/** @brief sensor_task 顺序采样双 AS5600，失败也发布以便立即关闭输出 */
void hw::motor::sample_encoders()
{
    rotor_sample value;
    value.sequence = ++sequence;
    value.axes[0] = encoders[0].read();
    value.axes[1] = encoders[1].read();
    rotor_latest.set(value);
}

/** @brief 初始化固定硬件并逐台校准，返回时两侧 EN 均关闭 */
bool hw::motor::init()
{
    ready = false;
    const bool left_output = outputs[0].init();
    const bool right_output = outputs[1].init();
    if(!left_output || !right_output || !motor_pwm::init_timers() ||
       !hw::sensor::left_bus.init() || !hw::sensor::right_bus.init() ||
       !rotor_latest.init() || !encoder_latest.init() || !command_latest.init())
    {
        return false;
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        const bool calibrated = calibrate(i);
        outputs[i].set_enabled(false);
        if(!calibrated){return false;}
    }
    command_latest.set(torque_command{});
    ready = true;
    return true;
}

/** @brief 独立 FOC 循环；新样本更新滤波，每轮预测角度并以 taskYIELD 让出调度 */
void hw::motor::foc_loop(void *arg)
{
    rotor_sample sample;
    torque_command command;
    uint32_t applied_sequence = 0;
    while(true)
    {
        const bool available = rotor_latest.get(sample);
        command_latest.get(command);
        const uint32_t now_us = (uint32_t)esp_timer_get_time();
        const bool fresh = available && sample.axes[0].valid && sample.axes[1].valid &&
            (uint32_t)(now_us - sample.axes[0].timestamp_us) <= ENCODER_TIMEOUT_US &&
            (uint32_t)(now_us - sample.axes[1].timestamp_us) <= ENCODER_TIMEOUT_US;
        const bool enabled = ready && fresh && command.enabled &&
            (uint32_t)(now_us - command.timestamp_us) <= COMMAND_TIMEOUT_US;

        if(available && sample.sequence != applied_sequence)
        {
            applied_sequence = sample.sequence;
            for(uint8_t i = 0; i < 2; i++)
            {
                if(sample.axes[i].valid)
                {
                    motors[i].sample(sample.axes[i].raw, sample.axes[i].timestamp_us);
                }
            }
            encoder_state feedback;
            feedback.valid = fresh && ready;
            feedback.timestamp_us = sample.axes[0].timestamp_us;
            feedback.left_shaft_angle = (float)motors[0].full_count * motors[0].direction * COUNT_TO_RAD;
            feedback.right_shaft_angle = (float)motors[1].full_count * motors[1].direction * COUNT_TO_RAD;
            feedback.left_shaft_velocity = motors[0].speed_mrad_s * motors[0].direction * 0.001f;
            feedback.right_shaft_velocity = motors[1].speed_mrad_s * motors[1].direction * 0.001f;
            encoder_latest.set(feedback);
        }
        for(uint8_t i = 0; i < 2; i++)
        {
            if(enabled){outputs[i].write(motors[i].update(command.torque_uNm[i], now_us));}
            outputs[i].set_enabled(enabled);
        }
        taskYIELD();
    }
}
