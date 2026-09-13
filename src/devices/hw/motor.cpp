#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include "motor.h"

#include "foc_motor.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "hal/mcpwm_ll.h"
#include "esp_rom_sys.h"
#include "sensor.h"
#include "util/latest.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <math.h>

/* ---- 双电机固定配置与快照 ---- */

static constexpr uint32_t ENCODER_TIMEOUT_US = 5000;
static constexpr uint32_t COMMAND_TIMEOUT_US = 20000;
static constexpr int32_t ALIGNMENT_UQ = 12288; // 3 V / 8 V，Q15。
static constexpr int32_t DIRECTION_STEPS = 500;
static constexpr int32_t DIRECTION_MIN_COUNT = 41;
static constexpr float COUNT_TO_RAD = 6.28318530718f / 4096.0f;

static foc_motor motors[2];
static constexpr gpio_num_t ENABLE_PINS[] = {GPIO_NUM_22, GPIO_NUM_12};
static constexpr gpio_num_t PWM_PINS[2][3] =
{
    {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25},
    {GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14}
};
static constexpr uint32_t PWM_PERIOD = 3200; // 160 MHz / (2 * 25 kHz)。
static bool output_enabled[2]{};

struct encoder_sample
{
    uint32_t timestamp_us = 0;
    uint16_t raw = 0;
    bool valid = false;
};

struct rotor_sample
{
    uint32_t sequence = 0;
    encoder_sample axes[2];
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

/* ---- 本机 PWM 与编码器 ---- */

/** @brief 一次性初始化双电机共用的三相中心对齐 25 kHz 定时器 */
static bool init_pwm()
{
    // 固定双电机，先关闭两侧 EN，再连接六路 PWM。
    for(uint8_t axis = 0; axis < 2; axis++)
    {
        gpio_set_level(ENABLE_PINS[axis], 0);
        if(gpio_set_direction(ENABLE_PINS[axis], GPIO_MODE_OUTPUT) != ESP_OK){return false;}
        output_enabled[axis] = false;
        for(uint8_t phase = 0; phase < 3; phase++)
        {
            if(mcpwm_gpio_init(MCPWM_UNIT_0,
                (mcpwm_io_signals_t)(2 * phase + axis), PWM_PINS[axis][phase]) != ESP_OK)
            {
                return false;
            }
        }
    }
    mcpwm_config_t config{};
    config.frequency = 50000;
    config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;
    for(uint8_t i = 0; i < 3; i++)
    {
        if(mcpwm_init(MCPWM_UNIT_0, (mcpwm_timer_t)i, &config) != ESP_OK)
        {
            return false;
        }
        mcpwm_stop(MCPWM_UNIT_0, (mcpwm_timer_t)i);
    }
    MCPWM0.clk_cfg.clk_prescale = 0;
    mcpwm_sync_config_t sync{};
    sync.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    sync.count_direction = MCPWM_TIMER_DIRECTION_UP;
    for(uint8_t i = 0; i < 3; i++)
    {
        MCPWM0.timer[i].timer_cfg0.timer_prescale = 0;
        MCPWM0.timer[i].timer_cfg0.timer_period = PWM_PERIOD;
        MCPWM0.timer[i].timer_cfg0.timer_period_upmethod = 0;
        // 三相 A/B 比较值都在同步零点装载。
        MCPWM0.operators[i].gen_stmp_cfg.gen_a_upmethod = 1;
        MCPWM0.operators[i].gen_stmp_cfg.gen_b_upmethod = 1;
        if(mcpwm_sync_configure(MCPWM_UNIT_0, (mcpwm_timer_t)i, &sync) != ESP_OK ||
           mcpwm_start(MCPWM_UNIT_0, (mcpwm_timer_t)i) != ESP_OK)
        {
            return false;
        }
    }
    return mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0,
        MCPWM_SWSYNC_SOURCE_TEZ) == ESP_OK;
}

/** @brief 使能前等待比较值装载；关闭时立即拉低 EN */
static void set_enabled(uint8_t axis, bool value)
{
    if(value == output_enabled[axis]){return;}
    if(value){esp_rom_delay_us(50);}
    gpio_set_level(ENABLE_PINS[axis], value);
    output_enabled[axis] = value;
}

/** @brief 通过 IDF LL 写入 Q15 占空比，禁止三相更新中途装载 */
static void write_pwm(uint8_t axis, const foc_motor::duty &duty)
{
    MCPWM0.update_cfg.global_up_en = 0;
    for(uint8_t i = 0; i < 3; i++)
    {
        mcpwm_ll_operator_set_compare_value(&MCPWM0, i, axis,
            ((uint32_t)duty.phase[i] * PWM_PERIOD + 16384) >> 15);
    }
    MCPWM0.update_cfg.global_up_en = 1;
}

/** @brief 读取本机 AS5600 的 RAW_ANGLE，失败样本不更新时间戳 */
static encoder_sample read_encoder(uint8_t axis)
{
    encoder_sample value;
    uint8_t data[2];
    i2c_bus &bus = axis == 0 ? hw::sensor::left_bus : hw::sensor::right_bus;
    if(!bus.read_bytes(0x36, 0x0C, data, 2)){return value;}
    value.raw = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    value.timestamp_us = (uint32_t)esp_timer_get_time();
    value.valid = true;
    return value;
}

/* ---- 启动校准，仅在运行任务放行前读取 I2C ---- */

/** @brief 保持输出并读取新编码器样本，通信失败立即退出 */
static bool wait_encoder(uint8_t axis, uint32_t time_ms, const char *stage)
{
    for(uint32_t i = 0; i < time_ms; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        const encoder_sample value = read_encoder(axis);
        if(!value.valid)
        {
            ESP_LOGE("motor", "%s calibration: I2C read failed during %s",
                axis == 0 ? "left" : "right", stage);
            return false;
        }
        motors[axis].sample(value.raw, value.timestamp_us);
    }
    return true;
}

/** @brief 正反扫描判断方向，并平均 32 个新样本校准零电角 */
static bool calibrate(uint8_t axis)
{
    foc_motor &motor = motors[axis];
    if(!wait_encoder(axis, 1, "first sample")){return false;}
    write_pwm(axis, foc_motor::svpwm(ALIGNMENT_UQ, 0xC000));
    set_enabled(axis, true);
    if(!wait_encoder(axis, 300, "settle")){return false;}
    const int64_t start = motor.full_count;
    for(int32_t i = 0; i <= DIRECTION_STEPS; i++)
    {
        write_pwm(axis, foc_motor::svpwm(ALIGNMENT_UQ,
            (uint16_t)(0xC000 + i * 65536 / DIRECTION_STEPS)));
        if(!wait_encoder(axis, 2, "forward scan")){return false;}
    }
    const int64_t forward = motor.full_count;
    for(int32_t i = DIRECTION_STEPS; i >= 0; i--)
    {
        write_pwm(axis, foc_motor::svpwm(ALIGNMENT_UQ,
            (uint16_t)(0xC000 + i * 65536 / DIRECTION_STEPS)));
        if(!wait_encoder(axis, 2, "reverse scan")){return false;}
    }
    const int64_t forward_delta = forward - start;
    const int64_t reverse_delta = motor.full_count - forward;
    ESP_LOGI("motor", "%s calibration: forward=%lld reverse=%lld count (minimum=%ld)",
        axis == 0 ? "left" : "right", (long long)forward_delta,
        (long long)reverse_delta, (long)DIRECTION_MIN_COUNT);
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
        ESP_LOGE("motor", "%s calibration: direction check failed, forward=%lld reverse=%lld",
            axis == 0 ? "left" : "right", (long long)forward_delta, (long long)reverse_delta);
        return false;
    }
    if(!wait_encoder(axis, 300, "settle")){return false;}
    int64_t sum = 0;
    for(uint8_t i = 0; i < 32; i++)
    {
        if(!wait_encoder(axis, 1, "zero average")){return false;}
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
    value.axes[0] = read_encoder(0);
    value.axes[1] = read_encoder(1);
    rotor_latest.set(value);
}

/** @brief 初始化固定硬件并逐台校准，返回时两侧 EN 均关闭 */
bool hw::motor::init()
{
    esp_log_level_set("motor", ESP_LOG_INFO);
    ready = false;
    if(!init_pwm() ||
       !hw::sensor::left_bus.init() || !hw::sensor::right_bus.init() ||
       !rotor_latest.init() || !encoder_latest.init() || !command_latest.init())
    {
        return false;
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        ESP_LOGI("motor", "%s calibration start (Uq=3V)", i == 0 ? "left" : "right");
        const bool calibrated = calibrate(i);
        set_enabled(i, false);
        if(!calibrated)
        {
            ESP_LOGE("motor", "%s calibration failed; motors disabled, startup aborted",
                i == 0 ? "left" : "right");
            return false;
        }
        ESP_LOGI("motor", "%s calibration passed", i == 0 ? "left" : "right");
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
            if(enabled){write_pwm(i, motors[i].update(command.torque_uNm[i], now_us));}
            set_enabled(i, enabled);
        }
        taskYIELD();
    }
}
