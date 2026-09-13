#include "battery.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "util/latest.h"

/* ---- 电池采样配置与运行状态 ---- */

static constexpr adc1_channel_t BATTERY_ADC_CHANNEL = ADC1_CHANNEL_7;
static constexpr adc_bits_width_t BATTERY_ADC_WIDTH = ADC_WIDTH_BIT_12;
static constexpr adc_atten_t BATTERY_ADC_ATTEN = ADC_ATTEN_DB_12;
static constexpr adc_unit_t BATTERY_ADC_UNIT = ADC_UNIT_1;
static constexpr float BATTERY_DIVIDER_RATIO = 3.97f;
static constexpr float LOW_VOLTAGE = 7.4f;
static constexpr float RECOVER_VOLTAGE = 7.5f;
static constexpr uint8_t ADC_SAMPLE_COUNT = 16;
static constexpr uint8_t LOW_CONFIRM_COUNT = 5;
static constexpr uint8_t NORMAL_CONFIRM_COUNT = 5;
static constexpr uint8_t RECOVER_CONFIRM_COUNT = 10;

static util::latest<hw::battery::state> battery_latest;
static esp_adc_cal_characteristics_t adc_characteristics;
static hw::battery::state current_state;
static uint8_t low_confirm_count = 0;
static uint8_t normal_confirm_count = 0;
static uint8_t recover_confirm_count = 0;

/* ---- 电池采样与状态判定 ---- */

/**
 * @brief 读取经过多次平均和校准换算的电池电压
 *
 * @return 电池电压，单位伏
 */
static float read_voltage()
{
    uint32_t raw_sum = 0;
    for(uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
    {
        raw_sum += (uint32_t)adc1_get_raw(BATTERY_ADC_CHANNEL);
    }

    uint32_t raw_average = raw_sum / ADC_SAMPLE_COUNT;
    uint32_t adc_voltage_mv =
        esp_adc_cal_raw_to_voltage(raw_average, &adc_characteristics);
    return (float)adc_voltage_mv * BATTERY_DIVIDER_RATIO * 1.0e-3f;
}

/**
 * @brief 根据连续采样结果更新低电状态
 *
 * @param voltage 当前电池电压，单位伏
 */
static void update_low_state(float voltage)
{
    if(!current_state.valid)
    {
        if(voltage < LOW_VOLTAGE)
        {
            normal_confirm_count = 0;
            if(low_confirm_count < LOW_CONFIRM_COUNT){low_confirm_count++;}
            if(low_confirm_count >= LOW_CONFIRM_COUNT)
            {
                current_state.low = true;
                current_state.valid = true;
                low_confirm_count = 0;
            }
        }
        else
        {
            low_confirm_count = 0;
            if(normal_confirm_count < NORMAL_CONFIRM_COUNT){normal_confirm_count++;}
            if(normal_confirm_count >= NORMAL_CONFIRM_COUNT)
            {
                current_state.low = false;
                current_state.valid = true;
                normal_confirm_count = 0;
            }
        }
        return;
    }

    if(voltage < LOW_VOLTAGE)
    {
        recover_confirm_count = 0;
        if(current_state.low){return;}

        if(low_confirm_count < LOW_CONFIRM_COUNT){low_confirm_count++;}
        if(low_confirm_count >= LOW_CONFIRM_COUNT)
        {
            current_state.low = true;
            low_confirm_count = 0;
        }
        return;
    }

    if(voltage >= RECOVER_VOLTAGE)
    {
        low_confirm_count = 0;
        if(!current_state.low){return;}

        if(recover_confirm_count < RECOVER_CONFIRM_COUNT){recover_confirm_count++;}
        if(recover_confirm_count >= RECOVER_CONFIRM_COUNT)
        {
            current_state.low = false;
            recover_confirm_count = 0;
        }
        return;
    }

    low_confirm_count = 0;
    recover_confirm_count = 0;
}

/**
 * @brief 采样并发布最新电池状态
 */
static void update_status()
{
    current_state.timestamp_ms = millis();
    current_state.voltage = read_voltage();
    update_low_state(current_state.voltage);
    battery_latest.set(current_state);
}

/**
 * @brief 读取最新电池状态
 *
 * @param out 电池状态输出
 *
 * @return 已有电池状态时返回 true
 */
bool hw::battery::latest(hw::battery::state &out)
{
    return battery_latest.get(out);
}

/**
 * @brief 初始化电池 ADC 和状态快照
 */
void hw::battery::init()
{
    battery_latest.init();
    adc1_config_width(BATTERY_ADC_WIDTH);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN);
    esp_adc_cal_characterize(
        BATTERY_ADC_UNIT,
        BATTERY_ADC_ATTEN,
        BATTERY_ADC_WIDTH,
        0,
        &adc_characteristics);
    update_status();
}

/**
 * @brief 执行一次电池采样
 */
void hw::battery::update()
{
    update_status();
}
