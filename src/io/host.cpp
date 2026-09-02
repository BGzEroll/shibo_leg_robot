#include "host.h"

#include "bus/uart_bus.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "util/latest.h"

/* ---- 上位机通信运行状态 ---- */

static constexpr int16_t VISION_LOST_VALUE = 32767;
static constexpr uint32_t VISION_TIMEOUT_MS = 350;
static constexpr uint8_t INPUT_BUTTON_COUNT = 16;

static util::latest<control::remote_input> remote_latest;
static control::remote_input remote_state;
static uint8_t rx_buf[256];
static uint32_t rx_len = 0;
static uart_bus host_uart(0);
static portMUX_TYPE vision_lock = portMUX_INITIALIZER_UNLOCKED;
static io::host::vision_measurement vision_snapshot;
static uint32_t vision_sequence = 0;

/* ---- 状态访问 API ---- */

/**
 * @brief 读取上位机最新遥控输入快照
 *
 * @param out 输入快照输出
 *
 * @return 已有输入快照时返回 true
 */
bool io::host::latest_input(control::remote_input &out)
{
    return remote_latest.get(out);
}

/**
 * @brief 获取最新视觉测量快照
 *
 * @param out 视觉测量输出
 *
 * @return 当前视觉测量有效时返回 true
 */
bool io::host::latest_vision(io::host::vision_measurement &out)
{
    portENTER_CRITICAL(&vision_lock);
    out = vision_snapshot;
    portEXIT_CRITICAL(&vision_lock);

    if(!out.valid){return false;}
    if((uint32_t)(millis() - out.timestamp_ms) > VISION_TIMEOUT_MS)
    {
        out.valid = false;
        return false;
    }
    return true;
}

/* ---- 串口接收与解析 ---- */

/**
 * @brief 解析上位机发送的遥控输入帧
 *
 * @param frame 接收帧缓冲区
 */
static void parse_remote(control::remote_input &state, uint8_t *frame)
{
    if(frame[3] < 14){return;}

    uint8_t *payload = &frame[4];
    uint16_t held_buttons = (uint16_t)(payload[0] | (payload[1] << 8));
    uint16_t pressed_buttons = held_buttons & (uint16_t)~state.buttons;
    for(uint8_t i = 0; i < INPUT_BUTTON_COUNT; i++)
    {
        if(pressed_buttons & (uint16_t)(1U << i)){state.press_count[i]++;}
    }

    state.sequence++;
    state.timestamp_us = (uint32_t)esp_timer_get_time();
    state.buttons = held_buttons;
    for(uint8_t i = 0; i < 6; i++)
    {
        int16_t raw = (int16_t)(payload[2 + i * 2] | (payload[3 + i * 2] << 8));
        state.axes[i] = (float)raw * 1.0e-3f;
    }
    state.valid = true;
}

/**
 * @brief 解析 MaixCam 视觉测量帧
 *
 * @param frame 接收帧缓冲区
 */
static void parse_vision(uint8_t *frame)
{
    if(frame[3] != 4){return;}

    uint8_t *payload = &frame[4];
    io::host::vision_measurement data;
    data.dx = (int16_t)(payload[0] | (payload[1] << 8));
    data.dy = (int16_t)(payload[2] | (payload[3] << 8));
    data.timestamp_ms = millis();
    data.sequence = ++vision_sequence;
    data.valid = data.dx != VISION_LOST_VALUE && data.dy != VISION_LOST_VALUE;

    portENTER_CRITICAL(&vision_lock);
    vision_snapshot = data;
    portEXIT_CRITICAL(&vision_lock);
}

/**
 * @brief 校验遥控输入帧
 *
 * @param frame 接收帧缓冲区
 * @param payload_len 负载长度
 * @param frame_len 整帧长度
 *
 * @return 校验通过时返回 true
 */
static bool check_remote_frame(uint8_t *frame, uint8_t payload_len, uint32_t frame_len)
{
    uint8_t checksum = 0;
    for(uint32_t i = 0; i < payload_len; i++){checksum += frame[4 + i];}
    return checksum == frame[frame_len - 1];
}

/**
 * @brief 校验视觉帧
 *
 * @param frame 接收帧缓冲区
 * @param frame_len 整帧长度
 *
 * @return 校验通过时返回 true
 */
static bool check_vision_frame(uint8_t *frame, uint32_t frame_len)
{
    uint8_t checksum = 0;
    for(uint32_t i = 0; i < frame_len - 1; i++){checksum += frame[i];}
    return checksum == frame[frame_len - 1];
}

/**
 * @brief 解析 UART 接收缓冲区中的完整帧
 */
static void parse_rx()
{
    uint32_t index = 0;
    while(rx_len - index >= 5)
    {
        if(rx_buf[index] != 0xFF || rx_buf[index + 1] != 0xAA)
        {
            index++;
            continue;
        }

        uint8_t command = rx_buf[index + 2];
        uint8_t payload_len = rx_buf[index + 3];
        uint32_t frame_len = 2 + 1 + 1 + payload_len + 1;
        if(rx_len - index < frame_len){break;}

        uint8_t *frame = &rx_buf[index];
        if(command == 0x01 && check_remote_frame(frame, payload_len, frame_len))
        {
            parse_remote(remote_state, frame);
            remote_latest.set(remote_state);
        }
        else if(command == 0x02 && check_vision_frame(frame, frame_len))
        {
            parse_vision(frame);
        }
        index += frame_len;
    }

    if(index)
    {
        memmove(rx_buf, &rx_buf[index], rx_len - index);
        rx_len -= index;
    }
}

/**
 * @brief 从 UART 读取数据并推进接收解析
 */
static void update_rx()
{
    uint8_t temporary[32];
    uint32_t length = 0;
    if(host_uart.read_bytes(temporary, sizeof(temporary), length) != uart_result::OK){return;}
    if(!length){return;}
    if(rx_len + length > sizeof(rx_buf)){rx_len = 0;}

    memcpy(&rx_buf[rx_len], temporary, length);
    rx_len += length;
    parse_rx();
}

/* ---- host 公共 API 与任务入口 ---- */

/**
 * @brief 初始化上位机 UART 和输入快照
 */
void io::host::init()
{
    host_uart.init();
    remote_latest.init();
    remote_state = control::remote_input{};
    remote_state.stream_id = 1;
    remote_latest.set(remote_state);
    vision_snapshot = io::host::vision_measurement{};
    rx_len = 0;
}

/**
 * @brief 上位机 UART 接收任务入口
 *
 * @param arg RTOS 任务参数
 */
void io::host::task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        update_rx();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
