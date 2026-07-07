#include "host_comm.h"

#include "balance_core.h"
#include "bus/uart_bus.h"
#include "esp_timer.h"
#include "freertos/task.h"

namespace host_comm
{
    void init();
}

static constexpr int16_t VISION_LOST_VALUE = 32767;
static constexpr uint32_t VISION_TIMEOUT_MS = 350;

static QueueHandle_t rx_queue = nullptr;
static uint8_t rx_buf[256];
static uint32_t rx_len = 0;
static uint8_t tx_buf[160];
static uint32_t send_timer = 0;
static uart_bus host_uart(0);
static portMUX_TYPE vision_lock = portMUX_INITIALIZER_UNLOCKED;
static host_comm::vision_measurement vision_snapshot;
static uint32_t vision_seq = 0;

/**
 * @brief 获取上位机遥控输入队列
 *
 * @return 队列句柄
 */
QueueHandle_t host_comm::remote_queue()
{
    return rx_queue;
}

/**
 * @brief 获取最新视觉测量快照
 *
 * @param out 视觉测量输出
 *
 * @return 当前视觉测量有效时返回 true
 */
bool host_comm::vision_latest(vision_measurement &out)
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

/**
 * @brief 解析上位机发送的 Xbox 输入帧
 *
 * @param frame 接收帧缓冲区
 */
static void parse_xbox(uint8_t *frame)
{
    if(frame[3] < 14){return;}

    uint8_t *p = &frame[4];
    host_comm::remote_data data;
    data.timestamp_us = (uint32_t)esp_timer_get_time();
    data.buttons = (uint16_t)(p[0] | (p[1] << 8));
    for(uint8_t i = 0; i < 6; i++)
    {
        int16_t raw = (int16_t)(p[2 + i * 2] | (p[3 + i * 2] << 8));
        data.axes[i] = (float)raw * 1.0e-3f;
    }

    if(rx_queue)
    {
        xQueueOverwrite(rx_queue, &data);
    }
}

/**
 * @brief 解析 MaixCam 视觉测量帧
 *
 * @param frame 接收帧缓冲区
 */
static void parse_vision(uint8_t *frame)
{
    if(frame[3] != 4){return;}

    uint8_t *p = &frame[4];
    host_comm::vision_measurement data;
    data.dx = (int16_t)(p[0] | (p[1] << 8));
    data.dy = (int16_t)(p[2] | (p[3] << 8));
    data.timestamp_ms = millis();
    data.seq = ++vision_seq;
    data.valid = data.dx != VISION_LOST_VALUE && data.dy != VISION_LOST_VALUE;

    portENTER_CRITICAL(&vision_lock);
    vision_snapshot = data;
    portEXIT_CRITICAL(&vision_lock);
}

/**
 * @brief 校验 Xbox 遥控帧
 *
 * @param frame 接收帧缓冲区
 * @param payload_len 负载长度
 * @param frame_len 整帧长度
 *
 * @return 校验通过时返回 true
 */
static bool check_xbox_frame(uint8_t *frame, uint8_t payload_len, uint32_t frame_len)
{
    uint8_t checksum = 0;
    for(uint32_t i = 0; i < payload_len; i++)
    {
        checksum += frame[4 + i];
    }
    return checksum == frame[frame_len - 1];
}

/**
 * @brief 校验 MaixCam 视觉帧
 *
 * @param frame 接收帧缓冲区
 * @param frame_len 整帧长度
 *
 * @return 校验通过时返回 true
 */
static bool check_vision_frame(uint8_t *frame, uint32_t frame_len)
{
    uint8_t checksum = 0;
    for(uint32_t i = 0; i < frame_len - 1; i++)
    {
        checksum += frame[i];
    }
    return checksum == frame[frame_len - 1];
}

/**
 * @brief 解析 UART 接收缓冲区中的完整帧
 */
static void parse_rx()
{
    uint32_t idx = 0;
    while(rx_len - idx >= 5)
    {
        if(rx_buf[idx] != 0xFF || rx_buf[idx + 1] != 0xAA)
        {
            idx++;
            continue;
        }

        uint8_t cmd = rx_buf[idx + 2];
        uint8_t payload_len = rx_buf[idx + 3];
        uint32_t frame_len = 2 + 1 + 1 + payload_len + 1;
        if(rx_len - idx < frame_len){break;}

        uint8_t *frame = &rx_buf[idx];
        if(cmd == 0x01 && check_xbox_frame(frame, payload_len, frame_len))
        {
            parse_xbox(frame);
        }
        else if(cmd == 0x02 && check_vision_frame(frame, frame_len))
        {
            parse_vision(frame);
        }
        idx += frame_len;
    }

    if(idx)
    {
        memmove(rx_buf, &rx_buf[idx], rx_len - idx);
        rx_len -= idx;
    }
}

/**
 * @brief 从 UART 读取数据并推进接收解析
 */
static void update_rx()
{
    uint8_t tmp[32];
    uint32_t len = 0;
    if(host_uart.read_bytes(tmp, sizeof(tmp), len) != uart_result::OK){return;}
    if(!len){return;}
    if(rx_len + len > sizeof(rx_buf)){rx_len = 0;}

    memcpy(&rx_buf[rx_len], tmp, len);
    rx_len += len;
    parse_rx();
}

/**
 * @brief 向发送缓冲区写入一个浮点字段
 *
 * @param idx 缓冲区写入索引
 * @param id 字段或设备 ID
 * @param value 需要积分的值
 */
static void put_float(uint32_t &idx, uint8_t id, float value)
{
    tx_buf[idx++] = id;
    memcpy(&tx_buf[idx], &value, sizeof(float));
    idx += sizeof(float);
}

/**
 * @brief 按固定周期发送调试状态帧
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void send_status(uint32_t tick_ms)
{
    return;

    if((send_timer += tick_ms) < 20){return;}
    send_timer = 0;

    balance_core::status_snapshot status;
    if(!balance_core::get_status(status)){return;}

    uint32_t idx = 0;
    tx_buf[idx++] = 0xFF;
    tx_buf[idx++] = 0xAA;
    idx++;

    put_float(idx, 0x01, status.pitch_angle);
    put_float(idx, 0x02, status.pitch_rate);
    put_float(idx, 0x03, status.avg_linear_pos);
    put_float(idx, 0x04, status.avg_linear_vel);
    put_float(idx, 0x05, status.yaw_angle);
    put_float(idx, 0x06, status.yaw_rate);
    put_float(idx, 0x07, status.input[0]);
    put_float(idx, 0x08, status.input[1]);
    put_float(idx, 0x09, status.feedback_vector[4]);
    put_float(idx, 0x0A, status.roll_angle);
    put_float(idx, 0x0B, status.leg_height[0]);
    put_float(idx, 0x0C, status.leg_height[1]);

    tx_buf[2] = idx + 1;
    uint8_t checksum = 0;
    for(uint32_t i = 2; i < idx; i++){checksum += tx_buf[i];}
    tx_buf[idx++] = checksum;
    host_uart.write_bytes(tx_buf, idx);
}

/**
 * @brief 初始化上位机通信模块
 */
void host_comm::init()
{
    host_uart.init();
    rx_queue = xQueueCreate(1, sizeof(host_comm::remote_data));
}

/**
 * @brief 上位机通信任务入口
 *
 * @param arg RTOS 任务参数
 */
void host_comm::task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        update_rx();
        send_status(1);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
