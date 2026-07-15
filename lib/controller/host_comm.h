#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <Arduino.h>

namespace host_comm
{
    /**
     * @brief 上位机模块发布的最新遥控输入状态
     *
     * sequence 为本机成功解析的遥控帧序号，press_count 保存各按钮累计
     * 按下次数，不改变上位机线协议格式。
     */
    struct input
    {
        uint32_t stream_id = 0;
        uint32_t sequence = 0;
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        uint16_t press_count[16]{};
        float axes[6]{};
        bool valid = false;
    };

    struct vision_measurement
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint32_t timestamp_ms = 0;
        uint32_t seq = 0;
        bool valid = false;
    };

    bool peek_input(input &out);
    bool vision_latest(vision_measurement &out);
    void init();
    void task_entry(void *arg);
}

#endif
