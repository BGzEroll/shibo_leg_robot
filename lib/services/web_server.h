#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

namespace web_server
{
    /**
     * @brief Web 模块发布的最新遥控输入状态
     *
     * stream_id 对应 WebSocket 控制连接代次，sequence 保留客户端协议序号，
     * press_count 保存服务端累计接收的按钮按下次数。
     */
    struct input
    {
        uint32_t stream_id = 0;
        uint32_t sequence = 0;
        uint32_t timestamp_us = 0;
        uint16_t held_buttons = 0;
        uint16_t press_count[16]{};
        float axes[6]{};
        bool valid = false;
    };

    /**
     * @brief 读取网页遥控最新输入快照
     *
     * @param out 网页遥控输入输出
     *
     * @return 队列存在且已有快照时返回 true
     */
    bool peek_input(input &out);

    /**
     * @brief 初始化原生 HTTP 和 WebSocket 服务
     *
     * @return 服务及全部路由启动成功时返回 true
     */
    bool init();

    /**
     * @brief Web 服务维护任务入口
     *
     * @param arg RTOS 任务参数
     */
    void task_entry(void *arg);
}

#endif
