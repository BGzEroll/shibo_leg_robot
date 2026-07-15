#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

namespace web_server
{
    /**
     * @brief 网页遥控输入快照
     */
    struct remote_input
    {
        uint32_t timestamp_us = 0;
        uint16_t held_buttons = 0;
        uint16_t pressed_buttons = 0;
        float axes[6]{};
    };

    /**
     * @brief 读取网页遥控最新输入并消费按键边沿
     *
     * @param out 网页遥控输入输出
     *
     * @return 已收到过有效输入时返回 true
     */
    bool take_remote_input(remote_input &out);

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
