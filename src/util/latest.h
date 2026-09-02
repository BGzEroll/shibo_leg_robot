#ifndef UTIL_LATEST_H
#define UTIL_LATEST_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace util
{
    /**
     * @brief 通过长度为一的队列交换跨任务最新值
     *
     * 该类型只保留最后一次写入的快照，不提供排队、订阅或消息分发语义。
     *
     * @tparam value_type 快照类型
     */
    template<typename value_type>
    class latest
    {
        public:
            /**
             * @brief 创建单槽快照队列
             *
             * @return 创建成功或已经初始化时返回 true
             */
            bool init()
            {
                if(queue_){return true;}
                queue_ = xQueueCreate(1, sizeof(value_type));
                return queue_ != nullptr;
            }

            /**
             * @brief 覆盖为最新快照
             *
             * @param value 待发布快照
             *
             * @return 发布成功时返回 true
             */
            bool set(const value_type &value)
            {
                return queue_ && xQueueOverwrite(queue_, &value) == pdTRUE;
            }

            /**
             * @brief 读取当前最新快照
             *
             * @param out 快照输出
             *
             * @return 队列已有快照时返回 true
             */
            bool get(value_type &out) const
            {
                return queue_ && xQueuePeek(queue_, &out, 0) == pdTRUE;
            }

        private:
            QueueHandle_t queue_ = nullptr;
    };
}

#endif
