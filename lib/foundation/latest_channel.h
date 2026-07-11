#ifndef LATEST_CHANNEL_H
#define LATEST_CHANNEL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <type_traits>

namespace foundation
{
    /**
     * @brief 使用长度为 1 的 FreeRTOS 队列保存最新值
     *
     * @tparam value_type 通道数据类型
     */
    template<typename value_type>
    class latest_channel
    {
        public:
            latest_channel() = default;
            latest_channel(const latest_channel &) = delete;
            latest_channel &operator=(const latest_channel &) = delete;

        public:
            bool init()
            {
                static_assert(
                    std::is_trivially_copyable<value_type>::value,
                    "latest_channel requires a trivially copyable value type");
                if(queue){return true;}
                queue = xQueueCreate(1, sizeof(value_type));
                return queue != nullptr;
            }

        public:
            bool ready() const
            {
                return queue != nullptr;
            }

            bool publish(const value_type &value)
            {
                if(!queue){return false;}
                return xQueueOverwrite(queue, &value) == pdPASS;
            }

            bool latest(value_type &out) const
            {
                if(!queue){return false;}
                return xQueuePeek(queue, &out, 0) == pdTRUE;
            }

        private:
            QueueHandle_t queue = nullptr;
    };
}

#endif
