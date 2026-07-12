#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace port
{
    template<typename value_type, uint8_t capacity>
    class event_queue
    {
        public:
            void init()
            {
                if(handle){return;}
                handle = xQueueCreateStatic(capacity, sizeof(value_type), storage, &queue);
            }

        public:
            bool publish(const value_type &value)
            {
                return handle && xQueueSend(handle, &value, 0) == pdTRUE;
            }

            bool receive(value_type &out)
            {
                return handle && xQueueReceive(handle, &out, 0) == pdTRUE;
            }

        private:
            StaticQueue_t queue{};
            uint8_t storage[capacity * sizeof(value_type)]{};
            QueueHandle_t handle = nullptr;
    };
}

#endif
