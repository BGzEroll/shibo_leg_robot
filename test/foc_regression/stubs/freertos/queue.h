#pragma once
#include <cstring>
#include <vector>
struct fake_queue
{
    std::vector<unsigned char> data;
    bool written = false;
};
using QueueHandle_t = fake_queue *;
inline QueueHandle_t xQueueCreate(unsigned, unsigned size)
{
    return new fake_queue{std::vector<unsigned char>(size), false};
}
inline int xQueueOverwrite(QueueHandle_t q, const void *value)
{
    std::memcpy(q->data.data(), value, q->data.size()); q->written = true; return 1;
}
inline int xQueuePeek(QueueHandle_t q, void *value, unsigned)
{
    if(!q->written){return 0;}
    std::memcpy(value, q->data.data(), q->data.size()); return 1;
}
