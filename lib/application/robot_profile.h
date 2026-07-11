#ifndef ROBOT_PROFILE_H
#define ROBOT_PROFILE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace application
{
    struct task_descriptor
    {
        task_descriptor(TaskFunction_t entry, const char *name, uint32_t stack_size,
            UBaseType_t priority, BaseType_t core_id)
            : entry(entry),
              name(name),
              stack_size(stack_size),
              priority(priority),
              core_id(core_id)
        {
        }

        TaskFunction_t entry;
        const char *name;
        uint32_t stack_size;
        UBaseType_t priority;
        BaseType_t core_id;
    };

    const task_descriptor *task_list(uint8_t &count);
    void init();
}

#endif
