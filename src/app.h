#ifndef APP_H
#define APP_H

#include <stdint.h>

namespace app
{
    enum class startup_stage : uint8_t
    {
        STARTING = 0,
        SERVICES,
        TASKS,
        BATTERY,
        INDICATOR,
        SERVO,
        IMU,
        MOTOR,
        CONTROL,
        READY,
        TASK_CREATION_FAILED
    };

    bool ready();
    startup_stage stage();
    void start();
}

#endif
