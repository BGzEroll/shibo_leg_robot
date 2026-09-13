#pragma once
#include "FreeRTOS.h"
#include "esp_timer.h"
void taskYIELD();
void vTaskDelayUntil(TickType_t *, TickType_t);
inline TickType_t xTaskGetTickCount(){return fake_time_us / 1000;}
inline void vTaskDelay(TickType_t ticks){fake_time_us += ticks * 1000;}
