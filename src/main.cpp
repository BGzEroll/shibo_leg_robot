#include <Arduino.h>

#include "start.h"

void setup() {
  // put your setup code here, to run once:
  start_init_all();

  vTaskDelete(nullptr);    // 关闭 arduino 的 loop 任务
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(portMAX_DELAY);
}
