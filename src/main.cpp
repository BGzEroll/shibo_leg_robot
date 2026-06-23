#include <Arduino.h>
#include "start.h"


/**
 * @brief Arduino 启动回调
 */
void setup() {
  // put your setup code here, to run once:
  start_init_all();
}

/**
 * @brief Arduino 主循环回调
 */
void loop() {
  // put your main code here, to run repeatedly:
  delay(10000000);
}
