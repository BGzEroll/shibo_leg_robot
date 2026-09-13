#pragma once
#include <stdint.h>
using esp_err_t = int;
constexpr int ESP_OK = 0;
enum gpio_num_t {GPIO_NUM_5=5, GPIO_NUM_12=12, GPIO_NUM_14=14, GPIO_NUM_18=18,
    GPIO_NUM_19=19, GPIO_NUM_22=22, GPIO_NUM_23=23, GPIO_NUM_25=25,
    GPIO_NUM_26=26, GPIO_NUM_27=27, GPIO_NUM_32=32, GPIO_NUM_33=33};
constexpr int GPIO_MODE_OUTPUT = 1;
constexpr int GPIO_PULLUP_ENABLE = 1;
extern int fake_gpio[40];
inline int gpio_set_level(gpio_num_t pin, int value){fake_gpio[pin] = value; return 0;}
inline int gpio_set_direction(gpio_num_t, int){return 0;}
