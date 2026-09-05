#ifndef ESP32_MCU_H
#define ESP32_MCU_H

#include "../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"

#endif
#endif
