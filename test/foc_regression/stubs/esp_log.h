#pragma once
#include <cstdio>
#define ESP_LOGE(tag, ...) do { std::printf(__VA_ARGS__); std::puts(""); } while(0)
#define ESP_LOGI(tag, ...) do { std::printf(__VA_ARGS__); std::puts(""); } while(0)

constexpr int ESP_LOG_INFO = 3;
inline void esp_log_level_set(const char *, int) {}
