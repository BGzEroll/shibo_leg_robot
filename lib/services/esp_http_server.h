#ifndef ESP_HTTP_SERVER_H
#define ESP_HTTP_SERVER_H

#include <Arduino.h>

namespace esp_http_server
{
	void init();
	void task_entry(void *arg);
}

#endif
