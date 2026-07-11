#include "input_providers.h"

#include "esp_http_server.h"
#include "host_comm.h"
#include "xbox_dev.h"
#include "freertos/queue.h"

static constexpr uint32_t WEB_INPUT_TIMEOUT_US = 250000;

// 将 Xbox 设备输入适配为控制器输入源。
class xbox_input_provider_impl : public controller::input_provider
{
    public:
        controller::input_source source() const override
        {
            return controller::input_source::XBOX;
        }

        bool claims_control(uint32_t now_us) override
        {
            return xbox_dev::connected();
        }

        bool latest(controller::input_sample &out) override
        {
            xbox_dev::data data;
            if(!xbox_dev::queue() || xQueuePeek(xbox_dev::queue(), &data, 0) != pdTRUE){return false;}

            out.timestamp_us = data.timestamp_us;
            out.buttons = data.buttons;
            memcpy(out.axes, data.axes, sizeof(out.axes));
            return true;
        }
};

// 将网页遥控输入适配为控制器输入源。
class web_input_provider_impl : public controller::input_provider
{
    public:
        controller::input_source source() const override
        {
            return controller::input_source::WEB;
        }

        bool claims_control(uint32_t now_us) override
        {
            esp_http_server::remote_input_data data;
            QueueHandle_t queue = esp_http_server::remote_queue();
            if(!queue || xQueuePeek(queue, &data, 0) != pdTRUE){return false;}

            return data.timestamp_us != 0 &&
                   (uint32_t)(now_us - data.timestamp_us) <= WEB_INPUT_TIMEOUT_US;
        }

        bool latest(controller::input_sample &out) override
        {
            esp_http_server::remote_input_data data;
            QueueHandle_t queue = esp_http_server::remote_queue();
            if(!queue || xQueuePeek(queue, &data, 0) != pdTRUE){return false;}

            out.timestamp_us = data.timestamp_us;
            out.buttons = data.buttons;
            memcpy(out.axes, data.axes, sizeof(out.axes));
            return true;
        }
};

// 将上位机 UART 输入适配为控制器输入源。
class host_input_provider_impl : public controller::input_provider
{
    public:
        controller::input_source source() const override
        {
            return controller::input_source::HOST;
        }

        bool claims_control(uint32_t now_us) override
        {
            return true;
        }

        bool latest(controller::input_sample &out) override
        {
            host_comm::remote_data data;
            if(!host_comm::remote_queue() ||
               xQueuePeek(host_comm::remote_queue(), &data, 0) != pdTRUE)
            {
                return false;
            }

            out.timestamp_us = data.timestamp_us;
            out.buttons = data.buttons;
            memcpy(out.axes, data.axes, sizeof(out.axes));
            return true;
        }
};

static xbox_input_provider_impl xbox_provider;
static web_input_provider_impl web_provider;
static host_input_provider_impl host_provider;

/**
 * @brief 获取 Xbox 输入适配器
 *
 * @return Xbox 输入适配器
 */
controller::input_provider &application::xbox_input()
{
    return xbox_provider;
}

/**
 * @brief 获取网页输入适配器
 *
 * @return 网页输入适配器
 */
controller::input_provider &application::web_input()
{
    return web_provider;
}

/**
 * @brief 获取上位机输入适配器
 *
 * @return 上位机输入适配器
 */
controller::input_provider &application::host_input()
{
    return host_provider;
}
