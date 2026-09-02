#include "web.h"

#include "web_pages.h"

#include "control/control.h"
#include "hw/gamepad.h"
#include "hw/wifi.h"
#include "esp_timer.h"
#include "util/latest.h"
#include <WiFi.h>
#include <esp_http_server.h>
#include <string.h>

/* ---- HTTP 页面与运行状态 ---- */

static constexpr uint32_t BLE_SCAN_MS = 4000;
static constexpr uint32_t REMOTE_LEASE_TIMEOUT_MS = 500;
static constexpr uint32_t REMOTE_ACK_INTERVAL_MS = 200;
static constexpr uint8_t BLE_SCAN_MAX = 24;
static constexpr uint8_t REMOTE_BUTTON_COUNT = 16;
static constexpr uint8_t REMOTE_PROTOCOL_VERSION = 1;
static constexpr uint8_t REMOTE_INPUT_TYPE = 0x01;
static constexpr uint8_t REMOTE_STATUS_TYPE = 0x81;
static constexpr uint8_t REMOTE_STATUS_OK = 0;
static constexpr uint8_t REMOTE_STATUS_INVALID_FRAME = 1;
static constexpr uint8_t REMOTE_STATUS_XBOX_ACTIVE = 2;
static constexpr uint8_t REMOTE_STATUS_CONTROL_LOCKED = 3;
static constexpr uint8_t REMOTE_STATUS_AP_FORBIDDEN = 4;
static constexpr uint8_t REMOTE_STATUS_INTERNAL_ERROR = 5;
static constexpr uint8_t REMOTE_STATUS_ACTIVE = 1 << 0;
static constexpr uint8_t REMOTE_STATUS_LOW_LATENCY = 1 << 1;
static constexpr uint16_t REMOTE_INPUT_FRAME_SIZE = 22;
static constexpr uint16_t REMOTE_STATUS_FRAME_SIZE = 8;
static constexpr uint16_t HTTP_FORM_MAX_SIZE = 192;
static constexpr int32_t REMOTE_AXIS_MIN = -1000;
static constexpr int32_t REMOTE_AXIS_MAX = 1000;

static httpd_handle_t server = nullptr;
static util::latest<control::remote_input> remote_latest;
static portMUX_TYPE remote_lock = portMUX_INITIALIZER_UNLOCKED;
static control::remote_input remote_state;
static int remote_socket = -1;
static uint32_t remote_stream_id = 0;
static uint32_t remote_timestamp_ms = 0;
static uint32_t remote_ack_timestamp_ms = 0;
static uint32_t remote_sequence = 0;
static bool remote_sequence_valid = false;


/**
 * @brief 从协议帧读取小端 16 位无符号整数
 *
 * @param data 协议帧数据
 *
 * @return 解码后的整数
 */
static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(data[0] | (data[1] << 8));
}

/**
 * @brief 从协议帧读取小端 32 位无符号整数
 *
 * @param data 协议帧数据
 *
 * @return 解码后的整数
 */
static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief 查询网页遥控连接是否处于激活状态
 *
 * @return 已有激活连接时返回 true
 */
static bool remote_active()
{
    bool active;
    portENTER_CRITICAL(&remote_lock);
    active = remote_socket >= 0;
    portEXIT_CRITICAL(&remote_lock);
    return active;
}

/**
 * @brief 清除网页遥控租约与输入
 *
 * @param socket 待释放连接，负值表示无条件清理
 */
static void clear_remote_session(int socket)
{
    bool cleared = false;
    portENTER_CRITICAL(&remote_lock);
    if(socket < 0 || remote_socket == socket)
    {
        remote_socket = -1;
        remote_timestamp_ms = 0;
        remote_ack_timestamp_ms = 0;
        remote_sequence = 0;
        remote_sequence_valid = false;
        remote_state = control::remote_input{};
        remote_state.stream_id = remote_stream_id;
        remote_latest.set(remote_state);
        cleared = true;
    }
    portEXIT_CRITICAL(&remote_lock);

    if(cleared){hw::wifi::set_low_latency_mode(remote_active());}
}

/**
 * @brief 更新网页遥控租约失效状态
 */
static void update_remote_session()
{
    int socket;
    uint32_t timestamp_ms;
    portENTER_CRITICAL(&remote_lock);
    socket = remote_socket;
    timestamp_ms = remote_timestamp_ms;
    portEXIT_CRITICAL(&remote_lock);

    if(socket < 0){return;}

    uint32_t now_ms = millis();
    bool expired = timestamp_ms == 0 ||
        (uint32_t)(now_ms - timestamp_ms) > REMOTE_LEASE_TIMEOUT_MS;
    if(hw::wifi::config_portal_active() || hw::gamepad::connected() || expired)
    {
        clear_remote_session(socket);
        if(server){httpd_sess_trigger_close(server, socket);}
    }
}

/**
 * @brief 发送固定 HTTP 响应
 *
 * @param req HTTP 请求
 * @param status HTTP 状态文本
 * @param type 响应内容类型
 * @param body 响应正文
 *
 * @return ESP-IDF 响应结果
 */
static esp_err_t send_response(httpd_req_t *req, const char *status,
    const char *type, const char *body)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, type);
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief 发送 JSON HTTP 响应
 *
 * @param req HTTP 请求
 * @param status HTTP 状态文本
 * @param body JSON 正文
 *
 * @return ESP-IDF 响应结果
 */
static esp_err_t send_json(httpd_req_t *req, const char *status, const char *body)
{
    return send_response(req, status, "application/json", body);
}

/**
 * @brief 判断当前是否允许执行管理操作
 *
 * @param req HTTP 请求
 *
 * @return 没有网页遥控连接时返回 true
 */
static bool management_available(httpd_req_t *req)
{
    if(!remote_active()){return true;}
    send_json(req, "409 Conflict",
        "{\"ok\":false,\"error\":\"遥控连接期间不可操作\"}");
    return false;
}

/**
 * @brief 读取有界 HTTP 请求正文
 *
 * @param req HTTP 请求
 * @param out 正文输出缓冲区
 * @param capacity 输出缓冲区容量
 *
 * @return 完整读取正文时返回 true
 */
static bool read_request_body(httpd_req_t *req, char *out, uint16_t capacity)
{
    if(req->content_len == 0 || req->content_len >= capacity){return false;}

    uint16_t received = 0;
    while(received < req->content_len)
    {
        int result = httpd_req_recv(req, out + received, req->content_len - received);
        if(result <= 0){return false;}
        received += (uint16_t)result;
    }
    out[received] = '\0';
    return true;
}

/**
 * @brief 将十六进制字符转换为数值
 *
 * @param value 十六进制字符
 * @param out 数值输出
 *
 * @return 字符有效时返回 true
 */
static bool decode_hex(char value, uint8_t &out)
{
    if(value >= '0' && value <= '9')
    {
        out = (uint8_t)(value - '0');
        return true;
    }
    if(value >= 'A' && value <= 'F')
    {
        out = (uint8_t)(value - 'A' + 10);
        return true;
    }
    if(value >= 'a' && value <= 'f')
    {
        out = (uint8_t)(value - 'a' + 10);
        return true;
    }
    return false;
}

/**
 * @brief 从 URL 表单中解码指定字段
 *
 * @param form URL 编码表单
 * @param key 字段名
 * @param out 字段输出缓冲区
 * @param capacity 输出缓冲区容量
 *
 * @return 找到且完整解码字段时返回 true
 */
static bool decode_form_value(const char *form, const char *key, char *out, uint16_t capacity)
{
    size_t key_len = strlen(key);
    const char *field = form;
    while(*field)
    {
        const char *equals = strchr(field, '=');
        const char *end = strchr(field, '&');
        if(!end){end = field + strlen(field);}
        if(equals && equals < end && (size_t)(equals - field) == key_len &&
           strncmp(field, key, key_len) == 0)
        {
            uint16_t length = 0;
            const char *cursor = equals + 1;
            while(cursor < end)
            {
                if(length + 1 >= capacity){return false;}
                if(*cursor == '+')
                {
                    out[length++] = ' ';
                    cursor++;
                }
                else if(*cursor == '%')
                {
                    if(cursor + 2 >= end){return false;}
                    uint8_t high = 0;
                    uint8_t low = 0;
                    if(!decode_hex(cursor[1], high) || !decode_hex(cursor[2], low)){return false;}
                    uint8_t decoded = (uint8_t)((high << 4) | low);
                    if(decoded == 0){return false;}
                    out[length++] = (char)decoded;
                    cursor += 3;
                }
                else
                {
                    out[length++] = *cursor++;
                }
            }
            out[length] = '\0';
            return true;
        }
        field = *end ? end + 1 : end;
    }
    return false;
}

/**
 * @brief 检查 URL 表单是否包含指定字段
 *
 * @param form URL 编码表单
 * @param key 字段名
 *
 * @return 找到字段时返回 true
 */
static bool form_contains_key(const char *form, const char *key)
{
    size_t key_len = strlen(key);
    const char *field = form;
    while(*field)
    {
        const char *equals = strchr(field, '=');
        const char *end = strchr(field, '&');
        if(!end){end = field + strlen(field);}
        if(equals && equals < end && (size_t)(equals - field) == key_len &&
           strncmp(field, key, key_len) == 0)
        {
            return true;
        }
        field = *end ? end + 1 : end;
    }
    return false;
}

/**
 * @brief 去除字符串首尾 ASCII 空白
 *
 * @param value 待处理字符串
 */
static void trim_ascii(char *value)
{
    char *begin = value;
    while(*begin == ' ' || *begin == '\t' || *begin == '\r' || *begin == '\n'){begin++;}
    if(begin != value){memmove(value, begin, strlen(begin) + 1);}

    size_t length = strlen(value);
    while(length > 0)
    {
        char tail = value[length - 1];
        if(tail != ' ' && tail != '\t' && tail != '\r' && tail != '\n'){break;}
        value[--length] = '\0';
    }
}

/**
 * @brief 发送 JSON 转义字符串分块
 *
 * @param req HTTP 请求
 * @param value 原始字符串
 *
 * @return 发送成功时返回 true
 */
static bool send_json_string(httpd_req_t *req, const String &value)
{
    char buffer[96];
    uint16_t length = 0;
    buffer[length++] = '"';
    for(uint16_t i = 0; i < value.length(); i++)
    {
        char encoded[2];
        uint8_t encoded_len = 1;
        encoded[0] = value[i];
        if(value[i] == '"' || value[i] == '\\')
        {
            encoded[0] = '\\';
            encoded[1] = value[i];
            encoded_len = 2;
        }
        else if(value[i] == '\n' || value[i] == '\r')
        {
            encoded[0] = '\\';
            encoded[1] = value[i] == '\n' ? 'n' : 'r';
            encoded_len = 2;
        }
        if(length + encoded_len >= sizeof(buffer))
        {
            if(httpd_resp_send_chunk(req, buffer, length) != ESP_OK){return false;}
            length = 0;
        }
        memcpy(buffer + length, encoded, encoded_len);
        length += encoded_len;
    }
    buffer[length++] = '"';
    return httpd_resp_send_chunk(req, buffer, length) == ESP_OK;
}

/* ---- HTTP 请求处理 ---- */

/**
 * @brief 处理根页面请求
 */
static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    if(httpd_resp_sendstr_chunk(req, io::web_pages::console_html_prefix()) != ESP_OK){return ESP_FAIL;}
    if(!hw::wifi::config_portal_active() &&
       httpd_resp_sendstr_chunk(req, io::web_pages::console_html_modules()) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if(httpd_resp_sendstr_chunk(req, io::web_pages::console_html_suffix()) != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理 WiFi 配置页面请求
 */
static esp_err_t handle_wifi_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", io::web_pages::wifi_html());
}

/**
 * @brief 处理蓝牙设置页面请求
 */
static esp_err_t handle_bluetooth_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", io::web_pages::bluetooth_html());
}

/**
 * @brief 处理舵机中位校准页面请求
 */
static esp_err_t handle_servo_calibration_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", io::web_pages::servo_calibration_html());
}

/**
 * @brief 处理手机遥控页面请求
 */
static esp_err_t handle_remote_root(httpd_req_t *req)
{
    if(hw::wifi::config_portal_active())
    {
        return send_response(req, "403 Forbidden", "text/plain; charset=utf-8",
            "AP 配网模式下不可用");
    }

    return send_response(req, "200 OK", "text/html; charset=utf-8", io::web_pages::remote_html());
}

/**
 * @brief 处理 WiFi 扫描请求
 */
static esp_err_t handle_wifi_scan(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}

    int32_t count = WiFi.scanNetworks(false, true);
    httpd_resp_set_type(req, "application/json");
    if(httpd_resp_send_chunk(req, "[", 1) != ESP_OK){return ESP_FAIL;}
    for(int32_t i = 0; i < count; i++)
    {
        if(i && httpd_resp_send_chunk(req, ",", 1) != ESP_OK){return ESP_FAIL;}
        if(httpd_resp_sendstr_chunk(req, "{\"ssid\":") != ESP_OK){return ESP_FAIL;}
        if(!send_json_string(req, WiFi.SSID(i))){return ESP_FAIL;}

        char fields[80];
        int length = snprintf(fields, sizeof(fields), ",\"rssi\":%ld,\"secure\":%s}",
            (long)WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true");
        if(length <= 0 || length >= (int)sizeof(fields) ||
           httpd_resp_send_chunk(req, fields, length) != ESP_OK)
        {
            return ESP_FAIL;
        }
    }
    WiFi.scanDelete();
    if(httpd_resp_send_chunk(req, "]", 1) != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理 WiFi 连接和保存请求
 */
static esp_err_t handle_connect(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}

    char form[HTTP_FORM_MAX_SIZE];
    char ssid[33];
    char password[65]{};
    if(!read_request_body(req, form, sizeof(form)) ||
       !decode_form_value(form, "ssid", ssid, sizeof(ssid)))
    {
        return send_json(req, "400 Bad Request",
            "{\"ok\":false,\"error\":\"请求格式无效\"}");
    }
    if(form_contains_key(form, "password") &&
       !decode_form_value(form, "password", password, sizeof(password)))
    {
        return send_json(req, "400 Bad Request",
            "{\"ok\":false,\"error\":\"请求格式无效\"}");
    }
    trim_ascii(ssid);
    if(!ssid[0])
    {
        return send_json(req, "400 Bad Request",
            "{\"ok\":false,\"error\":\"SSID 为空\"}");
    }

    IPAddress ip;
    bool ok = hw::wifi::connect_and_save(String(ssid), String(password), ip);
    if(!ok)
    {
        return send_json(req, "200 OK",
            "{\"ok\":false,\"error\":\"连接超时\"}");
    }

    char json[64];
    IPAddress result_ip = ip;
    snprintf(json, sizeof(json), "{\"ok\":true,\"ip\":\"%u.%u.%u.%u\"}",
        result_ip[0], result_ip[1], result_ip[2], result_ip[3]);
    return send_json(req, "200 OK", json);
}

/**
 * @brief 处理 Xbox 状态查询请求
 */
static esp_err_t handle_xbox_status(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    const char *prefix = hw::gamepad::connected() ?
        "{\"connected\":true,\"target\":" :
        "{\"connected\":false,\"target\":";
    if(httpd_resp_sendstr_chunk(req, prefix) != ESP_OK){return ESP_FAIL;}
    if(!send_json_string(req, hw::gamepad::target_address())){return ESP_FAIL;}
    if(httpd_resp_send_chunk(req, "}", 1) != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理 BLE 扫描请求
 */
static esp_err_t handle_ble_scan(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}

    hw::gamepad::ble_device devices[BLE_SCAN_MAX];
    uint8_t count = 0;
    bool ok = hw::gamepad::scan_ble(devices, BLE_SCAN_MAX, count, BLE_SCAN_MS);
    if(!ok)
    {
        return send_json(req, "500 Internal Server Error",
            "{\"ok\":false,\"devices\":[]}");
    }

    httpd_resp_set_type(req, "application/json");
    if(httpd_resp_sendstr_chunk(req, "{\"ok\":true,\"devices\":[") != ESP_OK)
    {
        return ESP_FAIL;
    }
    for(uint8_t i = 0; i < count; i++)
    {
        if(i && httpd_resp_send_chunk(req, ",", 1) != ESP_OK){return ESP_FAIL;}
        if(httpd_resp_sendstr_chunk(req, "{\"address\":") != ESP_OK){return ESP_FAIL;}
        if(!send_json_string(req, devices[i].address)){return ESP_FAIL;}
        if(httpd_resp_sendstr_chunk(req, ",\"name\":") != ESP_OK){return ESP_FAIL;}
        if(!send_json_string(req, devices[i].name)){return ESP_FAIL;}

        char fields[112];
        int length = snprintf(fields, sizeof(fields),
            ",\"rssi\":%d,\"xbox\":%s,\"connectable\":%s}",
            devices[i].rssi,
            devices[i].xbox ? "true" : "false",
            devices[i].connectable ? "true" : "false");
        if(length <= 0 || length >= (int)sizeof(fields) ||
           httpd_resp_send_chunk(req, fields, length) != ESP_OK)
        {
            return ESP_FAIL;
        }
    }
    if(httpd_resp_sendstr_chunk(req, "]}") != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理目标 Xbox 手柄选择请求
 */
static esp_err_t handle_xbox_select(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}

    char form[HTTP_FORM_MAX_SIZE];
    char address[18];
    if(!read_request_body(req, form, sizeof(form)) ||
       !decode_form_value(form, "address", address, sizeof(address)))
    {
        return send_json(req, "400 Bad Request",
            "{\"ok\":false,\"error\":\"请求格式无效\"}");
    }
    if(!hw::gamepad::set_target_address(String(address)))
    {
        return send_json(req, "400 Bad Request",
            "{\"ok\":false,\"error\":\"蓝牙地址无效\"}");
    }

    return send_json(req, "200 OK", "{\"ok\":true}");
}

/**
 * @brief 处理舵机中位校准执行请求
 */
static esp_err_t handle_servo_middle_calibration(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}
    if(hw::wifi::config_portal_active())
    {
        return send_json(req, "403 Forbidden",
            "{\"ok\":false,\"error\":\"AP 配网模式下不可用\"}");
    }

    control::request_middle_calibration();
    return send_json(req, "200 OK", "{\"ok\":true}");
}

/**
 * @brief 处理舵机中位校准状态查询请求
 */
static esp_err_t handle_servo_middle_calibration_status(httpd_req_t *req)
{
    return send_json(req, "200 OK", control::middle_calibration_success() ?
        "{\"ok\":true,\"success\":true}" :
        "{\"ok\":true,\"success\":false}");
}

/**
 * @brief 将 32 位无符号整数写入小端协议字段
 *
 * @param data 协议字段输出
 * @param value 待写入数值
 */
static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

/**
 * @brief 发送网页遥控状态帧
 *
 * @param req WebSocket 请求
 * @param status 状态码
 * @param sequence 已处理输入序号
 *
 * @return ESP-IDF 发送结果
 */
static esp_err_t send_remote_status(httpd_req_t *req, uint8_t status, uint32_t sequence)
{
    uint8_t payload[REMOTE_STATUS_FRAME_SIZE]{};
    payload[0] = REMOTE_PROTOCOL_VERSION;
    payload[1] = REMOTE_STATUS_TYPE;
    payload[2] = status;
    payload[3] = remote_active() ?
        (uint8_t)(REMOTE_STATUS_ACTIVE | REMOTE_STATUS_LOW_LATENCY) : 0;
    write_u32_le(payload + 4, sequence);

    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_BINARY;
    frame.payload = payload;
    frame.len = sizeof(payload);
    return httpd_ws_send_frame(req, &frame);
}

/**
 * @brief 接收并应用定长网页遥控输入帧
 *
 * 帧格式固定为版本、类型、32 位序号、按住按钮、按下边沿和六个小端
 * int16 轴值，共 22 字节。
 *
 * @param req WebSocket 请求
 *
 * @return ESP-IDF 处理结果
 */
static esp_err_t handle_remote_websocket(httpd_req_t *req)
{
    int socket = httpd_req_to_sockfd(req);
    if(req->method == HTTP_GET)
    {
        uint8_t reject_status = REMOTE_STATUS_OK;
        if(hw::wifi::config_portal_active())
        {
            reject_status = REMOTE_STATUS_AP_FORBIDDEN;
        }
        else if(hw::gamepad::connected())
        {
            reject_status = REMOTE_STATUS_XBOX_ACTIVE;
        }

        bool acquired = false;
        portENTER_CRITICAL(&remote_lock);
        if(reject_status == REMOTE_STATUS_OK && remote_socket < 0)
        {
            remote_socket = socket;
            remote_timestamp_ms = millis();
            remote_ack_timestamp_ms = 0;
            remote_sequence = 0;
            remote_sequence_valid = false;
            remote_state = control::remote_input{};
            remote_state.stream_id = ++remote_stream_id;
            remote_latest.set(remote_state);
            acquired = true;
        }
        portEXIT_CRITICAL(&remote_lock);

        if(!acquired)
        {
            if(reject_status == REMOTE_STATUS_OK){reject_status = REMOTE_STATUS_CONTROL_LOCKED;}
            send_remote_status(req, reject_status, 0);
            return ESP_FAIL;
        }

        hw::wifi::set_low_latency_mode(true);
        return ESP_OK;
    }

    if(hw::gamepad::connected())
    {
        send_remote_status(req, REMOTE_STATUS_XBOX_ACTIVE, 0);
        clear_remote_session(socket);
        return ESP_FAIL;
    }

    bool owner;
    portENTER_CRITICAL(&remote_lock);
    owner = remote_socket == socket;
    portEXIT_CRITICAL(&remote_lock);
    if(!owner)
    {
        send_remote_status(req, REMOTE_STATUS_INTERNAL_ERROR, 0);
        return ESP_FAIL;
    }

    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t result = httpd_ws_recv_frame(req, &frame, 0);
    if(result != ESP_OK || frame.type != HTTPD_WS_TYPE_BINARY ||
       frame.len != REMOTE_INPUT_FRAME_SIZE)
    {
        send_remote_status(req, REMOTE_STATUS_INVALID_FRAME, 0);
        clear_remote_session(socket);
        return ESP_FAIL;
    }

    uint8_t payload[REMOTE_INPUT_FRAME_SIZE];
    frame.payload = payload;
    result = httpd_ws_recv_frame(req, &frame, sizeof(payload));
    if(result != ESP_OK || frame.type != HTTPD_WS_TYPE_BINARY)
    {
        clear_remote_session(socket);
        return ESP_FAIL;
    }

    uint32_t sequence = read_u32_le(payload + 2);
    uint16_t held_buttons = read_u16_le(payload + 6);
    uint16_t pressed_buttons = read_u16_le(payload + 8);
    bool valid = payload[0] == REMOTE_PROTOCOL_VERSION &&
        payload[1] == REMOTE_INPUT_TYPE &&
        (pressed_buttons & (uint16_t)~held_buttons) == 0;

    int16_t axis_values[6]{};
    for(uint8_t i = 0; i < 6 && valid; i++)
    {
        axis_values[i] = (int16_t)read_u16_le(payload + 10 + i * 2);
        valid = axis_values[i] >= REMOTE_AXIS_MIN && axis_values[i] <= REMOTE_AXIS_MAX;
    }
    if(!valid)
    {
        send_remote_status(req, REMOTE_STATUS_INVALID_FRAME, sequence);
        clear_remote_session(socket);
        return ESP_FAIL;
    }

    uint32_t now_ms = millis();
    bool accepted = false;
    bool send_ack = false;
    portENTER_CRITICAL(&remote_lock);
    bool sequence_valid = !remote_sequence_valid ||
        (int32_t)(sequence - remote_sequence) > 0;
    if(remote_socket == socket && sequence_valid)
    {
        remote_state.timestamp_us = (uint32_t)esp_timer_get_time();
        remote_state.buttons = held_buttons;
        for(uint8_t i = 0; i < REMOTE_BUTTON_COUNT; i++)
        {
            if(pressed_buttons & (uint16_t)(1U << i))
            {
                remote_state.press_count[i]++;
            }
        }
        for(uint8_t i = 0; i < 6; i++)
        {
            remote_state.axes[i] = (float)axis_values[i] * 1.0e-3f;
        }
        remote_timestamp_ms = now_ms;
        remote_sequence = sequence;
        remote_sequence_valid = true;
        remote_state.sequence = sequence;
        remote_state.valid = true;
        if(remote_latest.set(remote_state))
        {
            accepted = true;
            send_ack = pressed_buttons != 0 ||
                (uint32_t)(now_ms - remote_ack_timestamp_ms) >= REMOTE_ACK_INTERVAL_MS;
            if(send_ack){remote_ack_timestamp_ms = now_ms;}
        }
    }
    portEXIT_CRITICAL(&remote_lock);

    if(!accepted)
    {
        send_remote_status(req, REMOTE_STATUS_INVALID_FRAME, sequence);
        clear_remote_session(socket);
        return ESP_FAIL;
    }

    return send_ack ? send_remote_status(req, REMOTE_STATUS_OK, sequence) : ESP_OK;
}

/**
 * @brief 在原生 HTTP 会话关闭时释放网页遥控状态
 *
 * @param handle HTTP 服务句柄
 * @param socket 已关闭会话套接字
 */
static void handle_session_close(httpd_handle_t handle, int socket)
{
    clear_remote_session(socket);
}

/**
 * @brief 注册一个原生 HTTP 路由
 *
 * @param path 请求路径
 * @param method HTTP 方法
 * @param handler 请求处理函数
 * @param websocket 是否为 WebSocket 端点
 * @param subprotocol WebSocket 子协议
 *
 * @return 注册成功时返回 true
 */
static bool register_route(const char *path, httpd_method_t method,
    esp_err_t (*handler)(httpd_req_t *), bool websocket = false,
    const char *subprotocol = nullptr)
{
    httpd_uri_t route{};
    route.uri = path;
    route.method = method;
    route.handler = handler;
    route.user_ctx = nullptr;
    route.is_websocket = websocket;
    route.handle_ws_control_frames = false;
    route.supported_subprotocol = subprotocol;
    return httpd_register_uri_handler(server, &route) == ESP_OK;
}

/**
 * @brief 注册全部 HTTP 和 WebSocket 路由
 *
 * @return 全部注册成功时返回 true
 */
static bool register_routes()
{
    return register_route("/", HTTP_GET, handle_root) &&
           register_route("/wifi", HTTP_GET, handle_wifi_root) &&
           register_route("/bluetooth", HTTP_GET, handle_bluetooth_root) &&
           register_route("/servo/middle", HTTP_GET, handle_servo_calibration_root) &&
           register_route("/remote", HTTP_GET, handle_remote_root) &&
           register_route("/api/wifi/scan", HTTP_GET, handle_wifi_scan) &&
           register_route("/api/wifi/connect", HTTP_POST, handle_connect) &&
           register_route("/api/xbox/status", HTTP_GET, handle_xbox_status) &&
           register_route("/api/ble/scan", HTTP_GET, handle_ble_scan) &&
           register_route("/api/xbox/select", HTTP_POST, handle_xbox_select) &&
           register_route("/api/servo/middle-calibration", HTTP_POST,
               handle_servo_middle_calibration) &&
           register_route("/api/servo/middle-calibration/status", HTTP_GET,
               handle_servo_middle_calibration_status) &&
           register_route("/ws/remote", HTTP_GET, handle_remote_websocket, true,
               "shibo-remote-v1");
}

/* ---- web 公共 API ---- */

/**
 * @brief 读取网页遥控最新输入快照
 *
 * @param out 网页遥控输入输出
 *
 * @return 队列存在且已有快照时返回 true
 */
bool io::web::latest_input(control::remote_input &out)
{
    return remote_latest.get(out);
}

/**
 * @brief 初始化原生 HTTP 和 WebSocket 服务
 *
 * @return 服务及全部路由启动成功时返回 true
 */
bool io::web::init()
{
    if(server){return true;}

    if(!remote_latest.init()){return false;}
    clear_remote_session(-1);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.task_priority = 2;
    config.stack_size = 6144;
    config.core_id = 0;
    config.max_open_sockets = 4;
    config.max_uri_handlers = 16;
    config.lru_purge_enable = false;
    config.close_fn = handle_session_close;

    if(httpd_start(&server, &config) != ESP_OK){return false;}
    if(register_routes()){return true;}

    httpd_stop(server);
    server = nullptr;
    clear_remote_session(-1);
    return false;
}

/* ---- RTOS 任务入口 ---- */

/**
 * @brief Web 服务维护任务入口
 *
 * @param arg RTOS 任务参数
 */
void io::web::update()
{
    update_remote_session();
}
