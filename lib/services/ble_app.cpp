#include "ble_app.h"

#include "controller.h"
#include "wifi_dev.h"
#include "xbox_dev.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <string.h>

static constexpr const char *BLE_DEVICE_NAME = "SHIBO_LEG_ROBOT";
static constexpr const char *BLE_SERVICE_UUID = "2f7a0000-37e2-4b7f-9f2a-5ef0c2c9a001";
static constexpr const char *BLE_COMMAND_UUID = "2f7a0001-37e2-4b7f-9f2a-5ef0c2c9a001";
static constexpr const char *BLE_EVENT_UUID = "2f7a0002-37e2-4b7f-9f2a-5ef0c2c9a001";
static constexpr uint32_t BLE_PAIR_PASSKEY = 123456;
static constexpr uint16_t BLE_ADV_INTERVAL = 800;
static constexpr uint16_t BLE_PREFERRED_MIN_INTERVAL = 24;
static constexpr uint16_t BLE_PREFERRED_MAX_INTERVAL = 80;
static constexpr uint16_t BLE_MTU = 185;
static constexpr uint8_t COMMAND_QUEUE_LEN = 4;
static constexpr uint8_t COMMAND_FRAME_HEADER_LEN = 3;
static constexpr uint8_t EVENT_FRAME_HEADER_LEN = 5;
static constexpr uint8_t COMMAND_PAYLOAD_MAX = 128;
static constexpr uint8_t EVENT_PAYLOAD_MAX = 180;
static constexpr uint8_t WIFI_SCAN_MAX = 24;
static constexpr uint8_t BLE_SCAN_MAX = 24;
static constexpr uint8_t SSID_MAX_LEN = 32;
static constexpr uint8_t WIFI_PASSWORD_MAX_LEN = 64;
static constexpr uint8_t XBOX_ADDRESS_LEN = 17;
static constexpr uint8_t XBOX_NAME_MAX_LEN = 40;
static constexpr uint32_t XBOX_SCAN_MS = 4000;
static constexpr uint32_t EVENT_GAP_MS = 20;
static constexpr uint8_t EVENT_FLAG_MORE = 0x01;

enum class opcode : uint8_t
{
    GET_STATUS = 0x01,
    WIFI_SCAN_START = 0x02,
    WIFI_CONNECT_SAVE = 0x03,
    XBOX_BLE_SCAN_START = 0x04,
    XBOX_SELECT = 0x05,
    SERVO_MIDDLE_CALIBRATION_START = 0x06,
    SERVO_MIDDLE_CALIBRATION_STATUS = 0x07
};

enum class event_status : uint8_t
{
    OK = 0,
    BUSY = 1,
    INVALID = 2,
    TIMEOUT = 3,
    DENIED = 4,
    FAILED = 5
};

struct command_frame
{
    uint8_t opcode = 0;
    uint8_t request_id = 0;
    uint8_t payload_len = 0;
    uint8_t payload[COMMAND_PAYLOAD_MAX]{};
};

static QueueHandle_t command_queue = nullptr;
static NimBLECharacteristic *event_characteristic = nullptr;
static bool ble_connected = false;
static bool service_started = false;

/**
 * @brief 发送 BLE 事件帧
 *
 * @param code 命令码
 * @param request_id 请求序号
 * @param status 处理状态
 * @param flags 事件标志位
 * @param payload 事件载荷
 * @param payload_len 事件载荷长度
 *
 * @return 发送成功时返回 true
 */
static bool send_event(uint8_t code,
    uint8_t request_id,
    event_status status,
    uint8_t flags,
    const uint8_t *payload,
    uint8_t payload_len)
{
    if(!event_characteristic || payload_len > EVENT_PAYLOAD_MAX){return false;}

    uint8_t frame[EVENT_FRAME_HEADER_LEN + EVENT_PAYLOAD_MAX]{};
    frame[0] = code;
    frame[1] = request_id;
    frame[2] = (uint8_t)status;
    frame[3] = flags;
    frame[4] = payload_len;
    if(payload && payload_len)
    {
        memcpy(&frame[EVENT_FRAME_HEADER_LEN], payload, payload_len);
    }

    bool ok = event_characteristic->notify(frame, EVENT_FRAME_HEADER_LEN + payload_len);
    if(ble_connected)
    {
        vTaskDelay(pdMS_TO_TICKS(EVENT_GAP_MS));
    }
    return ok;
}

/**
 * @brief 发送不带载荷的 BLE 事件帧
 *
 * @param code 命令码
 * @param request_id 请求序号
 * @param status 处理状态
 *
 * @return 发送成功时返回 true
 */
static bool send_empty_event(uint8_t code, uint8_t request_id, event_status status)
{
    return send_event(code, request_id, status, 0, nullptr, 0);
}

/**
 * @brief 判断 WiFi 射频是否处于开启模式
 *
 * @return WiFi 开启时返回 true
 */
static bool wifi_enabled()
{
    return WiFi.getMode() != WIFI_OFF;
}

/**
 * @brief 把写入的 BLE 命令帧放入工作队列
 *
 * @param data 命令帧数据
 * @param len 命令帧长度
 */
static void enqueue_command(const uint8_t *data, uint16_t len)
{
    if(!data || len < COMMAND_FRAME_HEADER_LEN){return;}

    command_frame command;
    command.opcode = data[0];
    command.request_id = data[1];
    command.payload_len = data[2];

    if(command.payload_len > COMMAND_PAYLOAD_MAX ||
       len != (uint16_t)(COMMAND_FRAME_HEADER_LEN + command.payload_len))
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    if(command.payload_len)
    {
        memcpy(command.payload, &data[COMMAND_FRAME_HEADER_LEN], command.payload_len);
    }

    if(!command_queue || xQueueSend(command_queue, &command, 0) != pdTRUE)
    {
        send_empty_event(command.opcode, command.request_id, event_status::BUSY);
    }
}

// 只负责连接状态和断开后重新广播，慢操作放在 ble_app_task 中执行。
class app_server_callbacks : public NimBLEServerCallbacks
{
    public:
        void onConnect(NimBLEServer *server, NimBLEConnInfo &conn_info) override
        {
            ble_connected = true;
            server->updateConnParams(conn_info.getConnHandle(), 24, 80, 0, 400);
        }

        void onDisconnect(NimBLEServer *server, NimBLEConnInfo &conn_info, int reason) override
        {
            ble_connected = false;
            NimBLEDevice::startAdvertising();
        }
};

// 只解析帧并入队，避免 BLE 回调里执行扫描、连接等慢操作。
class command_callbacks : public NimBLECharacteristicCallbacks
{
    public:
        void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override
        {
            NimBLEAttValue value = characteristic->getValue();
            enqueue_command(value.data(), value.length());
        }
};

static app_server_callbacks server_callbacks;
static command_callbacks command_write_callbacks;

/**
 * @brief 处理状态查询命令
 *
 * @param command 命令帧
 */
static void handle_get_status(const command_frame &command)
{
    if(command.payload_len)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    uint8_t payload[22]{};
    String target = xbox_dev::target_address();
    uint8_t target_len = (uint8_t)min((uint32_t)target.length(), (uint32_t)XBOX_ADDRESS_LEN);
    payload[0] = xbox_dev::connected() ? 1 : 0;
    payload[1] = controller::middle_calibration_success() ? 1 : 0;
    payload[2] = wifi_enabled() ? 1 : 0;
    payload[3] = target_len;
    if(target_len)
    {
        memcpy(&payload[4], target.c_str(), target_len);
    }

    send_event(command.opcode, command.request_id, event_status::OK, 0, payload, 4 + target_len);
}

/**
 * @brief 处理 WiFi 扫描命令
 *
 * @param command 命令帧
 */
static void handle_wifi_scan(const command_frame &command)
{
    if(command.payload_len)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    wifi_dev::network networks[WIFI_SCAN_MAX];
    uint8_t count = 0;
    bool ok = wifi_dev::scan_networks(networks, WIFI_SCAN_MAX, count);
    if(!ok)
    {
        wifi_dev::shutdown();
        send_empty_event(command.opcode, command.request_id, event_status::FAILED);
        return;
    }

    for(uint8_t i = 0; i < count; i++)
    {
        uint8_t payload[3 + SSID_MAX_LEN]{};
        uint8_t ssid_len = (uint8_t)strnlen(networks[i].ssid, SSID_MAX_LEN);
        payload[0] = (uint8_t)networks[i].rssi;
        payload[1] = networks[i].secure ? 1 : 0;
        payload[2] = ssid_len;
        if(ssid_len)
        {
            memcpy(&payload[3], networks[i].ssid, ssid_len);
        }
        send_event(command.opcode, command.request_id, event_status::OK, EVENT_FLAG_MORE, payload, 3 + ssid_len);
    }

    wifi_dev::shutdown();
    send_empty_event(command.opcode, command.request_id, event_status::OK);
}

/**
 * @brief 处理 WiFi 连接和保存命令
 *
 * @param command 命令帧
 */
static void handle_wifi_connect_save(const command_frame &command)
{
    if(command.payload_len < 2)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    uint8_t ssid_len = command.payload[0];
    uint8_t password_len = command.payload[1];
    if(!ssid_len ||
       ssid_len > SSID_MAX_LEN ||
       password_len > WIFI_PASSWORD_MAX_LEN ||
       command.payload_len != (uint8_t)(2 + ssid_len + password_len))
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    char ssid[SSID_MAX_LEN + 1]{};
    char password[WIFI_PASSWORD_MAX_LEN + 1]{};
    memcpy(ssid, &command.payload[2], ssid_len);
    if(password_len)
    {
        memcpy(password, &command.payload[2 + ssid_len], password_len);
    }

    IPAddress ip;
    bool ok = wifi_dev::connect_and_save_station(ssid, password, ip);
    if(!ok)
    {
        wifi_dev::shutdown();
        send_empty_event(command.opcode, command.request_id, event_status::TIMEOUT);
        return;
    }

    char ip_text[16]{};
    snprintf(ip_text, sizeof(ip_text), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    uint8_t ip_len = (uint8_t)strnlen(ip_text, sizeof(ip_text));
    uint8_t payload[1 + sizeof(ip_text)]{};
    payload[0] = ip_len;
    memcpy(&payload[1], ip_text, ip_len);

    wifi_dev::shutdown();
    send_event(command.opcode, command.request_id, event_status::OK, 0, payload, 1 + ip_len);
}

/**
 * @brief 处理 Xbox 周围 BLE 设备扫描命令
 *
 * @param command 命令帧
 */
static void handle_xbox_ble_scan(const command_frame &command)
{
    if(command.payload_len)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    xbox_dev::ble_device devices[BLE_SCAN_MAX];
    uint8_t count = 0;
    bool ok = xbox_dev::scan_ble(devices, BLE_SCAN_MAX, count, XBOX_SCAN_MS);
    if(!ok)
    {
        send_empty_event(command.opcode, command.request_id, event_status::FAILED);
        return;
    }

    for(uint8_t i = 0; i < count; i++)
    {
        uint8_t payload[3 + XBOX_ADDRESS_LEN + 1 + XBOX_NAME_MAX_LEN]{};
        uint8_t address_len = (uint8_t)min((uint32_t)devices[i].address.length(), (uint32_t)XBOX_ADDRESS_LEN);
        uint8_t name_len = (uint8_t)min((uint32_t)devices[i].name.length(), (uint32_t)XBOX_NAME_MAX_LEN);
        payload[0] = (uint8_t)devices[i].rssi;
        payload[1] = devices[i].xbox ? 1 : 0;
        payload[2] = devices[i].connectable ? 1 : 0;
        if(address_len)
        {
            memcpy(&payload[3], devices[i].address.c_str(), address_len);
        }
        payload[3 + XBOX_ADDRESS_LEN] = name_len;
        if(name_len)
        {
            memcpy(&payload[3 + XBOX_ADDRESS_LEN + 1], devices[i].name.c_str(), name_len);
        }

        send_event(command.opcode,
            command.request_id,
            event_status::OK,
            EVENT_FLAG_MORE,
            payload,
            3 + XBOX_ADDRESS_LEN + 1 + name_len);
    }

    send_empty_event(command.opcode, command.request_id, event_status::OK);
}

/**
 * @brief 处理 Xbox 目标地址选择命令
 *
 * @param command 命令帧
 */
static void handle_xbox_select(const command_frame &command)
{
    if(command.payload_len != XBOX_ADDRESS_LEN)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    char address[XBOX_ADDRESS_LEN + 1]{};
    memcpy(address, command.payload, XBOX_ADDRESS_LEN);
    bool ok = xbox_dev::set_target_address(String(address));
    send_empty_event(command.opcode, command.request_id, ok ? event_status::OK : event_status::INVALID);
}

/**
 * @brief 处理舵机中位校准启动命令
 *
 * @param command 命令帧
 */
static void handle_servo_middle_calibration_start(const command_frame &command)
{
    if(command.payload_len)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    bool ok = controller::request_middle_calibration();
    send_empty_event(command.opcode, command.request_id, ok ? event_status::OK : event_status::FAILED);
}

/**
 * @brief 处理舵机中位校准状态查询命令
 *
 * @param command 命令帧
 */
static void handle_servo_middle_calibration_status(const command_frame &command)
{
    if(command.payload_len)
    {
        send_empty_event(command.opcode, command.request_id, event_status::INVALID);
        return;
    }

    uint8_t payload[1] = {(uint8_t)(controller::middle_calibration_success() ? 1U : 0U)};
    send_event(command.opcode, command.request_id, event_status::OK, 0, payload, sizeof(payload));
}

/**
 * @brief 分发 BLE 命令帧
 *
 * @param command 命令帧
 */
static void dispatch_command(const command_frame &command)
{
    switch((opcode)command.opcode)
    {
        case opcode::GET_STATUS:
            handle_get_status(command);
            break;

        case opcode::WIFI_SCAN_START:
            handle_wifi_scan(command);
            break;

        case opcode::WIFI_CONNECT_SAVE:
            handle_wifi_connect_save(command);
            break;

        case opcode::XBOX_BLE_SCAN_START:
            handle_xbox_ble_scan(command);
            break;

        case opcode::XBOX_SELECT:
            handle_xbox_select(command);
            break;

        case opcode::SERVO_MIDDLE_CALIBRATION_START:
            handle_servo_middle_calibration_start(command);
            break;

        case opcode::SERVO_MIDDLE_CALIBRATION_STATUS:
            handle_servo_middle_calibration_status(command);
            break;

        default:
            send_empty_event(command.opcode, command.request_id, event_status::INVALID);
            break;
    }
}

/**
 * @brief 初始化 BLE App GATT 服务
 */
void ble_app::init()
{
    if(service_started){return;}

    if(!NimBLEDevice::isInitialized())
    {
        NimBLEDevice::init(BLE_DEVICE_NAME);
    }

    NimBLEDevice::setDeviceName(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(BLE_MTU);
    NimBLEDevice::setSecurityPasskey(BLE_PAIR_PASSKEY);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityAuth(true, false, false);

    command_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(command_frame));

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(&server_callbacks, false);

    NimBLEService *service = server->createService(BLE_SERVICE_UUID);
    NimBLECharacteristic *command_characteristic = service->createCharacteristic(
        BLE_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC,
        COMMAND_FRAME_HEADER_LEN + COMMAND_PAYLOAD_MAX);
    event_characteristic = service->createCharacteristic(
        BLE_EVENT_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY,
        EVENT_FRAME_HEADER_LEN + EVENT_PAYLOAD_MAX);

    command_characteristic->setCallbacks(&command_write_callbacks);
    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->setName(BLE_DEVICE_NAME);
    advertising->addServiceUUID(service->getUUID());
    advertising->enableScanResponse(false);
    advertising->setAdvertisingInterval(BLE_ADV_INTERVAL);
    advertising->setPreferredParams(BLE_PREFERRED_MIN_INTERVAL, BLE_PREFERRED_MAX_INTERVAL);
    advertising->start();

    service_started = true;
}

/**
 * @brief BLE App 命令工作任务入口
 *
 * @param arg RTOS 任务参数
 */
void ble_app::task_entry(void *arg)
{
    command_frame command;
    while(true)
    {
        if(!command_queue)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if(xQueueReceive(command_queue, &command, portMAX_DELAY) == pdTRUE)
        {
            dispatch_command(command);
        }
    }
}
