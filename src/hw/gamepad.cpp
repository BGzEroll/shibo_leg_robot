#include "gamepad.h"

#include "NimBLEDevice.h"
#include "NimBLEUtils.h"
#include "Preferences.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "string.h"
#include "util/latest.h"
#include "xbox.h"

/* ---- BLE 配置与运行状态 ---- */

static constexpr const char *NVS_NAMESPACE = "xbox";
static constexpr const char *NVS_TARGET_KEY = "target";
static constexpr uint16_t XBOX_APPEARANCE = 964;
static constexpr uint8_t INPUT_BUTTON_COUNT = 16;
static constexpr uint32_t SCAN_PAUSE_MS = 60;
static constexpr const char *HID_SERVICE_UUID = "1812";
static constexpr const char *XBOX_MANUFACTURER_NORMAL = "060000";
static constexpr const char *XBOX_MANUFACTURER_SEARCHING = "0600030080";

static xbox *gamepad_driver = nullptr;
static util::latest<control::remote_input> input_latest;
static util::latest<bool> connection_latest;
static control::remote_input input_state;
static uint32_t gamepad_stream_id = 0;
static bool input_connected = false;
static String current_target_address;
static SemaphoreHandle_t gamepad_mutex = nullptr;

class ble_scan_callbacks : public NimBLEScanCallbacks
{
};

static ble_scan_callbacks scan_callbacks;

/* ---- BLE 连接内部流程 ---- */

/**
 * @brief 清空 Xbox 输入快照并切换输入流代次
 */
static void clear_input_state()
{
    input_connected = false;
    connection_latest.set(false);
    input_state = control::remote_input{};
    input_state.stream_id = ++gamepad_stream_id;
    input_latest.set(input_state);
}

/**
 * @brief 按当前手柄状态发布统一遥控输入快照
 */
static void publish_input_state()
{
    bool connected = gamepad_driver && gamepad_driver->get_connection_state();
    if(connected != input_connected)
    {
        input_connected = connected;
        connection_latest.set(input_connected);
        input_state = control::remote_input{};
        input_state.stream_id = ++gamepad_stream_id;
    }

    if(!connected)
    {
        input_latest.set(input_state);
        return;
    }

    uint16_t buttons = gamepad_driver->buttons;
    uint16_t pressed_buttons = buttons & (uint16_t)~input_state.buttons;
    for(uint8_t i = 0; i < INPUT_BUTTON_COUNT; i++)
    {
        if(pressed_buttons & (uint16_t)(1U << i)){input_state.press_count[i]++;}
    }

    input_state.sequence++;
    input_state.timestamp_us = (uint32_t)esp_timer_get_time();
    input_state.buttons = buttons;
    memcpy(input_state.axes, gamepad_driver->axes, sizeof(input_state.axes));
    input_state.valid = true;
    input_latest.set(input_state);
}

/**
 * @brief 停止当前 BLE 扫描和连接
 */
static void stop_ble_activity()
{
    NimBLEScan *scan = NimBLEDevice::getScan();
    if(scan->isScanning()){scan->stop();}
    scan->clearResults();

    std::vector<NimBLEClient *> clients = NimBLEDevice::getConnectedClients();
    for(NimBLEClient *client : clients){NimBLEDevice::deleteClient(client);}

    uint32_t start_ms = millis();
    while(NimBLEDevice::getConnectedClients().size() &&
          (uint32_t)(millis() - start_ms) < 800)
    {
        delay(20);
    }

    for(uint8_t i = 0; i < 8; i++)
    {
        NimBLEClient *client = NimBLEDevice::getDisconnectedClient();
        if(!client){break;}
        NimBLEDevice::deleteClient(client);
    }
}

/**
 * @brief 按当前目标地址重建 Xbox 连接对象
 */
static void rebuild_gamepad()
{
    delay(SCAN_PAUSE_MS);
    stop_ble_activity();

    xbox *old_gamepad = gamepad_driver;
    gamepad_driver = nullptr;
    if(old_gamepad){delete old_gamepad;}

    clear_input_state();
    gamepad_driver = new xbox(current_target_address.c_str());
    gamepad_driver->init();
}

/**
 * @brief 获取 gamepad 资源互斥锁
 *
 * @param timeout 等待超时时间
 *
 * @return 成功获取锁时返回 true
 */
static bool lock_gamepad(TickType_t timeout)
{
    return gamepad_mutex && xSemaphoreTake(gamepad_mutex, timeout) == pdTRUE;
}

/**
 * @brief 释放 gamepad 资源互斥锁
 */
static void unlock_gamepad()
{
    if(gamepad_mutex){xSemaphoreGive(gamepad_mutex);}
}

/**
 * @brief 对保存的蓝牙地址进行基础整理
 *
 * @param address 原始地址
 *
 * @return 整理后的地址
 */
static String normalize_address(const String &address)
{
    String normalized = address;
    normalized.trim();
    normalized.toLowerCase();
    return normalized;
}

/**
 * @brief 从 NVS 读取目标手柄地址
 *
 * @return 保存的手柄地址
 */
static String load_target_address()
{
    Preferences prefs;
    if(!prefs.begin(NVS_NAMESPACE, true)){return String();}
    String address = prefs.getString(NVS_TARGET_KEY, "");
    prefs.end();
    return normalize_address(address);
}

/**
 * @brief 保存目标手柄地址到 NVS
 *
 * @param address 目标蓝牙地址
 */
static void save_target_address(const String &address)
{
    Preferences prefs;
    if(!prefs.begin(NVS_NAMESPACE, false)){return;}
    prefs.putString(NVS_TARGET_KEY, normalize_address(address));
    prefs.end();
}

/**
 * @brief 判断扫描结果是否像 Xbox 手柄
 *
 * @param device 蓝牙广播设备
 *
 * @return 像 Xbox 手柄时返回 true
 */
static bool is_xbox_device(const NimBLEAdvertisedDevice *device)
{
    if(!device || device->getAppearance() != XBOX_APPEARANCE){return false;}
    if(!device->haveServiceUUID() ||
       !device->getServiceUUID().equals(NimBLEUUID(HID_SERVICE_UUID))){return false;}
    if(!device->haveManufacturerData()){return false;}

    std::string manufacturer = device->getManufacturerData();
    std::string hex = NimBLEUtils::dataToHexString(
        (uint8_t *)manufacturer.data(), manufacturer.length());
    return hex == XBOX_MANUFACTURER_NORMAL || hex == XBOX_MANUFACTURER_SEARCHING;
}

/* ---- gamepad 公共 API ---- */

/**
 * @brief 读取 Xbox 最新遥控输入
 *
 * @param out 输入快照输出
 *
 * @return 已有输入快照时返回 true
 */
bool hw::gamepad::latest_input(control::remote_input &out)
{
    return input_latest.get(out);
}

/**
 * @brief 查询 Xbox 手柄是否已经连接
 *
 * @return 已连接时返回 true
 */
bool hw::gamepad::connected()
{
    bool connected = false;
    connection_latest.get(connected);
    return connected;
}

/**
 * @brief 获取当前目标手柄蓝牙地址
 *
 * @return 目标手柄地址，空字符串表示自动发现
 */
String hw::gamepad::target_address()
{
    if(!lock_gamepad(pdMS_TO_TICKS(50))){return String();}
    String address = current_target_address;
    unlock_gamepad();
    return address;
}

/**
 * @brief 扫描周围 BLE 设备
 *
 * @param devices 扫描结果输出缓冲区
 * @param max_count 输出缓冲区容量
 * @param count 实际扫描到的数量
 * @param duration_ms 扫描持续时间，单位毫秒
 *
 * @return 扫描完成时返回 true
 */
bool hw::gamepad::scan_ble(hw::gamepad::ble_device *devices, uint8_t max_count,
    uint8_t &count, uint32_t duration_ms)
{
    count = 0;
    if(!devices || !max_count){return false;}

    if(!lock_gamepad(portMAX_DELAY)){return false;}
    delay(SCAN_PAUSE_MS);
    NimBLEScan *scan = NimBLEDevice::getScan();
    if(scan->isScanning()){scan->stop();}
    scan->setScanCallbacks(&scan_callbacks, false);
    scan->setActiveScan(true);
    scan->setDuplicateFilter(false);
    scan->setInterval(97);
    scan->setWindow(97);
    scan->setMaxResults(max_count);

    NimBLEScanResults results = scan->getResults(duration_ms, false);
    int32_t result_count = results.getCount();
    for(int32_t i = 0; i < result_count && count < max_count; i++)
    {
        const NimBLEAdvertisedDevice *device = results.getDevice((uint32_t)i);
        if(!device){continue;}

        hw::gamepad::ble_device &output = devices[count];
        output.address = String(device->getAddress().toString().c_str());
        output.name = device->haveName() ? String(device->getName().c_str()) : String();
        output.rssi = device->getRSSI();
        output.xbox = is_xbox_device(device);
        output.connectable = device->isConnectable();
        count++;
    }

    scan->clearResults();
    unlock_gamepad();
    return true;
}

/**
 * @brief 设置并保存目标手柄蓝牙地址
 *
 * @param address 目标蓝牙地址
 *
 * @return 地址长度有效时返回 true
 */
bool hw::gamepad::set_target_address(const String &address)
{
    String normalized = normalize_address(address);
    if(normalized.length() != 17){return false;}
    if(!lock_gamepad(portMAX_DELAY)){return false;}
    current_target_address = normalized;
    save_target_address(current_target_address);
    rebuild_gamepad();
    unlock_gamepad();
    return true;
}

/**
 * @brief 初始化 Xbox 输入快照和 BLE 连接
 */
void hw::gamepad::init()
{
    input_latest.init();
    connection_latest.init();
    gamepad_mutex = xSemaphoreCreateMutex();
    if(!gamepad_mutex){return;}
    current_target_address = load_target_address();
    connection_latest.set(false);
    rebuild_gamepad();
}

/**
 * @brief Xbox BLE 维护和输入采样任务入口
 *
 * @param arg RTOS 任务参数
 */
void hw::gamepad::task_entry(void *arg)
{
    while(true)
    {
        if(!lock_gamepad(pdMS_TO_TICKS(50)))
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if(!gamepad_driver)
        {
            unlock_gamepad();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        gamepad_driver->update();
        publish_input_state();
        unlock_gamepad();
        delay(20);
    }
}
