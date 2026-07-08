#include "xbox_dev.h"

#include "xbox.h"
#include "esp_timer.h"
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <Preferences.h>
#include <string.h>

/* ---- BLE 配置与运行状态 ---- */

static constexpr const char *NVS_NAMESPACE = "xbox";
static constexpr const char *NVS_TARGET_KEY = "target";
static constexpr uint16_t XBOX_APPEARANCE = 964;
static constexpr uint32_t SCAN_PAUSE_MS = 60;
static constexpr const char *HID_SERVICE_UUID = "1812";
static constexpr const char *XBOX_MANUFACTURER_NORMAL = "060000";
static constexpr const char *XBOX_MANUFACTURER_SEARCHING = "0600030080";

static xbox *gamepad = nullptr;
static QueueHandle_t xbox_data_queue = nullptr;
static volatile bool ble_scan_active = false;
static String current_target_address;

class ble_scan_callbacks : public NimBLEScanCallbacks
{
};

static ble_scan_callbacks scan_callbacks;

/* ---- BLE 连接内部流程 ---- */

/**
 * @brief 清空 Xbox 输入队列状态
 */
static void clear_input_state()
{
    if(!xbox_data_queue){return;}

    xbox_dev::data state = {};
    state.timestamp_us = (uint32_t)esp_timer_get_time();
    xQueueOverwrite(xbox_data_queue, &state);
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
    for(NimBLEClient *client : clients)
    {
        NimBLEDevice::deleteClient(client);
    }

    uint32_t start_ms = millis();
    while(NimBLEDevice::getConnectedClients().size() && (uint32_t)(millis() - start_ms) < 800)
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
    ble_scan_active = true;
    delay(SCAN_PAUSE_MS);

    stop_ble_activity();

    xbox *old_gamepad = gamepad;
    gamepad = nullptr;
    if(old_gamepad){delete old_gamepad;}

    clear_input_state();
    gamepad = new xbox(current_target_address.c_str());
    gamepad->init();

    ble_scan_active = false;
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
    String out = address;
    out.trim();
    out.toLowerCase();
    return out;
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
    if(!device){return false;}
    if(device->getAppearance() != XBOX_APPEARANCE){return false;}
    if(!device->haveServiceUUID() || !device->getServiceUUID().equals(NimBLEUUID(HID_SERVICE_UUID))){return false;}
    if(!device->haveManufacturerData()){return false;}

    std::string manufacturer = device->getManufacturerData();
    std::string hex = NimBLEUtils::dataToHexString((uint8_t *)manufacturer.data(), manufacturer.length());
    return hex == XBOX_MANUFACTURER_NORMAL || hex == XBOX_MANUFACTURER_SEARCHING;
}

/* ---- xbox_dev 公共 API ---- */

/**
 * @brief 获取 Xbox 输入数据队列
 *
 * @return 队列句柄
 */
QueueHandle_t xbox_dev::queue()
{
    return xbox_data_queue;
}

/**
 * @brief 查询 Xbox 手柄是否已经连接
 *
 * @return 已连接时返回 true
 */
bool xbox_dev::connected()
{
    return gamepad && gamepad->get_connection_state();
}

/**
 * @brief 获取当前目标手柄蓝牙地址
 *
 * @return 目标手柄地址，空字符串表示自动发现
 */
String xbox_dev::target_address()
{
    return current_target_address;
}

/**
 * @brief 扫描周围 BLE 设备
 *
 * @param devices 扫描结果输出缓冲区
 * @param max_count 输出缓冲区容量
 * @param count 实际扫描到的数量
 * @param duration_ms 扫描持续时间，单位毫秒
 *
 * @return 成功启动并完成扫描时返回 true
 */
bool xbox_dev::scan_ble(ble_device *devices, uint8_t max_count, uint8_t &count, uint32_t duration_ms)
{
    count = 0;
    if(!devices || !max_count){return false;}

    ble_scan_active = true;
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

        ble_device &out = devices[count];
        out.address = String(device->getAddress().toString().c_str());
        out.name = device->haveName() ? String(device->getName().c_str()) : String();
        out.rssi = device->getRSSI();
        out.xbox = is_xbox_device(device);
        out.connectable = device->isConnectable();
        count++;
    }

    scan->clearResults();
    ble_scan_active = false;
    return true;
}

/**
 * @brief 设置并保存目标手柄蓝牙地址
 *
 * @param address 目标蓝牙地址
 *
 * @return 地址格式有效时返回 true
 */
bool xbox_dev::set_target_address(const String &address)
{
    String normalized = normalize_address(address);
    if(normalized.length() != 17){return false;}

    current_target_address = normalized;
    save_target_address(current_target_address);
    rebuild_gamepad();
    return true;
}

/**
 * @brief 初始化 Xbox 设备模块
 */
void xbox_dev::init()
{
    current_target_address = load_target_address();
    xbox_data_queue = xQueueCreate(1, sizeof(xbox_dev::data));
    rebuild_gamepad();
}

/* ---- RTOS 任务入口 ---- */

/**
 * @brief Xbox 输入采样任务入口
 *
 * @param arg RTOS 任务参数
 */
void xbox_dev::task_entry(void *arg)
{
    while(true)
    {
        if(ble_scan_active || !gamepad)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        gamepad->update();

        xbox_dev::data state;
        state.timestamp_us = (uint32_t)esp_timer_get_time();
        state.buttons = gamepad->buttons;
        memcpy(state.axes, gamepad->axes, sizeof(gamepad->axes));

        if(xbox_data_queue)
        {
            xQueueOverwrite(xbox_data_queue, &state);
        }

        delay(20);
    }
}
