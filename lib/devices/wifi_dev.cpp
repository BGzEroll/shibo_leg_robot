#include "wifi_dev.h"

#include <Preferences.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_wifi.h>

/* ---- WiFi 配置与运行状态 ---- */

static constexpr const char *NVS_NAMESPACE = "wifi";
static constexpr const char *NVS_SSID_KEY = "ssid";
static constexpr const char *NVS_PASS_KEY = "pass";
static constexpr const char *AP_SSID = "SHIBO_LEG_ROBOT";
static constexpr const char *AP_PASS = "12345678";
static constexpr uint32_t START_CONNECT_TIMEOUT_MS = 8000;
static constexpr uint32_t PORTAL_CONNECT_TIMEOUT_MS = 12000;
static constexpr uint32_t STA_ONLY_DELAY_MS = 2000;
static constexpr uint32_t SLEEP_RESTORE_DELAY_MS = 3000;
static constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;

static portMUX_TYPE low_latency_lock = portMUX_INITIALIZER_UNLOCKED;
static bool portal_active = false;
static bool pending_sta_only = false;
static bool low_latency_requested = false;
static uint32_t sta_only_at_ms = 0;
static uint32_t sleep_restore_at_ms = 0;
static uint32_t low_latency_request_sequence = 0;

/* ---- WiFi 内部流程 ---- */

/**
 * @brief 应用不涉及省电模式切换的 WiFi 射频配置
 */
static void apply_radio_settings()
{
    WiFi.setTxPower(WIFI_TX_POWER);
}

/**
 * @brief 清除已经完成的 WiFi Sleep 恢复请求
 *
 * @param sequence 已应用的请求序号
 */
static void finish_sleep_restore(uint32_t sequence)
{
    portENTER_CRITICAL(&low_latency_lock);
    if(low_latency_request_sequence == sequence && !low_latency_requested)
    {
        sleep_restore_at_ms = 0;
    }
    portEXIT_CRITICAL(&low_latency_lock);
}

/**
 * @brief 获取符合 WiFi 与蓝牙共存约束的省电模式
 *
 * @param low_latency_requested 是否请求关闭 WiFi 省电
 *
 * @return 当前允许设置的 WiFi 省电模式
 */
static wifi_ps_type_t select_wifi_sleep_type(bool low_latency_requested)
{
    bool bluetooth_enabled =
        esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED;
    if(low_latency_requested && !bluetooth_enabled){return WIFI_PS_NONE;}
    return WIFI_PS_MIN_MODEM;
}

/**
 * @brief 在 WiFi 维护任务中应用最新低延迟请求
 */
static void apply_low_latency_request()
{
    bool requested;
    uint32_t restore_at_ms;
    uint32_t sequence;
    portENTER_CRITICAL(&low_latency_lock);
    requested = low_latency_requested;
    restore_at_ms = sleep_restore_at_ms;
    sequence = low_latency_request_sequence;
    portEXIT_CRITICAL(&low_latency_lock);

    uint32_t now_ms = millis();
    if(!requested &&
       (!restore_at_ms || (int32_t)(now_ms - restore_at_ms) < 0))
    {
        return;
    }

    wifi_ps_type_t target = select_wifi_sleep_type(requested);
    wifi_ps_type_t current;
    if(esp_wifi_get_ps(&current) != ESP_OK){return;}
    if(current == target)
    {
        if(!requested){finish_sleep_restore(sequence);}
        return;
    }

    bool request_current;
    portENTER_CRITICAL(&low_latency_lock);
    request_current = low_latency_request_sequence == sequence &&
        low_latency_requested == requested;
    portEXIT_CRITICAL(&low_latency_lock);
    if(!request_current){return;}

    if(esp_wifi_set_ps(target) == ESP_OK && !requested)
    {
        finish_sleep_restore(sequence);
    }
}

/**
 * @brief 从 NVS 读取保存的 WiFi 凭据
 *
 * @param ssid SSID 输出
 * @param password 密码输出
 *
 * @return 读到有效 SSID 时返回 true
 */
static bool load_credentials(String &ssid, String &password)
{
    Preferences prefs;
    if(!prefs.begin(NVS_NAMESPACE, true)){return false;}
    ssid = prefs.getString(NVS_SSID_KEY, "");
    password = prefs.getString(NVS_PASS_KEY, "");
    prefs.end();
    return ssid.length() > 0;
}

/**
 * @brief 将 WiFi 凭据保存到 NVS
 *
 * @param ssid SSID
 * @param password 密码
 */
static void save_credentials(const String &ssid, const String &password)
{
    Preferences prefs;
    if(!prefs.begin(NVS_NAMESPACE, false)){return;}
    prefs.putString(NVS_SSID_KEY, ssid);
    prefs.putString(NVS_PASS_KEY, password);
    prefs.end();
}

/**
 * @brief 等待 STA 连接完成或超时
 *
 * @param timeout_ms 超时时间，单位毫秒
 *
 * @return 连接成功时返回 true
 */
static bool wait_station_connected(uint32_t timeout_ms)
{
    uint32_t start_ms = millis();
    while((uint32_t)(millis() - start_ms) < timeout_ms)
    {
        if(WiFi.status() == WL_CONNECTED){return true;}
        delay(100);
    }
    return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief 按指定模式尝试连接 STA
 *
 * @param ssid SSID
 * @param password 密码
 * @param mode WiFi 工作模式
 * @param timeout_ms 超时时间，单位毫秒
 *
 * @return 连接成功时返回 true
 */
static bool connect_station(const String &ssid, const String &password, wifi_mode_t mode, uint32_t timeout_ms)
{
    WiFi.mode(mode);
    apply_radio_settings();
    WiFi.begin(ssid.c_str(), password.c_str());
    return wait_station_connected(timeout_ms);
}

/**
 * @brief 启动 AP_STA 配置模式
 */
static void start_config_portal()
{
    WiFi.mode(WIFI_AP_STA);
    apply_radio_settings();
    WiFi.softAP(AP_SSID, AP_PASS);
    portal_active = true;
}

/**
 * @brief 切换到纯 STA 模式并关闭配置热点
 */
static void switch_to_station_only()
{
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apply_radio_settings();
    portal_active = false;
    pending_sta_only = false;
}

/* ---- wifi_dev 公共 API ---- */

/**
 * @brief 查询 STA 是否已经连接
 *
 * @return 已连接时返回 true
 */
bool wifi_dev::station_connected()
{
    return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief 查询 WiFi 配置门户是否处于激活状态
 *
 * @return 配置门户激活时返回 true
 */
bool wifi_dev::config_portal_active()
{
    return portal_active;
}

/**
 * @brief 连接并保存新的 WiFi 凭据
 *
 * @param ssid SSID
 * @param password 密码
 * @param ip 连接成功后的 STA IP 输出
 *
 * @return 连接成功时返回 true
 */
bool wifi_dev::connect_and_save(const String &ssid, const String &password, IPAddress &ip)
{
    if(!ssid.length()){return false;}

    bool ok = connect_station(ssid, password, WIFI_AP_STA, PORTAL_CONNECT_TIMEOUT_MS);
    if(!ok){return false;}

    save_credentials(ssid, password);
    ip = WiFi.localIP();
    pending_sta_only = true;
    sta_only_at_ms = millis() + STA_ONLY_DELAY_MS;
    return true;
}

/**
 * @brief 获取 STA 本地 IP
 *
 * @return STA 本地 IP 地址
 */
IPAddress wifi_dev::station_ip()
{
    return WiFi.localIP();
}

/**
 * @brief 提交网页遥控使用的低延迟 WiFi 模式请求
 *
 * @param enabled 是否关闭 WiFi 休眠以降低遥控延迟
 */
void wifi_dev::set_low_latency_mode(bool enabled)
{
    uint32_t restore_at_ms = enabled ? 0 : millis() + SLEEP_RESTORE_DELAY_MS;

    portENTER_CRITICAL(&low_latency_lock);
    if(low_latency_requested != enabled)
    {
        low_latency_requested = enabled;
        sleep_restore_at_ms = restore_at_ms;
        low_latency_request_sequence++;
    }
    portEXIT_CRITICAL(&low_latency_lock);
}

/**
 * @brief 执行一次 WiFi 设备状态维护
 */
void wifi_dev::update()
{
    if(pending_sta_only && (int32_t)(millis() - sta_only_at_ms) >= 0)
    {
        switch_to_station_only();
    }
    apply_low_latency_request();
}

/**
 * @brief 初始化 WiFi 模块并按 NVS 凭据决定工作模式
 */
void wifi_dev::init()
{
    portENTER_CRITICAL(&low_latency_lock);
    low_latency_requested = false;
    sleep_restore_at_ms = 0;
    low_latency_request_sequence = 0;
    portEXIT_CRITICAL(&low_latency_lock);

    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    apply_radio_settings();

    String ssid;
    String password;
    if(load_credentials(ssid, password) &&
       connect_station(ssid, password, WIFI_STA, START_CONNECT_TIMEOUT_MS))
    {
        portal_active = false;
        return;
    }

    start_config_portal();
}
