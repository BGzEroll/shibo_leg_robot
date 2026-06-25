#include "wifi_dev.h"

#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

static constexpr const char *NVS_NAMESPACE = "wifi";
static constexpr const char *NVS_SSID_KEY = "ssid";
static constexpr const char *NVS_PASS_KEY = "pass";
static constexpr const char *AP_SSID = "SHIBO_LEG_ROBOT";
static constexpr const char *AP_PASS = "12345678";
static constexpr uint32_t START_CONNECT_TIMEOUT_MS = 8000;
static constexpr uint32_t PORTAL_CONNECT_TIMEOUT_MS = 12000;
static constexpr uint32_t STA_ONLY_DELAY_MS = 2000;

static WebServer server(80);
static bool config_portal_active = false;
static bool server_started = false;
static bool pending_sta_only = false;
static uint32_t sta_only_at_ms = 0;

/**
 * @brief 对 JSON 字符串内容进行转义
 *
 * @param value 原始字符串
 *
 * @return 转义后的字符串
 */
static String json_escape(const String &value)
{
    String out;
    out.reserve(value.length() + 8);
    for(size_t i = 0; i < value.length(); i++)
    {
        char c = value[i];
        if(c == '"' || c == '\\')
        {
            out += '\\';
            out += c;
        }
        else if(c == '\n')
        {
            out += "\\n";
        }
        else if(c == '\r')
        {
            out += "\\r";
        }
        else
        {
            out += c;
        }
    }
    return out;
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
    WiFi.begin(ssid.c_str(), password.c_str());
    return wait_station_connected(timeout_ms);
}

/**
 * @brief 返回配置页 HTML
 *
 * @return HTML 文本
 */
static String portal_html()
{
    return R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shibo WiFi</title>
<style>
body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#f5f7fb;color:#172033}
main{max-width:520px;margin:0 auto;padding:24px 18px}
h1{font-size:24px;margin:0 0 18px}
button,input{font:inherit}
button{border:0;background:#1d4ed8;color:white;padding:10px 14px;border-radius:6px}
button:disabled{opacity:.55}
.network{display:flex;justify-content:space-between;align-items:center;border:1px solid #d8deea;background:white;border-radius:6px;padding:10px 12px;margin:8px 0}
.network button{background:#334155;padding:7px 10px}
form{display:grid;gap:10px;margin-top:18px;background:white;border:1px solid #d8deea;border-radius:6px;padding:14px}
input{box-sizing:border-box;width:100%;padding:10px;border:1px solid #cbd5e1;border-radius:6px}
#status{margin-top:12px;min-height:24px;color:#475569}
small{color:#64748b}
</style>
</head>
<body>
<main>
<h1>Shibo WiFi</h1>
<button id="scan">扫描周围 WiFi</button>
<div id="list"></div>
<form id="form">
<input id="ssid" name="ssid" placeholder="SSID" required>
<input id="password" name="password" placeholder="密码" type="password">
<button id="connect" type="submit">连接并保存</button>
</form>
<div id="status"></div>
<small>连接成功后设备会切回 Station 模式，配置热点将关闭。</small>
</main>
<script>
const scanBtn=document.getElementById('scan');
const list=document.getElementById('list');
const statusEl=document.getElementById('status');
scanBtn.onclick=async()=>{
  scanBtn.disabled=true; statusEl.textContent='正在扫描...'; list.innerHTML='';
  try{
    const res=await fetch('/scan');
    const aps=await res.json();
    list.innerHTML=aps.map(ap=>`<div class="network"><span>${ap.ssid}<br><small>${ap.rssi} dBm ${ap.secure?'加密':'开放'}</small></span><button data-ssid="${ap.ssid.replace(/"/g,'&quot;')}">选择</button></div>`).join('') || '<p>未发现 WiFi</p>';
    list.querySelectorAll('button').forEach(btn=>btn.onclick=()=>document.getElementById('ssid').value=btn.dataset.ssid);
    statusEl.textContent='扫描完成';
  }catch(e){statusEl.textContent='扫描失败';}
  scanBtn.disabled=false;
};
document.getElementById('form').onsubmit=async(e)=>{
  e.preventDefault(); statusEl.textContent='正在连接...';
  const body=new URLSearchParams(new FormData(e.target));
  const res=await fetch('/connect',{method:'POST',body});
  const data=await res.json();
  statusEl.textContent=data.ok ? `连接成功，IP: ${data.ip}` : `连接失败: ${data.error||'请检查密码'}`;
};
</script>
</body>
</html>)HTML";
}

/**
 * @brief 处理配置页请求
 */
static void handle_root()
{
    server.send(200, "text/html; charset=utf-8", portal_html());
}

/**
 * @brief 处理 WiFi 扫描请求
 */
static void handle_scan()
{
    int count = WiFi.scanNetworks(false, true);
    String json = "[";
    for(int i = 0; i < count; i++)
    {
        if(i){json += ',';}
        json += "{\"ssid\":\"" + json_escape(WiFi.SSID(i)) + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ',';
        json += "\"secure\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true") + '}';
    }
    json += ']';
    WiFi.scanDelete();
    server.send(200, "application/json", json);
}

/**
 * @brief 处理 WiFi 连接和保存请求
 */
static void handle_connect()
{
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    ssid.trim();
    if(!ssid.length())
    {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"SSID 为空\"}");
        return;
    }

    bool ok = connect_station(ssid, password, WIFI_AP_STA, PORTAL_CONNECT_TIMEOUT_MS);
    if(!ok)
    {
        server.send(200, "application/json", "{\"ok\":false,\"error\":\"连接超时\"}");
        return;
    }

    save_credentials(ssid, password);
    pending_sta_only = true;
    sta_only_at_ms = millis() + STA_ONLY_DELAY_MS;

    String json = "{\"ok\":true,\"ip\":\"" + WiFi.localIP().toString() + "\"}";
    server.send(200, "application/json", json);
}

/**
 * @brief 启动 AP_STA 配置门户
 */
static void start_config_portal()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);

    if(!server_started)
    {
        server.on("/", HTTP_GET, handle_root);
        server.on("/scan", HTTP_GET, handle_scan);
        server.on("/connect", HTTP_POST, handle_connect);
        server.begin();
        server_started = true;
    }

    config_portal_active = true;
}

/**
 * @brief 切换到纯 STA 模式并关闭配置门户
 */
static void switch_to_station_only()
{
    if(server_started)
    {
        server.stop();
        server_started = false;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    config_portal_active = false;
    pending_sta_only = false;
}

/**
 * @brief 初始化 WiFi 模块并按 NVS 凭据决定工作模式
 */
void wifi_dev::init()
{
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);

    String ssid;
    String password;
    if(load_credentials(ssid, password) &&
       connect_station(ssid, password, WIFI_STA, START_CONNECT_TIMEOUT_MS))
    {
        config_portal_active = false;
        return;
    }

    start_config_portal();
}

/**
 * @brief WiFi 配置门户任务入口
 *
 * @param arg RTOS 任务参数
 */
void wifi_dev::task_entry(void *arg)
{
    (void)arg;
    while(true)
    {
        if(config_portal_active && server_started)
        {
            server.handleClient();
        }
        if(pending_sta_only && (int32_t)(millis() - sta_only_at_ms) >= 0)
        {
            switch_to_station_only();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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
 * @brief 获取 STA 本地 IP
 *
 * @return STA 本地 IP 地址
 */
IPAddress wifi_dev::station_ip()
{
    return WiFi.localIP();
}
