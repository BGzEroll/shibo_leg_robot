#include "esp_http_server.h"

#include "controller.h"
#include "wifi_dev.h"
#include "xbox_dev.h"
#include <WebServer.h>
#include <WiFi.h>

static constexpr uint32_t BLE_SCAN_MS = 4000;
static constexpr uint8_t BLE_SCAN_MAX = 24;

static WebServer server(80);
static bool server_started = false;

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
    for(uint32_t i = 0; i < (uint32_t)value.length(); i++)
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
 * @brief 返回主控制台 HTML
 *
 * @return HTML 文本
 */
static String console_html()
{
    String extra_modules;
    if(!wifi_dev::config_portal_active())
    {
        extra_modules = R"HTML(
				<a class="module" href="/servo/middle"><strong>舵机中位校准</strong><span>机械装配</span></a>
)HTML";
    }

    return String(R"HTML(<!doctype html>
		<html lang="zh-CN">
		<head>
			<meta charset="utf-8">
			<meta name="viewport" content="width=device-width,initial-scale=1">
			<title>Console</title>
			<style>
				body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#eef2f7;color:#111827}
				main{max-width:760px;margin:0 auto;padding:22px 16px}
				h1{font-size:26px;margin:0 0 18px}
				.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}
				.module{display:block;background:white;border:1px solid #d5dce8;border-radius:6px;padding:16px;text-decoration:none;color:#111827}
				.module strong{display:block;font-size:18px;margin-bottom:6px}
				.module span{color:#64748b;font-size:14px}
			</style>
		</head>
		<body>
			<main>
				<h1>Console</h1>
				<section class="grid">
					<a class="module" href="/wifi"><strong>WiFi 设置</strong><span>网络连接</span></a>
					<a class="module" href="/bluetooth"><strong>蓝牙设置</strong><span>Xbox 手柄</span></a>
)HTML") + extra_modules + R"HTML(				</section>
			</main>
		</body>
		</html>)HTML";
}

/**
 * @brief 返回 WiFi 配置页 HTML
 *
 * @return HTML 文本
 */
static String wifi_html()
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
				nav{margin-bottom:14px}
				button,input{font:inherit}
				button{border:0;background:#1d4ed8;color:white;padding:10px 14px;border-radius:6px}
				button:disabled{opacity:.55}
				.network{display:flex;justify-content:space-between;align-items:center;border:1px solid #d8deea;background:white;border-radius:6px;padding:10px 12px;margin:8px 0}
				.network button{background:#334155;padding:7px 10px}
				form{display:grid;gap:10px;margin-top:18px;background:white;border:1px solid #d8deea;border-radius:6px;padding:14px}
				input{box-sizing:border-box;width:100%;padding:10px;border:1px solid #cbd5e1;border-radius:6px}
				#status{margin-top:12px;min-height:24px;color:#475569}
				small{color:#64748b}
				a{color:#1d4ed8}
			</style>
		</head>
		<body>
			<main>
				<nav><a href="/">Console</a></nav>
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
						const res=await fetch('/wifi/scan');
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
 * @brief 返回蓝牙设置页 HTML
 *
 * @return HTML 文本
 */
static String bluetooth_html()
{
    return R"HTML(<!doctype html>
		<html lang="zh-CN">
		<head>
			<meta charset="utf-8">
			<meta name="viewport" content="width=device-width,initial-scale=1">
			<title>Shibo Bluetooth</title>
			<style>
				body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#eef2f7;color:#111827}
				main{max-width:760px;margin:0 auto;padding:22px 16px}
				header{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:18px}
				h1{font-size:24px;margin:0}
				button{font:inherit;border:0;background:#2563eb;color:white;padding:9px 13px;border-radius:6px}
				button:disabled{opacity:.55}
				a{color:#2563eb;text-decoration:none}
				.panel{background:white;border:1px solid #d5dce8;border-radius:6px;padding:14px;margin:12px 0}
				.row{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid #edf0f5;padding:8px 0}
				.row:last-child{border-bottom:0}
				.device{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;border:1px solid #d5dce8;background:white;border-radius:6px;padding:10px 12px;margin:8px 0}
				.name{font-weight:600}
				.meta{color:#64748b;font-size:13px;margin-top:3px}
				.tag{display:inline-block;color:#166534;background:#dcfce7;border-radius:4px;padding:1px 5px;margin-left:6px;font-size:12px}
				#status{min-height:24px;color:#475569}
			</style>
		</head>
		<body>
			<main>
				<header><h1>蓝牙设置</h1><a href="/">Console</a></header>
				<section class="panel">
					<div class="row"><span>手柄连接</span><strong id="connected">--</strong></div>
					<div class="row"><span>目标地址</span><strong id="target">--</strong></div>
				</section>
				<button id="scan">扫描蓝牙设备</button>
				<div id="status"></div>
				<div id="devices"></div>
			</main>
			<script>
				const scanBtn=document.getElementById('scan');
				const statusEl=document.getElementById('status');
				const devicesEl=document.getElementById('devices');
				async function refreshStatus(){
					try{
						const data=await (await fetch('/api/xbox/status')).json();
						document.getElementById('connected').textContent=data.connected?'已连接':'未连接';
						document.getElementById('target').textContent=data.target||'自动发现';
					}catch(e){}
				}
				function item(dev){
					const name=dev.name||'未命名设备';
					const tag=dev.xbox?'<span class="tag">Xbox</span>':'';
					return `<div class="device"><div><div class="name">${name}${tag}</div><div class="meta">${dev.address} · ${dev.rssi} dBm · ${dev.connectable?'可连接':'广播'}</div></div><button data-address="${dev.address}">选择</button></div>`;
				}
				scanBtn.onclick=async()=>{
					scanBtn.disabled=true; devicesEl.innerHTML=''; statusEl.textContent='正在扫描 BLE，约 4 秒...';
					try{
						const data=await (await fetch('/api/ble/scan')).json();
						devicesEl.innerHTML=data.devices.map(item).join('') || '<p>未发现蓝牙设备</p>';
						devicesEl.querySelectorAll('button').forEach(btn=>btn.onclick=async()=>{
							statusEl.textContent='正在保存目标手柄...';
							const body=new URLSearchParams({address:btn.dataset.address});
							const result=await (await fetch('/api/xbox/select',{method:'POST',body})).json();
							statusEl.textContent=result.ok?'已保存，正在用新地址重新连接手柄':(result.error||'保存失败');
						});
						statusEl.textContent='扫描完成';
					}catch(e){statusEl.textContent='扫描失败';}
					scanBtn.disabled=false;
				};
				refreshStatus(); setInterval(refreshStatus,2000);
			</script>
		</body>
		</html>)HTML";
}

/**
 * @brief 返回舵机中位校准页 HTML
 *
 * @return HTML 文本
 */
static String servo_calibration_html()
{
    return R"HTML(<!doctype html>
		<html lang="zh-CN">
		<head>
			<meta charset="utf-8">
			<meta name="viewport" content="width=device-width,initial-scale=1">
			<title>Servo Calibration</title>
			<style>
				body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#eef2f7;color:#111827}
				main{max-width:560px;margin:0 auto;padding:22px 16px}
				header{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:18px}
				h1{font-size:24px;margin:0}
				button{font:inherit;border:0;background:#2563eb;color:white;padding:10px 14px;border-radius:6px}
				button:disabled{opacity:.55}
				a{color:#2563eb;text-decoration:none}
				.panel{background:white;border:1px solid #d5dce8;border-radius:6px;padding:14px;margin:12px 0}
				#status{min-height:24px;color:#475569}
			</style>
		</head>
		<body>
			<main>
				<header><h1>舵机中位校准</h1><a href="/">Console</a></header>
				<section class="panel">
					<button id="run">执行</button>
					<div id="status"></div>
				</section>
			</main>
			<script>
				const runBtn=document.getElementById('run');
				const statusEl=document.getElementById('status');
				runBtn.onclick=async()=>{
					runBtn.disabled=true; statusEl.textContent='正在执行...';
					try{
						const data=await (await fetch('/api/servo/middle-calibration',{method:'POST'})).json();
						statusEl.textContent=data.ok?'已提交中位校准流程':(data.error||'执行失败');
					}catch(e){statusEl.textContent='执行失败';}
					runBtn.disabled=false;
				};
			</script>
		</body>
		</html>)HTML";
}

/**
 * @brief 处理根页面请求
 */
static void handle_root()
{
    server.send(200, "text/html; charset=utf-8", console_html());
}

/**
 * @brief 处理 WiFi 配置页面请求
 */
static void handle_wifi_root()
{
    server.send(200, "text/html; charset=utf-8", wifi_html());
}

/**
 * @brief 处理蓝牙设置页面请求
 */
static void handle_bluetooth_root()
{
    server.send(200, "text/html; charset=utf-8", bluetooth_html());
}

/**
 * @brief 处理舵机中位校准页面请求
 */
static void handle_servo_calibration_root()
{
    server.send(200, "text/html; charset=utf-8", servo_calibration_html());
}

/**
 * @brief 处理 WiFi 扫描请求
 */
static void handle_wifi_scan()
{
    int32_t count = WiFi.scanNetworks(false, true);
    String json = "[";
    for(int32_t i = 0; i < count; i++)
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

    IPAddress ip;
    bool ok = wifi_dev::connect_and_save(ssid, password, ip);
    if(!ok)
    {
        server.send(200, "application/json", "{\"ok\":false,\"error\":\"连接超时\"}");
        return;
    }

    String json = "{\"ok\":true,\"ip\":\"" + ip.toString() + "\"}";
    server.send(200, "application/json", json);
}

/**
 * @brief 处理 Xbox 状态查询请求
 */
static void handle_xbox_status()
{
    String json = "{\"connected\":" + String(xbox_dev::connected() ? "true" : "false") + ',';
    json += "\"target\":\"" + json_escape(xbox_dev::target_address()) + "\"}";
    server.send(200, "application/json", json);
}

/**
 * @brief 处理 BLE 扫描请求
 */
static void handle_ble_scan()
{
    xbox_dev::ble_device devices[BLE_SCAN_MAX];
    uint8_t count = 0;
    bool ok = xbox_dev::scan_ble(devices, BLE_SCAN_MAX, count, BLE_SCAN_MS);
    if(!ok)
    {
        server.send(500, "application/json", "{\"ok\":false,\"devices\":[]}");
        return;
    }

    String json = "{\"ok\":true,\"devices\":[";
    for(uint8_t i = 0; i < count; i++)
    {
        if(i){json += ',';}
        json += "{\"address\":\"" + json_escape(devices[i].address) + "\",";
        json += "\"name\":\"" + json_escape(devices[i].name) + "\",";
        json += "\"rssi\":" + String(devices[i].rssi) + ',';
        json += "\"xbox\":" + String(devices[i].xbox ? "true" : "false") + ',';
        json += "\"connectable\":" + String(devices[i].connectable ? "true" : "false") + '}';
    }
    json += "]}";
    server.send(200, "application/json", json);
}

/**
 * @brief 处理目标 Xbox 手柄选择请求
 */
static void handle_xbox_select()
{
    String address = server.arg("address");
    if(!xbox_dev::set_target_address(address))
    {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"蓝牙地址无效\"}");
        return;
    }

    server.send(200, "application/json", "{\"ok\":true}");
}

/**
 * @brief 处理舵机中位校准执行请求
 */
static void handle_servo_middle_calibration()
{
    if(wifi_dev::config_portal_active())
    {
        server.send(403, "application/json", "{\"ok\":false,\"error\":\"AP 配网模式下不可用\"}");
        return;
    }

    controller::request_middle_calibration();
    server.send(200, "application/json", "{\"ok\":true}");
}

/**
 * @brief 初始化 HTTP 服务并注册路由
 */
void esp_http_server::init()
{
    if(server_started){return;}

    wifi_dev::init();

    server.on("/", HTTP_GET, handle_root);
    server.on("/wifi", HTTP_GET, handle_wifi_root);
    server.on("/bluetooth", HTTP_GET, handle_bluetooth_root);
    server.on("/servo/middle", HTTP_GET, handle_servo_calibration_root);
    server.on("/scan", HTTP_GET, handle_wifi_scan);
    server.on("/wifi/scan", HTTP_GET, handle_wifi_scan);
    server.on("/connect", HTTP_POST, handle_connect);
    server.on("/api/wifi/connect", HTTP_POST, handle_connect);
    server.on("/api/xbox/status", HTTP_GET, handle_xbox_status);
    server.on("/api/ble/scan", HTTP_GET, handle_ble_scan);
    server.on("/api/xbox/select", HTTP_POST, handle_xbox_select);
    server.on("/api/servo/middle-calibration", HTTP_POST, handle_servo_middle_calibration);
    server.begin();
    server_started = true;
}

/**
 * @brief HTTP 服务任务入口
 *
 * @param arg RTOS 任务参数
 */
void esp_http_server::task_entry(void *arg)
{
    while(true)
    {
        wifi_dev::update();
        if(server_started)
        {
            server.handleClient();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
