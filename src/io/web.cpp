#include "web_server.h"

#include "controller.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "wifi_dev.h"
#include "xbox_dev.h"
#include <WiFi.h>
#include <esp_http_server.h>
#include <string.h>

/* ---- HTTP 页面与运行状态 ---- */

static constexpr uint32_t BLE_SCAN_MS = 4000;
static constexpr uint32_t SERVICE_TASK_DELAY_MS = 50;
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
static QueueHandle_t remote_input_queue = nullptr;
static portMUX_TYPE remote_lock = portMUX_INITIALIZER_UNLOCKED;
static web_server::input remote_state;
static int remote_socket = -1;
static uint32_t remote_stream_id = 0;
static uint32_t remote_timestamp_ms = 0;
static uint32_t remote_ack_timestamp_ms = 0;
static uint32_t remote_sequence = 0;
static bool remote_sequence_valid = false;

/**
 * @brief 返回主控制台 HTML 前段
 *
 * @return HTML 文本
 */
static const char *console_html_prefix()
{
    return R"HTML(<!doctype html>
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
)HTML";
}

/**
 * @brief 返回主控制台扩展模块 HTML
 *
 * @return HTML 文本
 */
static const char *console_html_modules()
{
    return R"HTML(
                    <a class="module" href="/remote"><strong>手机遥控</strong><span>双摇杆与手柄按键</span></a>
                    <a class="module" href="/servo/middle"><strong>舵机中位校准</strong><span>机械装配</span></a>
)HTML";
}

/**
 * @brief 返回主控制台 HTML 后段
 *
 * @return HTML 文本
 */
static const char *console_html_suffix()
{
    return R"HTML(                </section>
            </main>
        </body>
        </html>)HTML";
}

/**
 * @brief 返回 WiFi 配置页 HTML
 *
 * @return HTML 文本
 */
static const char *wifi_html()
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
                        const res=await fetch('/api/wifi/scan');
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
                    const res=await fetch('/api/wifi/connect',{method:'POST',body});
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
static const char *bluetooth_html()
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
static const char *servo_calibration_html()
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
                const sleep=(ms)=>new Promise(resolve=>setTimeout(resolve,ms));
                async function waitCalibration(){
                    const start=Date.now();
                    while(Date.now()-start<10000){
                        await sleep(500);
                        const data=await (await fetch('/api/servo/middle-calibration/status')).json();
                        if(data.ok&&data.success){return true;}
                    }
                    return false;
                }
                runBtn.onclick=async()=>{
                    runBtn.disabled=true; statusEl.textContent='正在执行...';
                    try{
                        const data=await (await fetch('/api/servo/middle-calibration',{method:'POST'})).json();
                        if(!data.ok){
                            statusEl.textContent=data.error||'执行失败';
                            runBtn.disabled=false;
                            return;
                        }
                        statusEl.textContent='已提交中位校准流程，等待完成...';
                        statusEl.textContent=await waitCalibration()?'校准成功':'校准失败：10 秒内未收到成功回报';
                    }catch(e){statusEl.textContent='执行失败';}
                    runBtn.disabled=false;
                };
            </script>
        </body>
        </html>)HTML";
}

/**
 * @brief 返回手机遥控页面 HTML
 *
 * @return HTML 文本
 */
static const char *remote_html()
{
    return R"HTML(<!doctype html>
        <html lang="zh-CN">
        <head>
            <meta charset="utf-8">
            <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
            <title>Shibo Remote</title>
            <style>
                *{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
                html,body{width:100%;height:100%;margin:0;overflow:hidden;overscroll-behavior:none}
                body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#101419;color:#f8fafc;touch-action:none}
                [hidden]{display:none!important}
                .message{height:100%;display:grid;place-items:center;padding:24px;font-size:20px;text-align:center}
                #remote{width:100vw;height:100dvh;min-height:280px;display:grid;grid-template-rows:48px minmax(0,1fr);padding:max(6px,env(safe-area-inset-top)) max(10px,env(safe-area-inset-right)) max(8px,env(safe-area-inset-bottom)) max(10px,env(safe-area-inset-left))}
                .toolbar{position:relative;display:flex;align-items:center;justify-content:center}
                #status{position:absolute;left:4px;max-width:calc(50% - 30px);overflow:hidden;white-space:nowrap;text-overflow:ellipsis;color:#fca5a5;font-size:12px}
                #view-toggle{width:42px;height:38px;border:1px solid #475569;border-radius:6px;background:#202833;color:#f8fafc;font-size:22px;line-height:1}
                .view{position:relative;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:clamp(12px,8vw,96px);align-items:center;justify-items:center}
                .stick-area{width:100%;height:100%;display:grid;place-items:center;position:relative}
                .stick{position:relative;width:min(39vw,calc(100dvh - 76px));max-width:330px;min-width:112px;aspect-ratio:1;border-radius:50%;border:2px solid #64748b;background:#1b222b;box-shadow:inset 0 0 0 10px #151b22;touch-action:none}
                .stick::before,.stick::after{content:"";position:absolute;background:#394554;pointer-events:none}
                .stick::before{left:50%;top:12%;bottom:12%;width:1px}
                .stick::after{top:50%;left:12%;right:12%;height:1px}
                .knob{position:absolute;left:50%;top:50%;width:34%;aspect-ratio:1;transform:translate(-50%,-50%);border-radius:50%;background:#e2e8f0;border:3px solid #94a3b8;box-shadow:0 4px 12px #0008;pointer-events:none}
                .stick-label{position:absolute;bottom:4px;color:#94a3b8;font-size:12px}
                .button-zone{position:relative;width:100%;height:100%;min-height:220px;display:flex;align-items:center;justify-content:center}
                .shoulder{position:absolute;top:3%;width:clamp(54px,14vw,110px);height:38px;border-radius:6px}
                .left-zone .shoulder{left:8%}.right-zone .shoulder{right:8%}
                .key{border:1px solid #64748b;background:#26313d;color:#f8fafc;font:700 clamp(12px,3vw,18px)/1 system-ui;touch-action:none}
                .key.active{background:#cbd5e1;color:#111827;box-shadow:inset 0 0 0 2px #f8fafc}
                .round{width:clamp(42px,10vw,68px);aspect-ratio:1;border-radius:50%}
                .dpad,.face{position:relative;width:clamp(132px,34vw,230px);aspect-ratio:1}
                .dpad .key,.face .key{position:absolute}
                .up,.y{left:50%;top:0;transform:translateX(-50%)}
                .down,.a{left:50%;bottom:0;transform:translateX(-50%)}
                .left,.x{left:0;top:50%;transform:translateY(-50%)}
                .right,.b{right:0;top:50%;transform:translateY(-50%)}
                .up,.down,.left,.right{width:31%;aspect-ratio:1;border-radius:5px}
                .a{background:#166534}.b{background:#991b1b}.x{background:#1d4ed8}.y{background:#a16207}
                .stick-key{position:absolute;bottom:3%;width:clamp(48px,11vw,76px);height:36px;border-radius:6px}
                .left-zone .stick-key{left:10%}.right-zone .stick-key{right:10%}
                .center-keys{position:absolute;left:50%;bottom:2%;transform:translateX(-50%);display:flex;gap:8px;z-index:2}
                .center-keys .key{height:34px;min-width:58px;padding:0 9px;border-radius:6px;font-size:12px}
                @media (max-width:520px){
                    #remote{grid-template-rows:42px minmax(0,1fr);padding:4px 6px}
                    .view{gap:10px}
                    .stick{width:min(39vw,calc(100dvh - 62px))}
                    .button-zone{min-height:180px}
                }
            </style>
        </head>
        <body>
            <div id="desktop" class="message" hidden>请在手机上打开</div>
            <main id="remote" hidden>
                <div class="toolbar">
                    <span id="status"></span>
                    <button id="view-toggle" type="button" title="切换控制视图" aria-label="切换控制视图">⇄</button>
                </div>
                <section id="stick-view" class="view">
                    <div class="stick-area">
                        <div class="stick" data-x="0" data-y="1"><div class="knob"></div></div>
                        <span class="stick-label">L</span>
                    </div>
                    <div class="stick-area">
                        <div class="stick" data-x="2" data-y="3"><div class="knob"></div></div>
                        <span class="stick-label">R</span>
                    </div>
                </section>
                <section id="button-view" class="view" hidden>
                    <div class="button-zone left-zone">
                        <button class="key shoulder" data-bit="256">LB</button>
                        <div class="dpad">
                            <button class="key up" data-bit="4096">▲</button>
                            <button class="key down" data-bit="32768">▼</button>
                            <button class="key left" data-bit="8192">◀</button>
                            <button class="key right" data-bit="16384">▶</button>
                        </div>
                        <button class="key stick-key" data-bit="1024">LS</button>
                    </div>
                    <div class="button-zone right-zone">
                        <button class="key shoulder" data-bit="512">RB</button>
                        <div class="face">
                            <button class="key round y" data-bit="8">Y</button>
                            <button class="key round a" data-bit="1">A</button>
                            <button class="key round x" data-bit="4">X</button>
                            <button class="key round b" data-bit="2">B</button>
                        </div>
                        <button class="key stick-key" data-bit="2048">RS</button>
                    </div>
                    <div class="center-keys">
                        <button class="key" data-bit="64">SELECT</button>
                        <button class="key" data-bit="32">START</button>
                    </div>
                </section>
            </main>
            <script>
                const isPhone=navigator.maxTouchPoints>0&&matchMedia('(pointer:coarse)').matches;
                const desktop=document.getElementById('desktop');
                const remote=document.getElementById('remote');
                if(!isPhone){desktop.hidden=false;}
                else{remote.hidden=false;}

                const axes=[0,0,0,0,0,0];
                let buttons=0;
                let socket=null;
                let sequence=0;
                let reconnectStep=0;
                let reconnectTimer=null;
                let stateTimer=null;
                let lastStickSendAt=0;
                let controlEnabled=true;
                const sentAt=new Map();
                const reconnectDelays=[250,500,1000,2000];
                const resetters=[];
                const statusEl=document.getElementById('status');

                function scheduleState(){
                    if(stateTimer!==null)return;
                    const now=performance.now();
                    const remaining=20-(now-lastStickSendAt);
                    if(remaining<=0){
                        lastStickSendAt=now;
                        sendState(0);
                        return;
                    }
                    stateTimer=setTimeout(()=>{
                        stateTimer=null;
                        lastStickSendAt=performance.now();
                        sendState(0);
                    },remaining);
                }

                function bindStick(stick){
                    const knob=stick.querySelector('.knob');
                    const xIndex=Number(stick.dataset.x);
                    const yIndex=Number(stick.dataset.y);
                    let pointerId=null;
                    function reset(){
                        pointerId=null;
                        axes[xIndex]=0; axes[yIndex]=0;
                        knob.style.transform='translate(-50%,-50%)';
                    }
                    function update(e){
                        const rect=stick.getBoundingClientRect();
                        const radius=rect.width*.5;
                        let dx=e.clientX-(rect.left+radius);
                        let dy=e.clientY-(rect.top+radius);
                        const distance=Math.hypot(dx,dy);
                        if(distance>radius){dx*=radius/distance;dy*=radius/distance;}
                        axes[xIndex]=Math.max(-1,Math.min(1,dx/radius));
                        axes[yIndex]=Math.max(-1,Math.min(1,-dy/radius));
                        knob.style.transform=`translate(calc(-50% + ${dx}px),calc(-50% + ${dy}px))`;
                    }
                    stick.addEventListener('pointerdown',e=>{
                        if(pointerId!==null)return;
                        e.preventDefault(); pointerId=e.pointerId;
                        stick.setPointerCapture(pointerId); update(e); scheduleState();
                    });
                    stick.addEventListener('pointermove',e=>{
                        if(e.pointerId===pointerId){e.preventDefault();update(e);scheduleState();}
                    });
                    ['pointerup','pointercancel','lostpointercapture'].forEach(type=>
                        stick.addEventListener(type,e=>{
                            if(e.pointerId===pointerId){reset();sendState(0);}
                        })
                    );
                    resetters.push(reset);
                }
                document.querySelectorAll('.stick').forEach(bindStick);

                document.querySelectorAll('[data-bit]').forEach(key=>{
                    const bit=Number(key.dataset.bit);
                    const pointers=new Set();
                    function release(e){
                        pointers.delete(e.pointerId);
                        if(!pointers.size){
                            buttons&=~bit;
                            key.classList.remove('active');
                            sendState(0);
                        }
                    }
                    key.addEventListener('pointerdown',e=>{
                        e.preventDefault(); pointers.add(e.pointerId);
                        key.setPointerCapture(e.pointerId);
                        buttons|=bit; key.classList.add('active');
                        sendState(bit);
                    });
                    ['pointerup','pointercancel','lostpointercapture'].forEach(type=>
                        key.addEventListener(type,release)
                    );
                    resetters.push(()=>{
                        pointers.clear(); buttons&=~bit; key.classList.remove('active');
                    });
                });

                function clearInput(){
                    axes.fill(0); buttons=0;
                    resetters.forEach(reset=>reset());
                }
                function buildFrame(pressedButtons){
                    const buffer=new ArrayBuffer(22);
                    const view=new DataView(buffer);
                    const frameSequence=(++sequence)>>>0;
                    view.setUint8(0,1);
                    view.setUint8(1,1);
                    view.setUint32(2,frameSequence,true);
                    view.setUint16(6,buttons&0xffff,true);
                    view.setUint16(8,pressedButtons&0xffff,true);
                    axes.forEach((value,index)=>
                        view.setInt16(10+index*2,Math.round(Math.max(-1,Math.min(1,value))*1000),true)
                    );
                    sentAt.set(frameSequence,performance.now());
                    if(sentAt.size>16){sentAt.delete(sentAt.keys().next().value);}
                    return {buffer,frameSequence};
                }
                function sendState(pressedButtons){
                    if(!socket||socket.readyState!==WebSocket.OPEN)return false;
                    if(socket.bufferedAmount>256){
                        statusEl.textContent='发送积压，正在重连';
                        socket.close(1011,'backpressure');
                        return false;
                    }
                    const frame=buildFrame(pressedButtons);
                    socket.send(frame.buffer);
                    return true;
                }
                function closeControl(){
                    controlEnabled=false;
                    if(reconnectTimer!==null){
                        clearTimeout(reconnectTimer);
                        reconnectTimer=null;
                    }
                    if(stateTimer!==null){
                        clearTimeout(stateTimer);
                        stateTimer=null;
                    }
                    clearInput();
                    sendState(0);
                    if(socket&&socket.readyState<=WebSocket.OPEN){
                        const closingSocket=socket;
                        try{closingSocket.close(1000,'inactive');}
                        catch(error){
                            closingSocket.onopen=()=>closingSocket.close(1000,'inactive');
                        }
                    }
                }
                function resumeControl(){
                    controlEnabled=true;
                    connect();
                }
                function scheduleReconnect(){
                    if(!controlEnabled||document.hidden||reconnectTimer!==null)return;
                    const delay=reconnectDelays[Math.min(reconnectStep,reconnectDelays.length-1)];
                    reconnectStep++;
                    reconnectTimer=setTimeout(()=>{reconnectTimer=null;connect();},delay);
                }
                function connect(){
                    if(!isPhone||!controlEnabled||document.hidden||
                       socket!==null)return;
                    statusEl.textContent='正在连接';
                    const scheme=location.protocol==='https:'?'wss':'ws';
                    socket=new WebSocket(`${scheme}://${location.host}/ws/remote`,'shibo-remote-v1');
                    socket.binaryType='arraybuffer';
                    socket.onopen=()=>{
                        reconnectStep=0;
                        sequence=0;
                        sentAt.clear();
                        clearInput();
                        statusEl.textContent='已连接';
                        sendState(0);
                    };
                    socket.onmessage=e=>{
                        if(!(e.data instanceof ArrayBuffer)||e.data.byteLength!==8)return;
                        const view=new DataView(e.data);
                        if(view.getUint8(0)!==1||view.getUint8(1)!==0x81)return;
                        const status=view.getUint8(2);
                        const ack=view.getUint32(4,true);
                        const started=sentAt.get(ack);
                        if(status===0&&started!==undefined){
                            statusEl.textContent=`已连接 · ${Math.round(performance.now()-started)} ms`;
                        }else if(status!==0){
                            const errors={1:'协议帧错误',2:'Xbox 已接管',3:'已有遥控连接',4:'AP 配网模式不可用'};
                            statusEl.textContent=errors[status]||`控制错误 ${status}`;
                        }
                        for(const key of sentAt.keys()){
                            if(key<=ack){sentAt.delete(key);}
                        }
                    };
                    socket.onerror=()=>{statusEl.textContent='连接错误';};
                    socket.onclose=()=>{
                        socket=null;
                        clearInput();
                        statusEl.textContent='连接中断';
                        scheduleReconnect();
                    };
                }

                const stickView=document.getElementById('stick-view');
                const buttonView=document.getElementById('button-view');
                document.getElementById('view-toggle').addEventListener('click',()=>{
                    clearInput();
                    const showButtons=buttonView.hidden;
                    buttonView.hidden=!showButtons;
                    stickView.hidden=showButtons;
                    sendState(0);
                });

                document.addEventListener('contextmenu',e=>e.preventDefault());
                document.addEventListener('visibilitychange',()=>{
                    if(document.hidden){closeControl();}else{resumeControl();}
                });
                window.addEventListener('blur',closeControl);
                window.addEventListener('focus',resumeControl);
                window.addEventListener('pagehide',closeControl);
                if(isPhone){
                    connect();
                    setInterval(()=>sendState(0),50);
                }
            </script>
        </body>
        </html>)HTML";
}

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
        remote_state = web_server::input{};
        remote_state.stream_id = remote_stream_id;
        if(remote_input_queue)
        {
            xQueueOverwrite(remote_input_queue, &remote_state);
        }
        cleared = true;
    }
    portEXIT_CRITICAL(&remote_lock);

    if(cleared){wifi_dev::set_low_latency_mode(remote_active());}
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
    if(wifi_dev::config_portal_active() || xbox_dev::connected() || expired)
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
    if(httpd_resp_sendstr_chunk(req, console_html_prefix()) != ESP_OK){return ESP_FAIL;}
    if(!wifi_dev::config_portal_active() &&
       httpd_resp_sendstr_chunk(req, console_html_modules()) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if(httpd_resp_sendstr_chunk(req, console_html_suffix()) != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理 WiFi 配置页面请求
 */
static esp_err_t handle_wifi_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", wifi_html());
}

/**
 * @brief 处理蓝牙设置页面请求
 */
static esp_err_t handle_bluetooth_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", bluetooth_html());
}

/**
 * @brief 处理舵机中位校准页面请求
 */
static esp_err_t handle_servo_calibration_root(httpd_req_t *req)
{
    return send_response(req, "200 OK", "text/html; charset=utf-8", servo_calibration_html());
}

/**
 * @brief 处理手机遥控页面请求
 */
static esp_err_t handle_remote_root(httpd_req_t *req)
{
    if(wifi_dev::config_portal_active())
    {
        return send_response(req, "403 Forbidden", "text/plain; charset=utf-8",
            "AP 配网模式下不可用");
    }

    return send_response(req, "200 OK", "text/html; charset=utf-8", remote_html());
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
    bool ok = wifi_dev::connect_and_save(String(ssid), String(password), ip);
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
    const char *prefix = xbox_dev::connected() ?
        "{\"connected\":true,\"target\":" :
        "{\"connected\":false,\"target\":";
    if(httpd_resp_sendstr_chunk(req, prefix) != ESP_OK){return ESP_FAIL;}
    if(!send_json_string(req, xbox_dev::target_address())){return ESP_FAIL;}
    if(httpd_resp_send_chunk(req, "}", 1) != ESP_OK){return ESP_FAIL;}
    return httpd_resp_send_chunk(req, nullptr, 0);
}

/**
 * @brief 处理 BLE 扫描请求
 */
static esp_err_t handle_ble_scan(httpd_req_t *req)
{
    if(!management_available(req)){return ESP_OK;}

    xbox_dev::ble_device devices[BLE_SCAN_MAX];
    uint8_t count = 0;
    bool ok = xbox_dev::scan_ble(devices, BLE_SCAN_MAX, count, BLE_SCAN_MS);
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
    if(!xbox_dev::set_target_address(String(address)))
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
    if(wifi_dev::config_portal_active())
    {
        return send_json(req, "403 Forbidden",
            "{\"ok\":false,\"error\":\"AP 配网模式下不可用\"}");
    }

    controller::request_middle_calibration();
    return send_json(req, "200 OK", "{\"ok\":true}");
}

/**
 * @brief 处理舵机中位校准状态查询请求
 */
static esp_err_t handle_servo_middle_calibration_status(httpd_req_t *req)
{
    return send_json(req, "200 OK", controller::middle_calibration_success() ?
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
        if(wifi_dev::config_portal_active())
        {
            reject_status = REMOTE_STATUS_AP_FORBIDDEN;
        }
        else if(xbox_dev::connected())
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
            remote_state = web_server::input{};
            remote_state.stream_id = ++remote_stream_id;
            xQueueOverwrite(remote_input_queue, &remote_state);
            acquired = true;
        }
        portEXIT_CRITICAL(&remote_lock);

        if(!acquired)
        {
            if(reject_status == REMOTE_STATUS_OK){reject_status = REMOTE_STATUS_CONTROL_LOCKED;}
            send_remote_status(req, reject_status, 0);
            return ESP_FAIL;
        }

        wifi_dev::set_low_latency_mode(true);
        return ESP_OK;
    }

    if(xbox_dev::connected())
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
        remote_state.held_buttons = held_buttons;
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
        if(xQueueOverwrite(remote_input_queue, &remote_state) == pdPASS)
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

/* ---- web_server 公共 API ---- */

/**
 * @brief 读取网页遥控最新输入快照
 *
 * @param out 网页遥控输入输出
 *
 * @return 队列存在且已有快照时返回 true
 */
bool web_server::peek_input(web_server::input &out)
{
    return remote_input_queue &&
           xQueuePeek(remote_input_queue, &out, 0) == pdTRUE;
}

/**
 * @brief 初始化原生 HTTP 和 WebSocket 服务
 *
 * @return 服务及全部路由启动成功时返回 true
 */
bool web_server::init()
{
    if(server){return true;}

    wifi_dev::init();
    if(!remote_input_queue)
    {
        remote_input_queue = xQueueCreate(1, sizeof(web_server::input));
    }
    if(!remote_input_queue){return false;}
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
void web_server::task_entry(void *arg)
{
    while(true)
    {
        wifi_dev::update();
        update_remote_session();
        vTaskDelay(pdMS_TO_TICKS(SERVICE_TASK_DELAY_MS));
    }
}
