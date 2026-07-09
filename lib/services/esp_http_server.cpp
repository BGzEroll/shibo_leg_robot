#include "esp_http_server.h"

#include "controller.h"
#include "esp_timer.h"
#include "wifi_dev.h"
#include "xbox_dev.h"
#include <WebServer.h>
#include <WiFi.h>
#include <errno.h>
#include <stdlib.h>

/* ---- HTTP 页面与运行状态 ---- */

static constexpr uint32_t BLE_SCAN_MS = 4000;
static constexpr uint32_t HTTP_TASK_DELAY_MS = 50;
static constexpr uint32_t REMOTE_LEASE_TIMEOUT_MS = 500;
static constexpr uint8_t BLE_SCAN_MAX = 24;

static WebServer server(80);
static QueueHandle_t remote_input_queue = nullptr;
static bool server_started = false;
static uint32_t remote_lease_id = 0;
static uint32_t remote_lease_timestamp_ms = 0;
static constexpr const char *REMOTE_AXIS_ARGS[6] = {"a0", "a1", "a2", "a3", "a4", "a5"};

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
                <a class="module" href="/remote"><strong>手机遥控</strong><span>双摇杆与手柄按键</span></a>
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
)HTML") + extra_modules + R"HTML(                </section>
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
static String remote_html()
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

                const random=new Uint32Array(1);
                crypto.getRandomValues(random);
                const sessionId=random[0]||1;
                const axes=[0,0,0,0,0,0];
                let buttons=0;
                let inFlight=false;
                let sendPending=false;
                const resetters=[];
                const statusEl=document.getElementById('status');

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
                        stick.setPointerCapture(pointerId); update(e);
                    });
                    stick.addEventListener('pointermove',e=>{
                        if(e.pointerId===pointerId){e.preventDefault();update(e);}
                    });
                    ['pointerup','pointercancel','lostpointercapture'].forEach(type=>
                        stick.addEventListener(type,e=>{if(e.pointerId===pointerId){reset();}})
                    );
                    resetters.push(reset);
                }
                document.querySelectorAll('.stick').forEach(bindStick);

                document.querySelectorAll('[data-bit]').forEach(key=>{
                    const bit=Number(key.dataset.bit);
                    const pointers=new Set();
                    function release(e){
                        pointers.delete(e.pointerId);
                        if(!pointers.size){buttons&=~bit;key.classList.remove('active');}
                    }
                    key.addEventListener('pointerdown',e=>{
                        e.preventDefault(); pointers.add(e.pointerId);
                        key.setPointerCapture(e.pointerId);
                        buttons|=bit; key.classList.add('active');
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
                function requestBody(){
                    const body=new URLSearchParams();
                    body.set('s',String(sessionId));
                    body.set('b',String(buttons&0xffff));
                    axes.forEach((value,index)=>
                        body.set(`a${index}`,String(Math.round(Math.max(-1,Math.min(1,value))*1000)))
                    );
                    return body.toString();
                }
                async function sendState(force=false){
                    if(!isPhone||document.hidden){return;}
                    if(inFlight){
                        if(force){sendPending=true;}
                        return;
                    }
                    inFlight=true;
                    try{
                        const response=await fetch('/api/remote/input',{
                            method:'POST',
                            headers:{'Content-Type':'application/x-www-form-urlencoded'},
                            body:requestBody(),
                            cache:'no-store',
                            keepalive:force
                        });
                        if(response.status===204){statusEl.textContent='';}
                        else{
                            const data=await response.json().catch(()=>({}));
                            statusEl.textContent=data.error||'遥控连接不可用';
                        }
                    }catch(e){statusEl.textContent='连接中断';}
                    inFlight=false;
                    if(sendPending){
                        sendPending=false;
                        sendState(true);
                    }
                }
                function sendZero(){
                    clearInput();
                    if(!isPhone)return;
                    const blob=new Blob([requestBody()],{type:'application/x-www-form-urlencoded'});
                    navigator.sendBeacon('/api/remote/input',blob);
                }

                const stickView=document.getElementById('stick-view');
                const buttonView=document.getElementById('button-view');
                document.getElementById('view-toggle').addEventListener('click',()=>{
                    clearInput();
                    const showButtons=buttonView.hidden;
                    buttonView.hidden=!showButtons;
                    stickView.hidden=showButtons;
                    sendState(true);
                });

                document.addEventListener('contextmenu',e=>e.preventDefault());
                document.addEventListener('visibilitychange',()=>{
                    if(document.hidden){sendZero();}else{sendState(true);}
                });
                window.addEventListener('blur',sendZero);
                window.addEventListener('pagehide',sendZero);
                if(isPhone){
                    sendState(true);
                    setInterval(()=>sendState(false),50);
                }
            </script>
        </body>
        </html>)HTML";
}

/**
 * @brief 严格解析无符号十进制请求参数
 *
 * @param name 参数名
 * @param min_value 最小允许值
 * @param max_value 最大允许值
 * @param out 解析结果
 *
 * @return 参数存在且格式和范围有效时返回 true
 */
static bool parse_uint32_arg(const char *name, uint32_t min_value, uint32_t max_value,
    uint32_t &out)
{
    if(!server.hasArg(name)){return false;}

    String value = server.arg(name);
    if(!value.length()){return false;}
    for(uint32_t i = 0; i < (uint32_t)value.length(); i++)
    {
        if(value[i] < '0' || value[i] > '9'){return false;}
    }

    errno = 0;
    char *end = nullptr;
    unsigned long parsed = strtoul(value.c_str(), &end, 10);
    if(errno == ERANGE || !end || *end != '\0' ||
       parsed < min_value || parsed > max_value)
    {
        return false;
    }

    out = (uint32_t)parsed;
    return true;
}

/**
 * @brief 严格解析有符号十进制请求参数
 *
 * @param name 参数名
 * @param min_value 最小允许值
 * @param max_value 最大允许值
 * @param out 解析结果
 *
 * @return 参数存在且格式和范围有效时返回 true
 */
static bool parse_int32_arg(const char *name, int32_t min_value, int32_t max_value,
    int32_t &out)
{
    if(!server.hasArg(name)){return false;}

    String value = server.arg(name);
    if(!value.length()){return false;}
    uint32_t digit_start = value[0] == '-' ? 1 : 0;
    if(digit_start == (uint32_t)value.length()){return false;}
    for(uint32_t i = digit_start; i < (uint32_t)value.length(); i++)
    {
        if(value[i] < '0' || value[i] > '9'){return false;}
    }

    errno = 0;
    char *end = nullptr;
    long parsed = strtol(value.c_str(), &end, 10);
    if(errno == ERANGE || !end || *end != '\0' ||
       parsed < min_value || parsed > max_value)
    {
        return false;
    }

    out = (int32_t)parsed;
    return true;
}

/**
 * @brief 判断网页遥控会话能否取得或续用租约
 *
 * @param session_id 网页会话 ID
 * @param now_ms 当前毫秒时间戳
 *
 * @return 当前会话可使用租约时返回 true
 */
static bool remote_lease_available(uint32_t session_id, uint32_t now_ms)
{
    return remote_lease_id == 0 ||
           remote_lease_id == session_id ||
           (uint32_t)(now_ms - remote_lease_timestamp_ms) > REMOTE_LEASE_TIMEOUT_MS;
}

/**
 * @brief 清空手机网页遥控输入队列
 */
static void clear_remote_input()
{
    if(!remote_input_queue){return;}

    esp_http_server::remote_input_data data;
    xQueueOverwrite(remote_input_queue, &data);
}

/**
 * @brief 清除网页遥控租约与输入
 */
static void clear_remote_lease()
{
    remote_lease_id = 0;
    remote_lease_timestamp_ms = 0;
    clear_remote_input();
}

/**
 * @brief 更新网页遥控租约失效状态
 */
static void update_remote_lease()
{
    if(!remote_lease_id){return;}

    uint32_t now_ms = millis();
    if(wifi_dev::config_portal_active() ||
       (uint32_t)(now_ms - remote_lease_timestamp_ms) > REMOTE_LEASE_TIMEOUT_MS)
    {
        clear_remote_lease();
    }
}

/* ---- HTTP 请求处理 ---- */

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
 * @brief 处理手机遥控页面请求
 */
static void handle_remote_root()
{
    if(wifi_dev::config_portal_active())
    {
        server.send(403, "text/plain; charset=utf-8", "AP 配网模式下不可用");
        return;
    }

    server.send(200, "text/html; charset=utf-8", remote_html());
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
 * @brief 处理舵机中位校准状态查询请求
 */
static void handle_servo_middle_calibration_status()
{
    String json = "{\"ok\":true,\"success\":";
    json += controller::middle_calibration_success() ? "true" : "false";
    json += '}';
    server.send(200, "application/json", json);
}

/**
 * @brief 处理手机网页遥控输入
 */
static void handle_remote_input()
{
    if(wifi_dev::config_portal_active())
    {
        clear_remote_lease();
        server.send(403, "application/json",
            "{\"ok\":false,\"error\":\"AP 配网模式下不可用\"}");
        return;
    }

    uint32_t session_id = 0;
    uint32_t buttons = 0;
    int32_t axis_values[6]{};
    bool valid =
        parse_uint32_arg("s", 1, UINT32_MAX, session_id) &&
        parse_uint32_arg("b", 0, UINT16_MAX, buttons);
    for(uint8_t i = 0; i < 6 && valid; i++)
    {
        valid = parse_int32_arg(REMOTE_AXIS_ARGS[i], -1000, 1000, axis_values[i]);
    }
    if(!valid)
    {
        server.send(400, "application/json",
            "{\"ok\":false,\"error\":\"遥控输入格式无效\"}");
        return;
    }

    if(xbox_dev::connected())
    {
        server.send(409, "application/json",
            "{\"ok\":false,\"error\":\"Xbox 手柄已连接\"}");
        return;
    }

    uint32_t now_ms = millis();
    if(!remote_lease_available(session_id, now_ms))
    {
        server.send(409, "application/json",
            "{\"ok\":false,\"error\":\"遥控器正在被其他设备使用\"}");
        return;
    }

    if(!remote_input_queue)
    {
        server.send(503, "application/json",
            "{\"ok\":false,\"error\":\"遥控输入队列未初始化\"}");
        return;
    }

    float axes[6]{};
    for(uint8_t i = 0; i < 6; i++)
    {
        axes[i] = (float)axis_values[i] / 1000.0f;
    }

    esp_http_server::remote_input_data data;
    data.timestamp_us = (uint32_t)esp_timer_get_time();
    data.buttons = (uint16_t)buttons;
    memcpy(data.axes, axes, sizeof(data.axes));

    remote_lease_id = session_id;
    remote_lease_timestamp_ms = now_ms;
    xQueueOverwrite(remote_input_queue, &data);
    server.send(204, "text/plain", "");
}

/* ---- esp_http_server 公共 API ---- */

/**
 * @brief 获取手机网页遥控输入队列
 *
 * @return 队列句柄
 */
QueueHandle_t esp_http_server::remote_queue()
{
    return remote_input_queue;
}

/**
 * @brief 初始化 HTTP 服务并注册路由
 */
void esp_http_server::init()
{
    if(server_started){return;}

    wifi_dev::init();
    if(!remote_input_queue)
    {
        remote_input_queue = xQueueCreate(1, sizeof(esp_http_server::remote_input_data));
    }
    clear_remote_lease();

    server.on("/", HTTP_GET, handle_root);
    server.on("/wifi", HTTP_GET, handle_wifi_root);
    server.on("/bluetooth", HTTP_GET, handle_bluetooth_root);
    server.on("/servo/middle", HTTP_GET, handle_servo_calibration_root);
    server.on("/remote", HTTP_GET, handle_remote_root);
    server.on("/api/wifi/scan", HTTP_GET, handle_wifi_scan);
    server.on("/api/wifi/connect", HTTP_POST, handle_connect);
    server.on("/api/xbox/status", HTTP_GET, handle_xbox_status);
    server.on("/api/ble/scan", HTTP_GET, handle_ble_scan);
    server.on("/api/xbox/select", HTTP_POST, handle_xbox_select);
    server.on("/api/servo/middle-calibration", HTTP_POST, handle_servo_middle_calibration);
    server.on("/api/servo/middle-calibration/status", HTTP_GET, handle_servo_middle_calibration_status);
    server.on("/api/remote/input", HTTP_POST, handle_remote_input);
    server.begin();
    server_started = true;
}

/* ---- RTOS 任务入口 ---- */

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
        update_remote_lease();
        if(server_started)
        {
            server.handleClient();
        }
        vTaskDelay(pdMS_TO_TICKS(HTTP_TASK_DELAY_MS));
    }
}
