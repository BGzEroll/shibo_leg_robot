#ifndef IO_WEB_PAGES_H
#define IO_WEB_PAGES_H

namespace io
{
    namespace web_pages
    {
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
    }
}

#endif
