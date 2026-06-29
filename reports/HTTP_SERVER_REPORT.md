# ESP HTTP Server 报告

本文档说明当前 `lib/services/esp_http_server.*` 的职责、启动链路、Console 主页面、WiFi 配网流程、蓝牙设置页面和 BLE/Xbox 相关接口。

结论：当前 HTTP server 已经从 `devices` 层抽到 `services` 层，作为“板载网页服务入口”存在。它内部拉起 `wifi_dev`，统一用 `/` 提供 Console 主页面，再把 WiFi 设置和蓝牙设置作为子模块入口；Xbox 蓝牙扫描和目标手柄选择仍由 `xbox_dev` 执行，HTTP server 只负责把网页请求转成服务调用。

## 模块边界

当前相关模块分工如下：

- `esp_http_server`：注册 HTTP 路由、返回 HTML/JSON、在任务中调用 `server.handleClient()`。
- `wifi_dev`：管理 WiFi 模式、NVS 凭据、STA 连接、AP_STA 配置门户、STA-only 延迟切换。
- `xbox_dev`：管理 Xbox BLE 连接对象、扫描 BLE 设备、保存目标手柄地址、重建手柄连接。
- `start.cpp`：调用 `esp_http_server::init()` 并创建 `esp_http_server::task_entry`。

这意味着网页服务已经成为 WiFi 和简单上位机功能的入口，但硬件设备行为仍在各自 `devices` 模块中。

## 启动链路

系统启动入口在 `start_init_all()`：

```cpp
led_dev::init();
xbox_dev::init();
esp_http_server::init();
controller::init();
task_list();
```

HTTP 服务初始化链路：

```text
start_init_all()
        |
        v
esp_http_server::init()
        |
        v
wifi_dev::init()
        |
        +-- NVS 有 WiFi 凭据且连接成功 -> WIFI_STA
        |
        +-- 无凭据或连接失败 -> WIFI_AP_STA + 配置热点
        |
        v
注册 HTTP 路由
        |
        v
server.begin()
```

任务创建后，`esp_http_server::task_entry()` 每 10 ms 执行：

```text
wifi_dev::update()
server.handleClient()
vTaskDelay(10 ms)
```

`wifi_dev::update()` 目前只负责处理“配网成功后延迟切换到纯 STA 模式”的维护逻辑。

## WiFi 工作模式

`wifi_dev::init()` 会先关闭持久化写入并打开自动重连：

```cpp
WiFi.persistent(false);
WiFi.setAutoReconnect(true);
```

随后尝试读取 NVS：

- namespace：`wifi`
- SSID key：`ssid`
- password key：`pass`

启动时行为：

1. 如果 NVS 中有 SSID，并且在 `START_CONNECT_TIMEOUT_MS = 8000` ms 内连接成功：
   - 使用 `WIFI_STA`。
   - `portal_active = false`。
   - 不启动配置热点。
2. 如果没有凭据，或连接失败：
   - 使用 `WIFI_AP_STA`。
   - 启动配置热点。
   - `portal_active = true`。

配置热点参数：

- SSID：`SHIBO_LEG_ROBOT`
- password：`12345678`

通过网页提交新 WiFi 后：

1. `wifi_dev::connect_and_save()` 在 `WIFI_AP_STA` 模式下尝试连接新路由。
2. 最多等待 `PORTAL_CONNECT_TIMEOUT_MS = 12000` ms。
3. 成功后保存 SSID/password 到 NVS。
4. 返回 STA IP 给网页。
5. 设置 `pending_sta_only = true`。
6. `STA_ONLY_DELAY_MS = 2000` ms 后由 `wifi_dev::update()` 切到纯 `WIFI_STA` 并关闭配置热点。

这样配置成功后不需要重启，网页先能拿到成功响应，然后设备再延迟关闭 AP。

## Console 主页面

`handle_root()` 统一返回 Console 主页面：

```cpp
server.send(200, "text/html; charset=utf-8", console_html());
```

Console 的职责是只展示子模块入口，不直接承载具体功能页面。当前入口包括：

- `/wifi`：WiFi 设置。
- `/bluetooth`：蓝牙设置。

AP 配网模式和 Station 模式都使用同一个 Console。AP 配网模式下只显示 WiFi 设置和蓝牙设置入口；Station 模式下额外显示舵机中位校准等运行维护入口。后续如果新增更多上位机模块，应在 `console_html()` 中按 `wifi_dev::config_portal_active()` 控制可见性，避免 AP 配网状态暴露无关入口。

`/wifi` 总是返回 WiFi 设置页面，即使当前已经处于 Station 模式。

## WiFi 配网页

配网页由 `wifi_html()` 返回，页面功能：

- 扫描周围 WiFi。
- 列表显示 SSID、RSSI、是否加密。
- 点击列表项将 SSID 填入表单。
- 输入密码后提交连接并保存。
- 连接成功后显示 STA IP。

相关路由：

```text
GET  /wifi
GET  /scan
GET  /wifi/scan
POST /connect
POST /api/wifi/connect
```

`/scan` 和 `/wifi/scan` 目前是同一个处理函数，返回 JSON 数组：

```json
[
  {
    "ssid": "example",
    "rssi": -50,
    "secure": true
  }
]
```

`/connect` 和 `/api/wifi/connect` 也是同一个处理函数，请求参数：

```text
ssid=<SSID>
password=<PASSWORD>
```

成功响应：

```json
{
  "ok": true,
  "ip": "192.168.x.x"
}
```

失败响应：

```json
{
  "ok": false,
  "error": "连接超时"
}
```

如果 SSID 为空，返回 HTTP 400：

```json
{
  "ok": false,
  "error": "SSID 为空"
}
```

## 蓝牙设置页面

蓝牙设置页由 `bluetooth_html()` 返回。当前页面功能比较轻量，主要围绕 Xbox 手柄：

- 显示手柄连接状态。
- 显示当前目标手柄地址。
- 扫描周围 BLE 设备。
- 列表显示设备名、地址、RSSI、是否可连接、是否像 Xbox。
- 点击某个设备后保存其地址为目标手柄地址，并触发 `xbox_dev` 重建连接。
- 页面右上角保留 `/` 链接，可回到 Console 主页面。

相关路由：

```text
GET  /api/xbox/status
GET  /api/ble/scan
POST /api/xbox/select
```

`/api/xbox/status` 返回：

```json
{
  "connected": true,
  "target": "aa:bb:cc:dd:ee:ff"
}
```

如果没有保存目标地址，`target` 为空字符串，页面显示为“自动发现”。

`/api/ble/scan` 会调用：

```cpp
xbox_dev::scan_ble(devices, BLE_SCAN_MAX, count, BLE_SCAN_MS)
```

当前参数：

- `BLE_SCAN_MS = 4000`
- `BLE_SCAN_MAX = 24`

成功响应：

```json
{
  "ok": true,
  "devices": [
    {
      "address": "aa:bb:cc:dd:ee:ff",
      "name": "Xbox Wireless Controller",
      "rssi": -42,
      "xbox": true,
      "connectable": true
    }
  ]
}
```

失败响应：

```json
{
  "ok": false,
  "devices": []
}
```

`/api/xbox/select` 请求参数：

```text
address=<BLE_ADDRESS>
```

成功后：

- `xbox_dev::set_target_address()` 校验地址长度。
- 地址保存到 NVS。
- 停止当前 BLE 扫描和连接。
- 删除旧 `xbox` 对象。
- 清空输入队列状态。
- 用新地址重建 `xbox` 对象并调用 `init()`。

成功响应：

```json
{
  "ok": true
}
```

地址无效时返回 HTTP 400：

```json
{
  "ok": false,
  "error": "蓝牙地址无效"
}
```

## 舵机中位校准页面

舵机中位校准页由 `servo_calibration_html()` 返回。该入口只在 Station 模式下从 Console 显示，页面目前只有一个执行按钮。

相关路由：

```text
GET  /servo/middle
POST /api/servo/middle-calibration
```

点击执行按钮后，页面会向 `/api/servo/middle-calibration` 发送 POST 请求。服务端只向控制器提交请求：

```cpp
controller::request_middle_calibration();
```

控制器收到请求后，不会由 HTTP 任务直接操作舵机，而是在 `BALANCE` 模式下进入 `MIDDLE_CALIBRATION` 动作流程：

1. 先按 SIT 流程让小车坐下。
2. 坐下完成后关闭电机输出并清空平衡参考。
3. 执行 `sts3032::calibrate_middle()`。
4. 发送起立姿态。
5. 进入 recover，稳定后回到 `BALANCE`。

当前 `sts3032::calibrate_middle()` 行为：

- 左舵机关闭扭矩，等待 `100` ms。
- 右舵机关闭扭矩，等待 `1000` ms。
- 左右舵机扭矩切回 `128`。

成功响应：

```json
{
  "ok": true
}
```

如果仍在 AP 配网模式下直接访问执行接口，返回 HTTP 403：

```json
{
  "ok": false,
  "error": "AP 配网模式下不可用"
}
```

## BLE 扫描与 Xbox 连接互斥

`xbox_dev` 内部使用 `ble_scan_active` 避免扫描期间和手柄采样任务冲突：

```text
HTTP /api/ble/scan
        |
        v
xbox_dev::scan_ble()
        |
        v
ble_scan_active = true
        |
        v
xbox_dev::task_entry() 暂停 gamepad->update()
        |
        v
扫描完成后 ble_scan_active = false
```

选择新手柄地址时，`rebuild_gamepad()` 也会临时设置 `ble_scan_active = true`，停止扫描和连接，删除旧 client，再重建 `xbox` 对象。

这部分目前是同步阻塞式调用：HTTP 请求会等待 BLE 扫描约 4 秒。优点是实现简单；缺点是扫描期间 WebServer 当前任务被占用，不过控制器和平衡任务在其他 RTOS 任务中继续运行。

## 路由表

当前 HTTP 路由如下：

```text
GET  /                  返回 Console 主页面
GET  /wifi              返回 WiFi 设置页面
GET  /bluetooth         返回蓝牙设置页面
GET  /servo/middle      返回舵机中位校准页面
GET  /scan              扫描 WiFi，兼容旧入口
GET  /wifi/scan         扫描 WiFi
POST /connect           连接并保存 WiFi，兼容旧入口
POST /api/wifi/connect  连接并保存 WiFi
GET  /api/xbox/status   查询 Xbox 连接状态和目标地址
GET  /api/ble/scan      扫描 BLE 设备
POST /api/xbox/select   保存目标 Xbox 地址并重建连接
POST /api/servo/middle-calibration 提交舵机中位校准流程
```

## 当前优点

1. WiFi 配网不再需要重启，成功后延迟关闭 AP 并切到 STA。
2. HTTP service 统一托管网页，上层不需要在 `start.cpp` 中分别管理 WiFi 任务。
3. `wifi_dev` 和 `xbox_dev` 仍然保留设备所有权，HTTP server 只做服务编排。
4. `/scan`、`/connect` 保留兼容入口，同时新增了更清晰的 `/api/...` 入口。
5. Console 主页面已经把功能拆成子模块入口，后续增加网页功能时不用继续堆在根页面。
6. 当前页面已经能覆盖最需要的实机操作：配置 WiFi、扫描 BLE、选择 Xbox 手柄、执行舵机中位校准。

## 当前限制和风险

1. HTML/CSS/JS 直接以内嵌字符串放在 `.cpp` 中，后续页面复杂后维护成本会变高。
2. BLE 扫描是同步 HTTP 请求，扫描期间当前 WebServer 任务会阻塞约 4 秒。
3. WiFi 扫描也是同步执行，扫描期间 WebServer 响应会暂停。
4. 当前没有鉴权机制；同一网络内访问设备 IP 的客户端可以扫描 BLE、修改目标手柄地址和重新配置 WiFi。
5. 配置热点密码固定为 `12345678`，便于调试，但不是长期部署的安全配置。
6. 舵机中位校准会短暂关闭再恢复左右腿舵机扭矩，使用前应保证机器人处于安全姿态。
7. `/api/xbox/select` 只校验地址长度为 17，没有校验 `xx:xx:xx:xx:xx:xx` 的十六进制格式。
8. 页面中的 SSID 和 BLE 名称做了 JSON 转义，但 HTML 模板拼接仍比较轻量，未来如果输入源更复杂，最好增加 HTML escape。
9. `esp_http_server::task_entry()` 的参数 `arg` 当前未使用，符合项目里“不写 `(void)arg;`”的风格，但编译选项如果未来打开未使用参数警告，可能需要统一处理。

## 后续整理建议

如果网页继续扩展成 ESP 上位机，可以按下面方向整理：

1. 将 HTML 拆成独立头文件或生成文件，避免 `esp_http_server.cpp` 继续膨胀。
2. 把阻塞式 BLE 扫描改成“启动扫描 + 轮询结果”的异步接口，避免 HTTP 请求挂 4 秒。
3. 增加一个轻量 `/api/system/status`，返回 WiFi 模式、STA IP、AP 是否开启、手柄状态等统一状态。
4. 给配置和控制接口加一个简单 token 或首次配网密码，至少避免同网段随意改配置。
5. 如果后续动作控制也走网页，建议新增 `controller_service` 或 `robot_api` 层，让 HTTP server 只处理协议和页面，不直接理解过多业务动作。
