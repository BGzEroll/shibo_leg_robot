# String 使用风险分析

## 结论

当前代码里的 `String` 使用总体风险可控，主要集中在 HTTP 配置服务、WiFi 凭据保存、Xbox 蓝牙地址/扫描结果这几块。控制核心、姿态环、电机驱动、UART 通信热路径里没有发现持续构造和拼接 `String` 的情况，因此短期内不太像会直接影响实时控制。

需要留意的是堆内存碎片和峰值内存：HTTP 返回 JSON 时会多次 `+=` 拼接，BLE 扫描结果结构体里保存了多个 `String`，这些都发生在配置/扫描路径中，频率不高，但如果长期打开网页频繁刷新、反复扫描 BLE，仍可能增加堆碎片风险。

## 出现位置

### `lib/services/esp_http_server.cpp`

- `json_escape()` 返回新的 `String`，内部已 `reserve(value.length() + 8)`，风险较低。
- `console_html()`、`wifi_html()`、`bluetooth_html()`、`servo_calibration_html()` 返回整页 HTML 字符串，只在页面请求时构造，不在控制热路径中。
- `handle_wifi_scan()`、`handle_ble_scan()` 使用 `String json` 和多次 `+=` 拼接 JSON，是本次最需要关注的 `String` 使用点。
- `handle_xbox_status()` 每 2 秒会被蓝牙页面轮询一次，如果页面长期开着，会持续分配小字符串。
- `handle_connect()`、`handle_xbox_select()` 从 `server.arg()` 获取表单参数，这属于 WebServer API 边界，风险较低。

### `lib/devices/wifi_dev.cpp`

- `load_credentials()`、`save_credentials()` 使用 `Preferences::getString()` / `putString()`，只在启动和配网连接时发生。
- `connect_station()` 把 `String` 转为 `c_str()` 传给 `WiFi.begin()`，函数调用期间对象有效，没有明显悬空指针风险。
- `init()` 中的 `String ssid` / `password` 是启动路径的局部变量，频率很低。

### `lib/devices/xbox_dev.cpp` / `lib/devices/xbox_dev.h`

- `current_target_address` 是静态全局 `String`，只在初始化和选择新手柄地址时变化，风险低。
- `normalize_address()` 会复制、trim、转小写，输入很短，风险低。
- `scan_ble()` 将扫描结果的 `address` 和 `name` 写入 `ble_device` 里的 `String` 字段。这里最多 `BLE_SCAN_MAX = 24` 个结果，单次扫描会产生一批动态分配，是 BLE 扫描路径的主要堆碎片风险来源。
- `target_address()` 返回 `String` 副本，HTTP 状态查询时会产生一次拷贝，但频率低。

## 风险判断

### 低风险

- WiFi SSID/password 存取：发生在启动或手动配网时，字符串长度有限。
- Xbox 目标地址：固定 17 字符左右，变化次数很少。
- HTML 页面返回：配置网页请求时才构造，不参与控制闭环。

### 中等风险

- WiFi 扫描 JSON 拼接：热点数量多时，多次 `json +=` 可能导致扩容和堆碎片。
- BLE 扫描 JSON 拼接：设备数量最多 24 个，同时每个设备结构体还有 `String address/name`。
- 蓝牙状态轮询：网页每 2 秒请求一次 `/api/xbox/status`，长期开页面会重复创建短 `String`。

### 当前未发现的高风险

- 没有看到在 `balance_core`、`controller`、电机驱动、Xbox 输入采样 20ms 循环里反复拼接 `String`。
- 没有看到串口解析、控制包解析里使用 `String`。
- 没有看到把临时 `String.c_str()` 指针保存到异步长期对象里的明显问题。

## 建议

1. 暂时不需要大规模替换所有 `String`。
2. 如果要优化，优先处理 `handle_wifi_scan()` 和 `handle_ble_scan()` 的 JSON 拼接：
   - 给 `json` 预估容量并 `reserve()`。
   - 或改为分段 `server.sendContent()` 输出，避免构造完整大字符串。
3. `xbox_dev::ble_device` 可以后续考虑把 `address` 改成固定 `char address[18]`，`name` 改成定长缓冲区，例如 32 或 48 字节；这能减少扫描路径的堆分配。
4. 如果你要验证实际风险，建议加一次临时堆监控，只在 HTTP/BLE 配置路径打印：
   - 扫描前后 `ESP.getFreeHeap()`。
   - 扫描前后 `ESP.getMaxAllocHeap()`。
   - 连续扫描 20 次后再看最大可分配块是否明显下降。

## 优先级

第一优先级：`handle_wifi_scan()`、`handle_ble_scan()` 的 `json.reserve()`。

第二优先级：`ble_device` 里的 `String address/name` 定长化。

第三优先级：HTML 大字符串改为 `server.send_P()` 或静态常量页。这个对实时控制影响不大，只有在内存压力确实出现时再做。
