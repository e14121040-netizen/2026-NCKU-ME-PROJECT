# ESP32 控制程式

> 取物機器人分散式控制系統的 ESP32 / ESP32-C3 韌體。  
> 正式手機控制端： **BLE Controller – Arduino ESP32**

## 系統架構

```text
BLE Controller – Arduino ESP32
            |
            | BLE
            v
ESP32 主控板 (01_MainController)
            |
            | ESP-NOW
   +--------+-----------+
   |                    |
   v                    v
C3 #1 腿部         C3 #2 大圓盤 + z
                        |
                        v
                  C3 #3 Servo / 夾爪
```

## 目錄說明

| 目錄 | 控制板 | 說明 |
|------|--------|------|
| `00_macaddress/` | 任意 ESP32/C3 | 印出 MAC 地址 |
| `01_MainController/` | ESP32 | BLE 主控板，負責命令分發 |
| `02_LegController/` | ESP32-C3 #1 | 腿部馬達控制 |
| `03_TurntableZController/` | ESP32-C3 #2 | 大圓盤旋轉與 z 升降 |
| `04_ServoClawController/` | ESP32-C3 #3 | r 齒條、θ、夾爪、擋板 |
| `protocol.h` | 共用 | 單一協議來源 |
| `_test/` | 測試 | 舊原型與局部測試程式 |

## 上傳流程

```bash
# 1. 先取得各 C3 的 MAC
arduino-cli compile --fqbn esp32:esp32:esp32c3 00_macaddress/
arduino-cli upload  --fqbn esp32:esp32:esp32c3 -p /dev/cu.usbmodem* 00_macaddress/

# 2. 把 MAC 填進 01_MainController.ino

# 3. 上傳主控板
# BLE + ESP-NOW 主控程式需使用 Huge APP partition
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app 01_MainController/

# 4. 上傳三塊子控板
arduino-cli compile --fqbn esp32:esp32:esp32c3 02_LegController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 03_TurntableZController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 04_ServoClawController/
```

## MAC 設定注意

- `01_MainController.ino` 中若仍是 `FF:FF:FF:FF:FF:FF`，代表該 peer 尚未配置完成。
- 主控板現在會：
  - 跳過 placeholder peer 的 `esp_now_add_peer`
  - 在 Serial / BLE log 中明確提示未配置
  - 不對 placeholder peer 送封包

## ESP32 主控板 Partition 注意

- `01_MainController` 同時使用 BLE、Wi-Fi/ESP-NOW，Arduino ESP32 core 3.3.7 下預設 `Default 4MB with spiffs` 分割區會太小。
- Arduino IDE 上傳主控板時，`Partition Scheme` 請選 `Huge APP (3MB No OTA/1MB SPIFFS)`。
- `arduino-cli` 請使用 `esp32:esp32:esp32:PartitionScheme=huge_app`。

## 正式 BLE 文字命令

### 腿部（C3 #1）

| 命令 | 功能 |
|------|------|
| `FORWARD` | 前進 |
| `BACKWARD` | 後退 |
| `LEFT` | 原地左旋 |
| `RIGHT` | 原地右旋 |
| `LEGSTOP` | 只停止腿部 |

### 大圓盤 + z（C3 #2）

| 命令 | 功能 |
|------|------|
| `TL` | 大圓盤左轉 |
| `TR` | 大圓盤右轉 |
| `UP` | z 上升 |
| `DOWN` | z 下降 |
| `TZSTOP` | 只停止 C3 #2 |

### Servo / 夾爪（C3 #3）

| 命令 | 功能 |
|------|------|
| `EXTEND` | r 齒條伸出 |
| `RETRACT` | r 齒條縮回 |
| `RSTOP` | 只停止 r 齒條 |
| `OPEN` | 夾爪張開 |
| `CLOSE` | 夾爪閉合 |
| `HOME` | 歸位 |
| `GATEOPEN` | 承物盒擋板打開 |
| `GATECLOSE` | 承物盒擋板關閉 |
| `SERVOSTOP` | 只停止 C3 #3 的持續動作 |

### 全域命令

| 命令 | 功能 |
|------|------|
| `STOP` | 全域急停 |
| `SPD:xxx` | 設定腿部預設速度 |

> `SPD:xxx` 建議初測使用 120~150。主控板會把過低值夾到可動範圍；確認腿部馬達溫度與電流正常後再上調。

## 單字元相容命令

> 這些保留給 Serial Monitor 與舊配置除錯，不是正式手機控制主路徑。

| 類別 | 命令 |
|------|------|
| 腿部 | `f/b/l/r/q/e/F/B/L/R` |
| 大圓盤 + z | `a/d/u/j` |
| Servo | `w/s/i/k/o/p/g/n/h` |
| 分路停止 | `1/2/3` |
| 全域停止 | `0` |

## ACK 現況

- 共用 ACK 格式定義於 `protocol.h` 的 `ack_message`
- 目前只有 **C3 #2 (TurntableZController)** 有回 ACK
- 主控板會將 ACK 解析成：
  - `TurntableZ ACK cmd=... spd=... status=...`
- C3 #1 與 C3 #3 目前沒有 ACK 回傳，這是預期行為

## protocol.h 使用規則

- `protocol.h` 是唯一協議來源
- `01_MainController.ino`
- `02_LegController.ino`
- `03_TurntableZController.ino`
- `04_ServoClawController.ino`

以上四個 sketch 都必須直接 `#include "../protocol.h"`，不要再各自複製 enum / struct。

## 初次驗證建議

1. 先讓每塊子控板獨立上電，用 Serial Monitor 驗證本機命令
2. 再上傳主控板，確認 peer 加入結果
3. 最後再用 **BLE Controller – Arduino ESP32** 測試：
   - `FORWARD` → `LEGSTOP`
   - `TL` → `TZSTOP`
   - `EXTEND` → `RSTOP`
   - `STOP`

## 供電提醒

- Servo V+ 走 XL4015，不能從 C3 直接取電
- C3 走 LM2596 5V；腿部 C3 #1 使用腿部電池組的 LM2596-Leg，C3 #2/#3 使用其他機構電池組的 LM2596-Other
- BTS7960 的 `R_EN / L_EN` 都必須接 3.3V
- 各電源組內必須共地；腿部組與其他機構組透過 ESP-NOW 無線通訊，正式架構下不跨組共地
