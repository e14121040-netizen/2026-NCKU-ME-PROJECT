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
   +--------+----------------+----------------+
   |                         |                |
   v                         v                v
C3 #1 腿部        C3 #2 旋轉端 RotatingArm  C3 #3 固定端 FixedStage
                  r / θ / 夾爪 / z          spin / Gate / 暫存盒
```

## 目錄說明

| 目錄 | 控制板 | 說明 |
|------|--------|------|
| `00_macaddress/` | 任意 ESP32/C3 | 印出 MAC 地址 |
| `01_MainController/` | ESP32 | BLE 主控板，負責命令分發 |
| `02_LegController/` | ESP32-C3 #1 | 腿部馬達控制 |
| `03_RotatingArmController/` | ESP32-C3 #2 | C3 #2 旋轉端：r 齒條、θ、夾爪、z 升降 |
| `04_FixedStageController/` | ESP32-C3 #3 | C3 #3 固定端：spin right/left、暫存盒 Gate |
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
arduino-cli compile --fqbn esp32:esp32:esp32c3 03_RotatingArmController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 04_FixedStageController/
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

### 旋轉端上方機構（C3 #2）

| 命令 | 功能 |
|------|------|
| `EXTEND` | r 齒條伸出 |
| `RETRACT` | r 齒條縮回 |
| `RSTOP` | 只停止 r 齒條 |
| `THETA+` / `i` | θ 正向步進 |
| `THETA-` / `k` | θ 反向步進 |
| `OPEN` | 夾爪張開 |
| `CLOSE` | 夾爪閉合 |
| `UP` | z 上升 |
| `DOWN` | z 下降 |
| `HOME` | r 縮回、θ 回中、夾爪張開，並請固定端 Gate 關閉 |
| `ARMSTOP` | 停止 C3 #2 旋轉端 |
| `TZSTOP` | 舊名相容，等同 `ARMSTOP` |
| `SERVOSTOP` | 舊名相容，等同 `ARMSTOP` |

### 固定端暫存盒 / spin（C3 #3）

| 命令 | 功能 |
|------|------|
| `TL` | 固定端 spin left |
| `TR` | 固定端 spin right |
| `GATEOPEN` | 暫存盒 Gate 打開 |
| `GATECLOSE` | 暫存盒 Gate 關閉 |
| `FIXEDSTOP` | 停止 C3 #3 固定端 |

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
| 固定端 spin / Gate | `a/d/g/n` |
| 旋轉端上方機構 | `w/s/x/i/k/o/p/u/j/h` |
| 分路停止 | `1=LEGSTOP`、`2=ARMSTOP`、`3=FIXEDSTOP` |
| 全域停止 | `0` |

## ACK 現況

- 共用 ACK 格式定義於 `protocol.h` 的 `ack_message`
- 目前只有 **C3 #2 (RotatingArmController)** 有回 ACK
- 主控板會將 ACK 解析成：
  - `RotatingArm ACK cmd=... spd=... status=...`
- C3 #1 與 C3 #3 目前沒有 ACK 回傳，這是預期行為

## protocol.h 使用規則

- `protocol.h` 是唯一協議來源
- `01_MainController.ino`
- `02_LegController.ino`
- `03_RotatingArmController.ino`
- `04_FixedStageController.ino`

以上四個 sketch 都必須直接 `#include "../protocol.h"`，不要再各自複製 enum / struct。

## 初次驗證建議

1. 先讓每塊子控板獨立上電，用 Serial Monitor 驗證本機命令
2. 再上傳主控板，確認 peer 加入結果
3. 最後再用 **BLE Controller – Arduino ESP32** 測試：
   - `FORWARD` → `LEGSTOP`
   - `UP` → `ARMSTOP`
   - `EXTEND` → `RSTOP`
   - `TL` → `FIXEDSTOP`
   - `GATEOPEN` → `GATECLOSE`
   - `STOP`

## 供電與配線提醒

- C3 #2 放旋轉端，上方 servo 與 Z 馬達控制線不要跨旋轉關節。
- C3 #3 放固定端，spin right/left 與暫存盒 Gate 不跟上方線束綁在一起。
- 有限角度（例如 +/-90 deg 或 +/-180 deg）時，跨旋轉處的供電線走旋轉中心並預留鬆弛線圈即可，不必使用 slip ring。
- Servo V+ 走 XL4015，不能從 C3 直接取電。
- C3 走 LM2596 5V；腿部 C3 #1 使用腿部電池組的 LM2596-Leg，C3 #2/#3 使用其他機構電池組的 LM2596-Other。
- BTS7960 的 `VCC`、`R_EN`、`L_EN` 都必須接 3.3V。
- 各電源組內必須共地；腿部組與其他機構組透過 ESP-NOW 無線通訊，正式架構下不跨組共地。
