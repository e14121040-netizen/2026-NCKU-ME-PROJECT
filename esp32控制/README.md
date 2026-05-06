# ESP32 控制程式

> 取物機器人分散式控制系統的所有 ESP32 / ESP32-C3 程式碼。  
> 最後更新：2026/05/05

## 系統架構

```
手機 App ──BLE──→ ESP32 主控板 (01_MainController)
                       │        ── USB 插電腦供電
           ┌───────────┼───────────┐
           │           │           │
      ESP-NOW     ESP-NOW     ESP-NOW
           │           │           │
           ▼           ▼           ▼
      C3 #1 腿部  C3 #2 大圓盤+z  C3 #3 Servo
  (02_LegController) (03_TurntableZ) (04_ServoClaw)
```

## 目錄說明

| 目錄 | 控制板 | 說明 |
|------|--------|------|
| `00_macaddress/` | 任意 ESP32/C3 | 工具：印出 MAC 地址（設定 ESP-NOW 時使用） |
| `01_MainController/` | ESP32 | **主控板**：BLE UART + ESP-NOW 指令分發（USB 插電腦供電） |
| `02_LegController/` | ESP32-C3 #1 | **腿部**：BTS7960 ×2 驅動 JGB37-520 ×2（差速轉向） |
| `03_TurntableZController/` | ESP32-C3 #2 | **大圓盤+z**：BTS7960 #3 驅動 XD-25GA 370 + BTS7960 #4 驅動 JGY370 + 限位開關 |
| `04_ServoClawController/` | ESP32-C3 #3 | **Servo**：MG996R 360°(r齒條) + 180°(θ) + 180°(夾爪) + 限位開關 |
| `protocol.h` | — | 共用通訊協議（enum + struct 定義） |
| `_test/` | — | 測試用程式（按鈕控制原型、Servo 測試、ESP32 基礎測試） |

## 上傳流程

```bash
# 1. 取得各 C3 的 MAC 地址
arduino-cli compile --fqbn esp32:esp32:esp32c3 00_macaddress/
arduino-cli upload  --fqbn esp32:esp32:esp32c3 -p /dev/cu.usbmodem* 00_macaddress/

# 2. 將 MAC 填入 01_MainController.ino 的 turntableZ_Address / servoClaw_Address

# 3. 上傳主控板（ESP32）
arduino-cli compile --fqbn esp32:esp32:esp32 01_MainController/
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial* 01_MainController/

# 4. 上傳子控板（ESP32-C3）
arduino-cli compile --fqbn esp32:esp32:esp32c3 02_LegController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 03_TurntableZController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 04_ServoClawController/
```

> ⚠ ESP32-C3 Super Mini 上傳時需確認 Arduino IDE 設定：`USB CDC On Boot: Enabled`

## 指令速查

### 腿部指令（→ C3 #1）

| 指令 | 功能 | 指令 | 功能 |
|------|------|------|------|
| `f` | 前進 | `F` | 半速前進 |
| `b` | 後退 | `B` | 半速後退 |
| `l` | 左轉 | `L` | 半速左轉 |
| `r` | 右轉 | `R` | 半速右轉 |
| `q` | 左旋 | `e` | 右旋 |

### 大圓盤 + z 指令（→ C3 #2）

| 指令 | 功能 |
|------|------|
| `a` | 大圓盤左轉 |
| `d` | 大圓盤右轉 |
| `u` | z 上升 |
| `j` | z 下降 |

### Servo / 夾爪指令（→ C3 #3）

| 指令 | 功能 |
|------|------|
| `w` | r 齒條伸出 |
| `s` | r 齒條縮回 |
| `i` | θ 旋轉（正） |
| `k` | θ 旋轉（反） |
| `o` | 夾爪張開 |
| `p` | 夾爪閉合 |
| `h` | 歸位 |

### 全域指令

| 指令 | 功能 |
|------|------|
| `0` | **全部停止**（三路同時發送） |

### BLE 字串指令（主控板額外支援）

| 指令 | 功能 |
|------|------|
| `FORWARD` / `BACKWARD` / `LEFT` / `RIGHT` | 腿部控制 |
| `STOP` | 全部停止 |
| `SPD:xxx` | 設定速度 (0~255) |
| `EXTEND` / `RETRACT` | r 齒條伸出/縮回 |
| `OPEN` / `CLOSE` | 夾爪張開/閉合 |
| `HOME` | 歸位 |
| `UP` / `DOWN` | z 升降 |
| `TL` / `TR` | 大圓盤左/右轉 |

### 子控板本機測試（Serial Monitor）

各子控板可透過 Serial Monitor 直接輸入指令測試，不需主控板。額外支援：

| 指令 | 功能 |
|------|------|
| `+` / `-` | DC 馬達加速/減速（C3 #1, #2） |
| `?` | 顯示目前狀態 |

## 供電注意事項

- **Servo 馬達供電**：接 XL4015 5A 降壓模組 5V 輸出（Servo 專用）
- **MCU 供電**：ESP32-C3 ×3 接 LM2596 3A 降壓模組 5V 輸出（MCU 專用）
- **ESP32 主控板**：USB 插電腦供電，與機器人電源獨立（ESP-NOW 無線通訊）
- **BTS7960 R_EN/L_EN**：接 3.3V 常開（腳部、大圓盤）
- **BTS7960 R_EN/L_EN**：所有 BTS7960 的 R_EN/L_EN 都必須接 3.3V，否則馬達不會轉
- **ESP32Servo 函式庫**：`04_ServoClawController` 需安裝 `ESP32Servo` 函式庫
- **共用協議**：`protocol.h` 定義所有通訊的 enum 和 struct，修改時需同步更新

## FQBN 對照

| 控制板 | FQBN | Baud Rate |
|--------|------|-----------|
| ESP32 主控板 | `esp32:esp32:esp32` | 115200 |
| ESP32-C3 子控板 | `esp32:esp32:esp32c3` | 115200 |

## 開發環境完整設定

### Arduino IDE 設定

1. **安裝 ESP32 Board 支援**
   - 工具 → 開發板管理員 → 搜尋 `ESP32` → 安裝 **Espressif Systems ESP32** (≥ 3.x)
   - 或在「偏好設定」加入額外開發板管理員網址：
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```

2. **安裝必要函式庫**
   - `ESP32Servo`（C3 #3 需要）：素描 → 匯入函式庫 → 管理函式庫 → 搜尋 `ESP32Servo` → 安裝
   - `WiFi`、`esp_now`、`BLEDevice`：ESP32 Board 內建，無需額外安裝

3. **ESP32-C3 Super Mini 特殊設定**
   - 開發板選擇：`ESP32C3 Dev Module`
   - **USB CDC On Boot：Enabled**（必須！否則 Serial 無輸出）
   - Upload Speed：921600
   - Flash Mode：QIO
   - Flash Size：4MB

4. **ESP32 主控板設定**
   - 開發板選擇：`ESP32 Dev Module`
   - Upload Speed：921600

### arduino-cli 設定

```bash
# 安裝 ESP32 核心
arduino-cli core update-index
arduino-cli core install esp32:esp32

# 安裝函式庫
arduino-cli lib install ESP32Servo

# 確認已安裝
arduino-cli board listall | grep esp32
```

**上傳範例**（ESP32-C3）：
```bash
# 編譯
arduino-cli compile \
  --fqbn esp32:esp32:esp32c3 \
  --build-property "build.extra_flags=-DARDUINO_USB_CDC_ON_BOOT=1" \
  02_LegController/

# 上傳（注意 port 名稱可能不同）
arduino-cli upload \
  --fqbn esp32:esp32:esp32c3 \
  -p /dev/cu.usbmodem* \
  02_LegController/

# 開啟 Serial Monitor
arduino-cli monitor -p /dev/cu.usbmodem* --config baudrate=115200
```

> ⚠ ESP32-C3 Super Mini 使用 USB CDC，port 名稱為 `/dev/cu.usbmodem*`（非 `/dev/cu.usbserial*`）。
> 如果找不到 port，嘗試按住 BOOT 鍵再插 USB。

## 各子控板首次測試 SOP

> 在接上 ESP-NOW 之前，先用 Serial Monitor 逐一驗證每塊子控板的基本功能。

### C3 #1 腿部（LegController）測試

**前置**：BTS7960 ×2 已接好、馬達已接好、C3 #1 上傳 `02_LegController.ino`

| 步驟 | Serial 輸入 | 預期結果 |
|------|------------|---------|
| 1 | `f` | 兩馬達正轉（前進），Serial 顯示 `Forward, speed=200` |
| 2 | `0` | 兩馬達停止，Serial 顯示 `=== STOP ===` |
| 3 | `b` | 兩馬達反轉（後退） |
| 4 | `l` | 左馬達慢、右馬達快（左轉） |
| 5 | `r` | 左馬達快、右馬達慢（右轉） |
| 6 | `q` | 左反右正（原地左旋） |
| 7 | `+` | 速度增加 20，Serial 顯示 `Speed UP -> 220` |
| 8 | `-` | 速度減少 20 |
| 9 | `?` | 顯示目前速度 |

**故障排查**：
- 馬達不轉 → 確認 BTS7960 R_EN/L_EN 已接 3.3V，電源 B+/B- 接線正確
- 方向反了 → 交換 BTS7960 M+/M- 的兩條馬達線
- Serial 無輸出 → 確認 **USB CDC On Boot: Enabled**

---

### C3 #2 大圓盤+z（TurntableZController）測試

**前置**：BTS7960 #3/#4 已接好、馬達已接好、限位開關已接好

| 步驟 | Serial 輸入 | 預期結果 |
|------|------------|---------|
| 1 | `a` | 大圓盤左轉，Serial 顯示 `Turntable Left, speed=180` |
| 2 | `d` | 大圓盤右轉 |
| 3 | `u` | z 上升 |
| 4 | `j` | z 下降 |
| 5 | `0` | 全部停止 |
| 6 | `?` | 顯示速度 + 限位開關狀態 |
| 7 | 手壓限位開關 | 輸入 `?` 後對應限位顯示 `TRIGGERED` |
| 8 | 運轉中觸發限位 | 馬達自動停止 + Serial 顯示 `Emergency STOP` |
| 9 | 持續運轉 5 秒 | 超時保護觸發 `TIMEOUT -> Safety STOP` |

**故障排查**：
- 限位開關永遠 `TRIGGERED` → 確認接線為 NO（常開），未觸發時 GPIO 應為 HIGH
- 限位開關永遠 `OK` → 確認 GND 有接好，按壓時觸點確實閉合

---

### C3 #3 Servo（ServoClawController）測試

**前置**：XL4015 5V 已接 Servo V+、Servo 信號線已接 C3 GPIO、限位開關已接好

| 步驟 | Serial 輸入 | 預期結果 |
|------|------------|---------|
| 1 | `w` | 360° Servo 正轉（r 伸出） |
| 2 | `s` | 360° Servo 反轉（r 縮回） |
| 3 | `0` | 360° Servo 停轉 |
| 4 | `i` | θ 角度 +15°，Serial 顯示 `Theta -> 105°` |
| 5 | `k` | θ 角度 -15° |
| 6 | `o` | 夾爪張開 |
| 7 | `p` | 夾爪閉合 |
| 8 | `h` | 歸位（r 縮回 + 夾爪開 + θ 居中） |
| 9 | `?` | 顯示 r 狀態 + θ 角度 + 夾爪角度 + 限位狀態 |

**故障排查**：
- 360° Servo 發 `0` 仍微轉 → 修改 `R_STOP` 常數（±2~5），每顆 Servo 零點不同
- Servo 不動但會嗡嗡聲 → 供電不足，確認 XL4015 輸出 5V、電流充足
- Servo 抖動 → XL4015 輸出端電容是否有裝好、Servo GND 與 C3 GND 是否共地

## ESP-NOW 通訊偵錯

### 配對故障排查流程

```
ESP-NOW 不通？
 │
 ├── 1. 確認 MAC 地址正確
 │     └── 上傳 00_macaddress → 記錄 MAC → 填入 MainController
 │
 ├── 2. 確認 WiFi Channel 一致
 │     └── 主控和子控都設為 ESPNOW_CHANNEL = 1
 │     └── 主控有呼叫 esp_wifi_set_channel()
 │
 ├── 3. 確認 WiFi 模式
 │     └── 主控和子控都設為 WiFi.mode(WIFI_STA)
 │
 ├── 4. 單向測試
 │     └── 先只測主控 → C3 #1（其他兩路先不管）
 │     └── 主控 Serial 輸入 f → 觀察主控輸出 "ESP-NOW -> Leg: OK"
 │     └── 觀察 C3 #1 Serial 輸出 "ESP-NOW Recv -> CMD: 1"
 │
 ├── 5. 確認距離
 │     └── 先在 1 公尺內測試
 │
 └── 6. 確認電源
       └── C3 是否有穩定供電（不是靠 USB 邊上傳邊測）
```

### ACK 回傳驗證

目前只有 C3 #2（TurntableZController）和主控板有實作 ACK：
- C3 #2 收到指令後會自動回傳 `ack_message`
- 主控板 `OnDataRecv` 解析回傳並輸出 `Leg ACK cmd=... spd=... status=...`
- C3 #1 和 C3 #3 目前無 ACK 回傳

### 常見問題

| 問題 | 原因 | 解法 |
|------|------|------|
| `Failed to add peer` | MAC 地址格式錯誤 | 確認 6 byte 陣列格式正確 |
| `ESP-NOW -> Leg: FAIL` | 子控板未開機或 channel 不同 | 確認子控板上電 + channel 設定 |
| 主控 OK 但子控沒收到 | MAC 寫錯或 channel 不匹配 | 重新確認 MAC，兩邊都設 channel=1 |
| 收到但馬達不動 | 指令 enum 不一致 | 確認 enum 值與 protocol.h 一致 |

## protocol.h 使用說明

### 為何各 .ino 重複定義 enum？

`protocol.h` 定義了共用通訊協議（enum + struct），理論上各 `.ino` 應直接 `#include "../protocol.h"`。

但實際上：
- Arduino IDE 的 sketch 目錄結構有限制，`#include` 的相對路徑從 sketch 資料夾起算
- `../protocol.h` 在某些 IDE 版本或 arduino-cli 可能無法正確解析
- 因此各 `.ino` 中**重複定義了 enum 和 struct**，以確保相容性

### 同步更新規則

> ⚠ 修改任何 enum 或 struct 時，必須同步更新以下 5 個檔案：

1. `protocol.h`（主要定義檔）
2. `01_MainController/01_MainController.ino`（L71-126）
3. `02_LegController/02_LegController.ino`（L43-56）
4. `03_TurntableZController/03_TurntableZController.ino`（L63-80）
5. `04_ServoClawController/04_ServoClawController.ino`（L62-76）

建議：修改 `protocol.h` 後，用文字搜尋比對各 `.ino` 中的 enum 值是否一致。

