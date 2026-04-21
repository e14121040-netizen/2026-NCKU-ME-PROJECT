# ESP32 控制程式

> 本目錄包含取物機器人所有 ESP32 / ESP32-C3 的 Arduino 程式碼。  
> 採用 **分散式架構**：1 塊 ESP32 主控板 + 3 塊 ESP32-C3 子控板，透過 ESP-NOW 通訊。

## 系統架構

```
手機 App ──BLE/BT──→ ESP32 主控板 (MainController)
                       │
           ┌───────────┼───────────┐
           │           │           │
      ESP-NOW     ESP-NOW     ESP-NOW
           │           │           │
           ▼           ▼           ▼
      C3 #1 腿部  C3 #2 手臂  C3 #3 z/夾爪
   (LegController) (noah_夾爪控制測試) (ZClawController)
```

## 目錄說明

| 目錄 | 控制板 | 說明 | 狀態 |
|------|--------|------|------|
| `MainController/` | ESP32 | 主控板（BluetoothSerial 版）：藍芽接收 + ESP-NOW 指令分發 | ✅ 可用 |
| `MainController_BLE/` | ESP32 | 主控板（BLE 版）：BLE UART Service + ESP-NOW 指令分發，支援字串指令和 ACK | ✅ 可用 |
| `LegController/` | ESP32-C3 #1 | 腿部子控板：L298N 驅動步行馬達 ×2（差速轉向） | ✅ 可用 |
| `noah_夾爪控制測試/` | ESP32-C3 #2 | 手臂 r/θ 子控板：L298N 驅動手臂馬達 ×2 + 限位開關 ×4 | ✅ 可用 |
| `ZClawController/` | ESP32-C3 #3 | z/夾爪子控板：L298N 驅動 z 升降馬達 + 伺服馬達（夾爪+蓋板）+ 限位開關 ×2 | ✅ 可用 |
| `macaddress/` | 任意 ESP32/C3 | 工具程式：印出 MAC 地址（設定 ESP-NOW 時使用） | 🔧 工具 |
| `Leg_motor_prototype/` | — | 腿部馬達早期原型（含藍芽，已重構至 LegController） | 📦 參考 |
| `Leg_motor_prototype_button_control/` | — | 腿部馬達按鈕控制原型 | 📦 參考 |
| `esp32test/` | — | ESP32 基礎測試 | 📦 參考 |

### 主控板版本比較

| 比較項目 | `MainController` (BluetoothSerial) | `MainController_BLE` (BLE UART) |
|----------|------------------------------------|---------------------------------|
| 藍芽類型 | Classic Bluetooth | BLE 低功耗藍芽 |
| 裝置名稱 | `PickupRobot` | `ESP32_MainController` |
| App 相容 | Bluetooth Controller App 等 | BLE UART App（如 nRF Connect） |
| 指令格式 | 單字元 | 單字元 + 字串指令 + 速度設定 |
| ACK 回傳 | ❌ 無 | ✅ 有（子控板回傳確認） |
| 斷線急停 | ❌ 無 | ✅ 有 |
| 建議場景 | 簡易遙控 / 快速測試 | 進階控制 / 需要回饋的場景 |

## 快速開始

### 1. 取得各子控板 MAC 地址

將 `macaddress/macaddress.ino` 上傳至每塊 ESP32-C3，開啟 Serial Monitor (115200 baud) 記錄 MAC 地址。

### 2. 填入 MAC 地址

編輯 `MainController/MainController.ino`（或 `MainController_BLE/MainController_BLE.ino`），將 3 組 MAC 地址填入：

```cpp
uint8_t Leg_Address[]     = {0x??, 0x??, 0x??, 0x??, 0x??, 0x??};  // C3 #1
uint8_t r_theta_Address[] = {0x??, 0x??, 0x??, 0x??, 0x??, 0x??};  // C3 #2
uint8_t z_clap_Address[]  = {0x??, 0x??, 0x??, 0x??, 0x??, 0x??};  // C3 #3
```

### 3. 上傳程式（順序建議）

1. **先上傳子控板** — 讓它們進入等待接收狀態
   - `LegController.ino` → C3 #1
   - `noah_夾爪控制測試.ino` → C3 #2
   - `ZClawController.ino` → C3 #3
2. **再上傳主控板** — `MainController.ino`（或 `MainController_BLE.ino`）→ ESP32

### 4. 測試

- **Serial 測試**：在任一板子的 Serial Monitor 手動輸入指令測試
- **藍芽測試**：
  - BluetoothSerial 版：手機搜尋 `PickupRobot`，連線後發送指令
  - BLE 版：手機搜尋 `ESP32_MainController`，連線 UART Service 後發送指令

## 指令速查

### 腿部指令（→ C3 #1）

| 指令 | 功能 | 指令 | 功能 |
|------|------|------|------|
| `f` | 前進 | `F` | 半速前進 |
| `b` | 後退 | `B` | 半速後退 |
| `l` | 左轉 | `L` | 半速左轉 |
| `r` | 右轉 | `R` | 半速右轉 |
| `q` | 左旋 | `e` | 右旋 |

### 手臂 r/θ 指令（→ C3 #2）

| 指令 | 功能 |
|------|------|
| `w` | 手臂伸出 (r+) |
| `s` | 手臂縮回 (r-) |
| `a` | 手臂左轉 (θ-) |
| `d` | 手臂右轉 (θ+) |

### z/夾爪指令（→ C3 #3）

| 指令 | 功能 |
|------|------|
| `u` | z 上升 |
| `j` | z 下降 |
| `o` | 夾爪張開 (→ 180°) |
| `p` | 夾爪閉合 (→ 0°) |
| `t` | 蓋板開 (→ 90°) |
| `y` | 蓋板關 (→ 0°) |
| `h` | 歸位（z 下降到底 + 夾爪張開 + 蓋板關） |

### 子控板本機測試（Serial Monitor 額外指令）

| 指令 | 功能 | 適用控制板 |
|------|------|-----------|
| `+` | 加速 | 全部 |
| `-` | 減速 | 全部 |
| `?` | 顯示狀態 | 全部 |
| `1`~`9` | 夾爪角度 (20°~180°) | C3 #3 |

### 通用

| 指令 | 功能 |
|------|------|
| `0` | 全部停止 |

### BLE 版專用字串指令

> 僅限 `MainController_BLE` 版本

| 指令 | 功能 | 指令 | 功能 |
|------|------|------|------|
| `FORWARD` | 前進 | `BACKWARD` | 後退 |
| `LEFT` | 左轉 | `RIGHT` | 右轉 |
| `SPINL` / `QL` | 左旋 | `SPINR` / `ER` | 右旋 |
| `EXTEND` | 手臂伸出 | `RETRACT` | 手臂縮回 |
| `ARMLEFT` / `AL` | 手臂左轉 | `ARMRIGHT` / `AR` | 手臂右轉 |
| `UP` / `ZU` | z 上升 | `DOWN` / `ZD` | z 下降 |
| `OPEN` | 夾爪張開 | `CLOSE` | 夾爪閉合 |
| `TILT` / `LO` | 蓋板開 | `FLAT` / `LC` | 蓋板關 |
| `HOME` | 歸位 | `STOP` | 全部停止 |
| `SPD:200` | 設定速度 | | |

## 編譯與上傳

### Windows（Arduino IDE）

1. **安裝 Arduino IDE**
   - 從 [arduino.cc](https://www.arduino.cc/en/software) 下載安裝

2. **安裝 ESP32 開發板套件**
   - 檔案 → 偏好設定 → 額外的開發板管理員網址，加入：
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - 工具 → 開發板 → 開發板管理員 → 搜尋 `ESP32` → 安裝 **Espressif Systems** 的 ESP32

3. **安裝需要的函式庫**
   - 工具 → 管理函式庫 → 搜尋安裝 `ESP32Servo`（C3 #3 伺服控制用）
   - `WiFi`、`esp_now`、`BluetoothSerial`、`BLE` 已內建，不需另外安裝

4. **選擇開發板與 Port**
   - 主控板：工具 → 開發板 → `ESP32 Dev Module`
   - 子控板：工具 → 開發板 → `ESP32C3 Dev Module`
   - 工具 → 序列埠 → 選擇對應的 COM Port（如 `COM3`）

5. **上傳**
   - 開啟 `.ino` 檔案 → 點擊上傳按鈕（→）
   - 如果上傳失敗，嘗試在上傳時按住板子上的 **BOOT** 鍵

6. **Serial Monitor**
   - 工具 → 序列埠監控窗 → Baud Rate 設為 `115200`

### macOS（arduino-cli）

1. **確認 arduino-cli 已安裝**
   ```bash
   arduino-cli version
   ```

2. **安裝 ESP32 開發板套件**
   ```bash
   arduino-cli core install esp32:esp32
   ```

3. **安裝需要的函式庫**
   ```bash
   arduino-cli lib install ESP32Servo
   ```

4. **查看連接的板子**
   ```bash
   arduino-cli board list
   ```

5. **編譯與上傳**
   ```bash
   # ESP32 主控板（BluetoothSerial 版）
   arduino-cli compile --fqbn esp32:esp32:esp32 MainController/
   arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX MainController/

   # ESP32 主控板（BLE 版）
   arduino-cli compile --fqbn esp32:esp32:esp32 MainController_BLE/
   arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX MainController_BLE/

   # ESP32-C3 子控板
   arduino-cli compile --fqbn esp32:esp32:esp32c3 LegController/
   arduino-cli upload --fqbn esp32:esp32:esp32c3 -p /dev/cu.usbmodem-XXXX LegController/

   arduino-cli compile --fqbn esp32:esp32:esp32c3 ZClawController/
   arduino-cli upload --fqbn esp32:esp32:esp32c3 -p /dev/cu.usbmodem-XXXX ZClawController/
   ```

### 共通注意事項

- **FQBN 對照**：主控板 = `esp32:esp32:esp32`、子控板 = `esp32:esp32:esp32c3`
- **Baud Rate**：全部使用 `115200`
- **L298N ENA/ENB 跳線帽**：上傳程式前確認已拔除，否則 PWM 調速無效
- **伺服馬達供電**：伺服馬達接 5V 穩壓電源（經 LM2596 降壓），不可從 C3 的 3.3V pin 取電
- **ESP32Servo 函式庫**：`ZClawController` 需要安裝 `ESP32Servo` 函式庫
