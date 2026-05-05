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
| `02_LegController/` | ESP32-C3 #1 | **腿部**：L298N #1 驅動 JGB37-520 ×2（差速轉向） |
| `03_TurntableZController/` | ESP32-C3 #2 | **大圓盤+z**：L298N #2 驅動 XD-25GA 370 + JGY370 + 限位開關 |
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
- **L298N ENA/ENB 跳線帽**：必須拔除，否則 PWM 調速無效
- **ESP32Servo 函式庫**：`04_ServoClawController` 需安裝 `ESP32Servo` 函式庫
- **共用協議**：`protocol.h` 定義所有通訊的 enum 和 struct，修改時需同步更新

## FQBN 對照

| 控制板 | FQBN | Baud Rate |
|--------|------|-----------|
| ESP32 主控板 | `esp32:esp32:esp32` | 115200 |
| ESP32-C3 子控板 | `esp32:esp32:esp32c3` | 115200 |
