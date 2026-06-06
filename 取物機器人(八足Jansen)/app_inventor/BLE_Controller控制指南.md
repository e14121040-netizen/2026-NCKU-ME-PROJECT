# BLE Controller 控制指南

> 本組正式手機控制端： **BLE Controller – Arduino ESP32**  
> BLE 裝置名稱： `ESP32_MainController`

## 連線步驟

1. 手機安裝 **BLE Controller – Arduino ESP32**
2. 啟動 ESP32 主控板，確認主控板已開啟 BLE 廣播
3. 在 App 中搜尋裝置 `ESP32_MainController`
4. 連線成功後，建立自訂按鈕頁
5. 依照下方指令表填入每顆按鈕要送出的文字命令

## 推薦頁面配置

### 腿部區（C3 #1）

| 按鈕名稱 | 發送命令 |
|---------|---------|
| Forward | `FORWARD` |
| Backward | `BACKWARD` |
| Left Spin | `LEFT` |
| Right Spin | `RIGHT` |
| Leg Stop | `LEGSTOP` |

### 大圓盤 / z 區（C3 #2）

| 按鈕名稱 | 發送命令 |
|---------|---------|
| Turntable Left | `TL` |
| Turntable Right | `TR` |
| Z Up | `UP` |
| Z Down | `DOWN` |
| TZ Stop | `TZSTOP` |

### Servo / 夾爪區（C3 #3）

| 按鈕名稱 | 發送命令 |
|---------|---------|
| Extend | `EXTEND` |
| Retract | `RETRACT` |
| R Stop | `RSTOP` |
| Theta+ | `i` |
| Theta- | `k` |
| Claw Open | `OPEN` |
| Claw Close | `CLOSE` |
| Gate Open | `GATEOPEN` |
| Gate Close | `GATECLOSE` |
| Home | `HOME` |
| Servo Stop | `SERVOSTOP` |

### 全域急停

| 按鈕名稱 | 發送命令 |
|---------|---------|
| Emergency Stop | `STOP` |

## 使用原則

1. 每個區域都要有自己的停止鍵：
   - `LEGSTOP`
   - `TZSTOP`
   - `RSTOP`
   - `SERVOSTOP`
2. 全域急停 `STOP` 必須做成最大顆、最容易按到的紅色按鈕。
3. `STOP` 只用於緊急狀況或整體結束，不拿來取代各區停止鍵。
4. 腿部預設速度可額外做進階按鈕，4S 直供 12V 馬達時先用保守值：
   - `SPD:120`
   - `SPD:150`
   - `SPD:180`

## 建議視覺布局

```text
┌──────────────────────────────────────────────────────┐
│ [Forward] [Backward] [Left Spin] [Right Spin]       │
│ [Leg Stop]                                          │
│                                                      │
│ [Turntable Left] [Turntable Right] [Z Up] [Z Down]  │
│ [TZ Stop]                                           │
│                                                      │
│ [Extend] [Retract] [Theta+] [Theta-]                │
│ [Open] [Close] [Gate Open] [Gate Close] [Home]      │
│ [Servo Stop]                                        │
│                                                      │
│                 [ EMERGENCY STOP ]                  │
└──────────────────────────────────────────────────────┘
```

## 驗證順序

1. 先測 `FORWARD` / `LEGSTOP`
2. 再測 `TL` / `TZSTOP`
3. 再測 `EXTEND` / `SERVOSTOP`
4. 最後測 `STOP` 是否能同時停掉三路

## 常見問題

| 問題 | 檢查方式 |
|------|----------|
| 找不到 `ESP32_MainController` | 確認主控板已上電、BLE 已初始化 |
| 有連上但 C3 #2 / #3 沒反應 | 先檢查 `01_MainController.ino` 內 MAC 是否仍是 `FF:FF:FF:FF:FF:FF` |
| 按下按鈕後持續動作 | 這是現行設計，請按對應區域的 Stop 鍵停止 |
| `STOP` 沒有效果 | 檢查主控板 Serial 是否有收到 `STOP` |
