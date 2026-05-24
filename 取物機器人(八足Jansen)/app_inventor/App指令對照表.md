# 遙控指令對照表

## 正式控制端

> 本組正式使用 **BLE Controller – Arduino ESP32** 作為手機控制端。  
> BLE 裝置名稱：`ESP32_MainController`  
> ESP32 主控板透過 BLE 接收手機命令，再經由 ESP-NOW 分發到三塊子控板。

## 推薦按鈕映射

### 腿部控制（→ C3 #1）

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Forward | `FORWARD` | 前進 |
| Backward | `BACKWARD` | 後退 |
| Left Spin | `LEFT` | 原地左旋 |
| Right Spin | `RIGHT` | 原地右旋 |
| Leg Stop | `LEGSTOP` | 只停止腿部 |

> `LEFT` / `RIGHT` 為原地旋轉，不使用差速轉向。  
> 如需調整腿部速度，可在進階按鈕加入 `SPD:200` 這類命令。

### 大圓盤 + z 控制（→ C3 #2）

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Turntable Left | `TL` | 大圓盤左轉 |
| Turntable Right | `TR` | 大圓盤右轉 |
| Z Up | `UP` | z 上升 |
| Z Down | `DOWN` | z 下降 |
| TZ Stop | `TZSTOP` | 只停止 C3 #2 |

### Servo / 夾爪控制（→ C3 #3）

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Extend | `EXTEND` | r 齒條伸出 |
| Retract | `RETRACT` | r 齒條縮回 |
| Theta+ | `I` 或 `i` | θ 正向步進 |
| Theta- | `K` 或 `k` | θ 反向步進 |
| Claw Open | `OPEN` | 夾爪張開 |
| Claw Close | `CLOSE` | 夾爪閉合 |
| Gate Open | `GATEOPEN` | 承物盒擋板打開 |
| Gate Close | `GATECLOSE` | 承物盒擋板關閉 |
| Home | `HOME` | 歸位 |
| Servo Stop | `SERVOSTOP` | 只停止 C3 #3 的持續動作 |

### 全域急停

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Emergency Stop | `STOP` | 同時停止腿部、Turntable/Z、Servo |

## 按鈕配置建議

1. 每個持續動作區域都放自己的停止鍵：
   - 腿部區放 `LEGSTOP`
   - 大圓盤 / z 區放 `TZSTOP`
   - Servo 區放 `SERVOSTOP`
2. 另外保留一顆大顆紅色按鈕送 `STOP`，作為全域急停。
3. `SPD:xxx` 建議放到進階頁或小按鈕列，不和主控制區混在一起。

## 舊版相容指令

> 主控板仍保留單字元命令，供 Serial Monitor 或舊配置除錯使用：
> - 腿部：`f/b/l/r/q/e/F/B/L/R`
> - 大圓盤 + z：`a/d/u/j`
> - Servo：`w/s/i/k/o/p/g/n/h`
> - 分路停止：`1/2/3`
> - 全域停止：`0`

正式手機控制文件請以 **BLE Controller – Arduino ESP32 的文字命令** 為準，不再以單字元按鍵映射作為主流程。

## 相關文件

- BLE Controller 實作流程： [BLE_Controller控制指南.md](BLE_Controller控制指南.md)
- 舊 App Inventor 參考： [AppInventor開發指南.md](AppInventor開發指南.md)
- 主控板與子控板命令總覽： [../../esp32控制/README.md](../../esp32控制/README.md)
