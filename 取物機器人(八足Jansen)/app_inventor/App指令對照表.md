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

> `LEFT` / `RIGHT` 為原地旋轉，不做左右輪不同速轉彎。  
> 如需調整腿部速度，可在進階按鈕加入 `SPD:150` 這類命令；腿部 4S 直供 12V 馬達時先從 120~150 測試。

### 旋轉端上方機構控制（→ C3 #2 RotatingArm）

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Z Up | `UP` | z 上升 |
| Z Down | `DOWN` | z 下降 |
| Extend | `EXTEND` | r 齒條伸出 |
| Retract | `RETRACT` | r 齒條縮回 |
| R Stop | `RSTOP` | 只停止 r 齒條 |
| Theta+ | `I` 或 `i` | θ 正向步進 |
| Theta- | `K` 或 `k` | θ 反向步進 |
| Claw Open | `OPEN` | 夾爪張開 |
| Claw Close | `CLOSE` | 夾爪閉合 |
| Home | `HOME` | 歸位 |
| Arm Stop | `ARMSTOP` | 停止 C3 #2 旋轉端 |
| TZ Stop | `TZSTOP` | 舊名相容，等同 `ARMSTOP` |
| Servo Stop | `SERVOSTOP` | 舊名相容，停止上方 servo / Z 所在的 C3 #2 |

### 固定端暫存盒 / spin 控制（→ C3 #3 FixedStage）

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Turntable Left | `TL` | 固定端 spin left |
| Turntable Right | `TR` | 固定端 spin right |
| Gate Open | `GATEOPEN` | 承物盒擋板打開 |
| Gate Close | `GATECLOSE` | 承物盒擋板關閉 |
| Fixed Stop | `FIXEDSTOP` | 停止 C3 #3 固定端 |

### 全域急停

| App 按鈕 | 發送文字 | 功能 |
|---------|---------|------|
| Emergency Stop | `STOP` | 同時停止腿部、旋轉端、固定端 |

## 按鈕配置建議

1. 每個持續動作區域都放自己的停止鍵：
   - 腿部區放 `LEGSTOP`
   - 旋轉端區放 `ARMSTOP`；舊 App 可沿用 `TZSTOP` / `SERVOSTOP`
   - 固定端區放 `FIXEDSTOP`
   - r 齒條按住放開送 `RSTOP`
2. 另外保留一顆大顆紅色按鈕送 `STOP`，作為全域急停。
3. `SPD:xxx` 建議放到進階頁或小按鈕列，不和主控制區混在一起。

## 舊版相容指令

> 主控板仍保留單字元命令，供 Serial Monitor 或舊配置除錯使用：
> - 腿部：`f/b/l/r/q/e/F/B/L/R`
> - 固定端 spin / Gate：`a/d/g/n`
> - 旋轉端上方機構：`w/s/x/i/k/o/p/u/j/h`
> - 分路停止：`1=LEGSTOP`、`2=ARMSTOP`、`3=FIXEDSTOP`
> - 全域停止：`0`

正式手機控制文件請以 **BLE Controller – Arduino ESP32 的文字命令** 為準，不再以單字元按鍵映射作為主流程。

## 相關文件

- BLE Controller 實作流程： [BLE_Controller控制指南.md](BLE_Controller控制指南.md)
- 舊 App Inventor 參考： [AppInventor開發指南.md](AppInventor開發指南.md)
- 主控板與子控板命令總覽： [../../esp32控制/README.md](../../esp32控制/README.md)
