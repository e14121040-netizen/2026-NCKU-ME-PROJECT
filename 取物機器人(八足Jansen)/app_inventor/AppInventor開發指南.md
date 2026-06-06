# App Inventor 開發指南（歷史 / 備用）

> **目前正式控制流程不是 App Inventor。**  
> 本專案現行手機控制端為 **BLE Controller – Arduino ESP32**，請先閱讀 [BLE_Controller控制指南.md](BLE_Controller控制指南.md)。  
> 本文件保留給需要研究舊 `.aia`、參考 UI 佈局、或未來想自行開發備用控制 App 的成員。

## 目前定位

- 正式比賽 / 測試流程：使用 **BLE Controller – Arduino ESP32**
- App Inventor：歷史參考、備用方案、研究學長介面配置用
- 指令規格來源： [App指令對照表.md](App指令對照表.md)

## 可參考的舊資產

學長資料位於：

- `歷屆學長資料(2021屆)/底盤及吸氣馬達操作app.aia`
- `歷屆學長資料(2021屆)/手臂及伺服馬達操作app.aia`

這些檔案可以拿來看：

- 控制畫面如何分區
- 按鈕大小與排列方式
- 基本藍牙元件如何接線

## 若未來要自製備用 App，請遵守的現行規格

1. BLE 裝置名稱固定為 `ESP32_MainController`
2. 正式命令以文字命令為主：
   - 腿部：`FORWARD` / `BACKWARD` / `LEFT` / `RIGHT` / `LEGSTOP`
   - 大圓盤與 z：`TL` / `TR` / `UP` / `DOWN` / `TZSTOP`
   - Servo / 夾爪：`EXTEND` / `RETRACT` / `RSTOP` / `OPEN` / `CLOSE` / `HOME` / `GATEOPEN` / `GATECLOSE` / `SERVOSTOP`
   - 全域急停：`STOP`
3. 備用 App 也必須沿用正式裝置名稱 `ESP32_MainController`。
4. 不要再把「鬆手就送 `0`」當成主要停止策略。
   現行架構使用：
   - `LEGSTOP`
   - `TZSTOP`
   - `RSTOP`
   - `SERVOSTOP`
   - `STOP`

## 為什麼不再推薦 App Inventor 作為主流程

- 現場實際使用的是 **BLE Controller – Arduino ESP32**
- App Inventor 舊流程多半建立在早期單字元命令心智模型上，和目前 BLE 主控行為不完全一致
- 分路停止現在已經是正式控制邏輯，舊的按鍵釋放事件教學容易把停止語義教錯

## 仍然值得保留的用途

- 作為未來自製專用 App 的 UI 草圖參考
- 研究學長做法與按鈕區域分配
- 若 BLE Controller 不夠用，可作為備援開發起點

## 建議閱讀順序

1. [BLE_Controller控制指南.md](BLE_Controller控制指南.md)
2. [App指令對照表.md](App指令對照表.md)
3. `歷屆學長資料(2021屆)/` 內的 `.aia` 與截圖
