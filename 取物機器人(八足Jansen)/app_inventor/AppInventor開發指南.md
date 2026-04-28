# 遙控 App — 開發指南

> 本組使用 **ESP32 內建藍芽** 連線遙控 App。  
> 可選用 App Inventor（自建 App）或現成的 Bluetooth Controller / BLE UART App。  
> 指令字元對照請見 → [App指令對照表.md](App指令對照表.md)

## 方式一：修改學長的 App（推薦新手）

### 步驟

1. 開啟 [App Inventor](https://appinventor.mit.edu/)，登入 Google 帳號
2. 選擇 **Projects → Import project (.aia) from my computer**
3. 匯入學長的 `.aia` 檔案：
   - `歷屆學長資料(2021屆)/底盤及吸氣馬達操作app.aia`（底盤控制）
   - `歷屆學長資料(2021屆)/手臂及伺服馬達操作app.aia`（手臂控制）
4. 開啟學長的 App 截圖，了解原始介面佈局
5. 修改按鈕的發送字元，使其符合 [App指令對照表.md](App指令對照表.md)
6. 合併為一個 App（步行 + 手臂 + z/夾爪在同一畫面）

### 需要修改的重點
- Baud Rate：ESP32 藍芽使用 **115200**
- 步行指令字元：`f/b/l/r/q/e/0`（大寫 `F/B/L/R` = 半速）
- 手臂 r/θ 指令：`w`(伸出) / `s`(縮回) / `a`(左轉) / `d`(右轉)
- z/夾爪指令：`u`(z↑) / `j`(z↓) / `o`(爪開) / `p`(爪合) / `t`(蓋板開) / `y`(蓋板關) / `h`(歸位)

---

## 方式二：從零建立 App

### 第一步：建立新專案

1. App Inventor → **Start new project** → 命名為 `PickupRobotController`
2. Screen 設定：
   - **ScreenOrientation** = Landscape（橫式）
   - **Title** = `取物機器人遙控`

### 第二步：加入藍芽元件

從 **Palette** 拖入：
- `BluetoothClient`（Non-visible，連線用）
- `ListPicker`（按鈕，用來選擇藍芽裝置）
- `Button`（斷開連線用）

### 第三步：建立三區域控制按鈕

建議佈局（使用 `HorizontalArrangement` 和 `VerticalArrangement`）：

```
┌────────────────────────────────────────────────────────┐
│ [連線藍芽 ListPicker]                    [斷開 Button] │
│                                                        │
│   步行控制              手臂 r/θ        z/夾爪控制     │
│       [▲ btn_fwd]      [伸出 btn_ext]  [z↑ btn_zu]   │
│   [◀] [⬜ btn_stop] [▶] [左 btn_al][右 btn_ar]       │
│       [▼ btn_bwd]      [縮回 btn_ret]  [z↓ btn_zd]   │
│                                                        │
│   [↺ btn_sl] [↻ btn_sr]               [爪開][爪合]    │
│                                        [蓋板開][蓋板關] │
│   [ 慢速模式 Switch ]                  [歸位 btn_home] │
└────────────────────────────────────────────────────────┘
```

每個按鈕設定：
- **Width** = 最少 80px（建議更大，方便比賽時操作）
- **FontSize** = 20 以上
- **BackgroundColor** = 依功能分色（步行=藍、手臂=綠、夾爪=橘）

### 第四步：設計 Blocks

#### 藍芽連線

```
When ListPicker1.BeforePicking:
    Set ListPicker1.Elements to BluetoothClient1.AddressesAndNames

When ListPicker1.AfterPicking:
    Call BluetoothClient1.Connect(address = ListPicker1.Selection)
    If BluetoothClient1.IsConnected:
        Set Label_status.Text to "已連線"
```

#### 步行按鈕（按住/放開邏輯）

```
When btn_fwd.TouchDown:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "f")

When btn_fwd.TouchUp:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "0")
```

> **關鍵**：步行按鈕用 `TouchDown`（按住）+ `TouchUp`（放開停止），  
> 這樣放開手指機器人就會停下來，避免失控。

#### 手臂 r/θ 按鈕（按住/放開邏輯，DC 馬達持續動作）

```
When btn_ext.TouchDown:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "w")

When btn_ext.TouchUp:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "0")
```

> **注意**：手臂使用 DC 馬達（非伺服），需要和步行一樣用按住/放開控制。

#### 夾爪/蓋板按鈕（單擊邏輯，伺服馬達一次到位）

```
When btn_claw_open.Click:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "o")
```

#### 慢速模式切換

```
// 用一個全域變數 isSlow 控制
When Switch_speed.Changed:
    If Switch_speed.On:
        Set global isSlow to true
    Else:
        Set global isSlow to false

// 前進按鈕修改為：
When btn_fwd.TouchDown:
    If global isSlow:
        Call BluetoothClient1.SendText(text = "F")  // 大寫 = 慢速
    Else:
        Call BluetoothClient1.SendText(text = "f")  // 小寫 = 全速
```

### 第五步：測試

1. App Inventor → **Connect → AI Companion**（掃 QR Code 到手機測試）
2. 手機藍芽先配對 `PickupRobot`（BluetoothSerial 版）
3. App 中點「連線藍芽」→ 選擇 `PickupRobot`
4. 逐一測試所有按鈕，確認三個子控板都有回應

### 第六步：匯出安裝

- **Build → App (provide QR code for .apk)** → 手機掃碼安裝

---

## 常見問題

| 問題 | 解決方法 |
|------|----------|
| 找不到藍芽裝置 | 先在手機設定中配對 ESP32（裝置名 `PickupRobot`） |
| 連線後馬達不動 | 確認 ESP-NOW 已初始化、子控板 MAC 地址正確 |
| 按鈕太小不好按 | 加大按鈕的 Width/Height，至少 80×80 |
| iOS 無法使用 App Inventor BT | 改用 BLE 版主控板 + BLE UART App（如 nRF Connect） |
| 偶爾斷線 | 檢查電池電壓、避免離太遠（> 10m） |

