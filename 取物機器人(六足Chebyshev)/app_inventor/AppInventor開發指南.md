# App Inventor 遙控 App — 開發指南

> 本文件教你如何用 App Inventor 建立六足機器人的遙控 App。  
> 指令字元對照請見 → [App指令對照表.md](App指令對照表.md)

## 方式一：修改學長的 App（推薦新手）

### 步驟

1. 開啟 [App Inventor](https://appinventor.mit.edu/)，登入 Google 帳號
2. 選擇 **Projects → Import project (.aia) from my computer**
3. 匯入學長的 `.aia` 檔案：
   - `歷屆學長資料(2021屆)/底盤及吸氣馬達操作app.aia`（底盤控制）
   - `歷屆學長資料(2021屆)/手臂及伺服馬達操作app.aia`（手臂控制）
4. 開啟學長的 App 截圖（同目錄下的 `.png` / `.jpg`），了解原始介面佈局
5. 修改按鈕的發送字元，使其符合 [App指令對照表.md](App指令對照表.md)
6. 合併兩個 App 為一個（步行 + 手臂控制在同一畫面）

### 需要修改的重點
- 藍芽鮑率改為 **38400**
- 步行指令字元改為 `f/b/l/r/q/e/0`
- 新增手臂控制按鈕 `u/d/i/k/o/p/h`

---

## 方式二：從零建立 App

### 第一步：建立新專案

1. App Inventor → **Start new project** → 命名為 `WalkingRobotController`
2. Screen 設定：
   - **ScreenOrientation** = Landscape（橫式）
   - **Title** = `六足遙控`

### 第二步：加入藍芽元件

從 **Palette** 拖入：
- `BluetoothClient`（Non-visible，連線用）
- `ListPicker`（按鈕，用來選擇藍芽裝置）
- `Button`（斷開連線用）

### 第三步：建立步行控制按鈕

建議佈局（使用 `HorizontalArrangement` 和 `VerticalArrangement`）：

```
┌──────────────────────────────────────────┐
│ [連線藍芽 ListPicker]      [斷開 Button] │
│                                          │
│   步行控制               手臂控制        │
│       [▲ btn_fwd]     [肩↑ btn_su]     │
│   [◀] [⬜ btn_stop] [▶]  [歸位 btn_home]│
│       [▼ btn_bwd]     [肩↓ btn_sd]     │
│                                          │
│   [↺ btn_sl] [↻ btn_sr] [爪開] [爪合]  │
│                                          │
│   [速度切換 Switch]    [肘↑] [肘↓]      │
└──────────────────────────────────────────┘
```

每個按鈕設定：
- **Width** = 最少 80px（建議更大，方便操作）
- **FontSize** = 20 以上
- **BackgroundColor** = 依功能分色

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

#### 手臂按鈕（單擊邏輯）

```
When btn_shoulder_up.Click:
    If BluetoothClient1.IsConnected:
        Call BluetoothClient1.SendText(text = "u")
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
2. 手機藍芽先配對 HC-05（預設密碼 `1234`）
3. App 中點「連線藍芽」→ 選擇 HC-05
4. 逐一測試所有按鈕

### 第六步：匯出安裝

- **Build → App (provide QR code for .apk)** → 手機掃碼安裝

---

## 常見問題

| 問題 | 解決方法 |
|------|----------|
| 找不到藍芽裝置 | 先在手機設定中配對 HC-05（密碼 1234） |
| 連線後馬達不動 | 確認 HC-05 鮑率 = 38400，Arduino Serial1 也是 38400 |
| 按鈕太小不好按 | 加大按鈕的 Width/Height，至少 80×80 |
| iOS 無法使用 | HC-05 只支援 Android，iOS 需改用 BLE (HM-10) |
| 偶爾斷線 | 檢查電池電壓、避免離太遠（> 10m） |
