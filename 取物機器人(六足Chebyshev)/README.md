# 取物機器人（六足 Chebyshev）

> 本目錄包含六足 Chebyshev 連桿取物機器人的全部資料。

## 系統架構

```
手機 App (App Inventor)
    │  藍芽 (HC-05, 38400 baud)
    ▼
Arduino MEGA 2560
    ├── L298N → 直流減速馬達 ×2（左右側六足步行）
    └── PCA9685 (I2C) → 伺服馬達群
         ├── CH0: 肩關節 (MG996R)
         ├── CH1: 肘關節 (MG996R)
         └── CH2: 夾爪   (SG90/MG996R)
```

## 步行機構

- **類型**：Chebyshev Lambda Linkage 六足
- **步態**：三角步態（Tripod Gait）— 左右曲柄錯開 180°
- **驅動**：左右各 1 顆直流減速馬達，差速轉向
- **連桿參數**：曲柄 30mm / 連桿 75mm / 機架 75mm / 搖桿 75mm

詳細連桿設計請見 → [Chebyshev連桿設計參考.md](機構設計/Chebyshev連桿設計參考.md)

## 夾取機構

- 2~3 軸機械手臂（肩 + 肘 + 夾爪）
- 伺服馬達驅動，透過 PCA9685 控制
- 角度範圍與歸位值定義在 `walking_robot.ino` 中

## 目錄結構

```
取物機器人(六足Chebyshev)/
├── README.md              ← 本文件
├── BOM.md                 ← 零件清單與預算
├── arduino/
│   └── walking_robot.ino  ← Arduino 主程式
├── app_inventor/
│   ├── App指令對照表.md    ← 藍芽指令對照
│   └── AppInventor開發指南.md ← App 開發教學
├── 電控/
│   └── 接線圖.md          ← 完整接線參考
└── 機構設計/
    ├── Chebyshev連桿設計參考.md ← 連桿理論與尺寸
    └── chebyshev_visualization.py ← 連桿動畫工具
```

## 快速開始

1. 閱讀 [BOM.md](BOM.md) 確認零件
2. 閱讀 [電控/接線圖.md](電控/接線圖.md) 完成接線
3. 用 Arduino IDE 上傳 [walking_robot.ino](arduino/walking_robot.ino)
4. 用 App Inventor 建立遙控 App（見 [AppInventor開發指南.md](app_inventor/AppInventor開發指南.md)）
5. 測試藍芽控制步行和手臂

## 藍芽指令速查

| 指令 | 功能 | 指令 | 功能 |
|------|------|------|------|
| `f` | 前進 | `u` | 肩上升 |
| `b` | 後退 | `d` | 肩下降 |
| `l` | 左轉 | `i` | 肘上升 |
| `r` | 右轉 | `k` | 肘下降 |
| `q` | 左旋轉 | `o` | 爪張開 |
| `e` | 右旋轉 | `p` | 爪閉合 |
| `0` | 停止 | `h` | 手臂歸位 |

大寫 `F/B/L/R` = 半速版本。
