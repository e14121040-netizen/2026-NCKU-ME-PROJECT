```mermaid
graph TD
    %% ================= 樣式設定 =================
    classDef power fill:#ffe4e1,stroke:#a52a2a,stroke-width:2px;
    classDef mcu fill:#e0ffff,stroke:#008b8b,stroke-width:2px;
    classDef motor fill:#fffacd,stroke:#b8860b,stroke-width:2px;
    classDef note fill:#fcfcfa,stroke:#333,stroke-dasharray: 5 5,stroke-width:2px;

    %% ================= 電源系統 =================
    subgraph Power [機器人電源配置區]
        BATT[4x 18650 鋰電池<br>串聯 14.8V]
        XL4015[XL4015 降壓模組<br>5V / 5A]
        LM2596[LM2596 降壓模組<br>5V / 3A]

        BATT -->|14.8V 供電| XL4015
        BATT -->|14.8V 供電| LM2596
        BATT -->|14.8V 供電| BTS_VMOT[供電至：各區<br>BTS7960 VMOT 腳位]
        
        XL4015 -->|5V 穩定大電流| SERVO_PWR[供電至：所有<br>伺服馬達 VCC 接腳]
        LM2596 -->|5V 穩定電源| MCU_PWR[供電至：所有<br>C3 控制板 5V 腳位]
    end

    %% ================= 主控系統 =================
    subgraph Master [主控指令中心]
        APP(手機遙控 App) -.->|藍牙 Bluetooth 指令| ESP32_M[ESP32 主控板]
        PC[電腦 USB 供電] -->|USB 5V 獨立供電| ESP32_M
    end

    %% ================= 子控板 1 =================
    subgraph Slave1 [ESP32-C3 #1：腿部移動子系統]
        C3_1[ESP32-C3 #1<br>控制板] ==>|PWM 控制訊號 L/R| BTS_1[BTS7960<br>驅動器 x2]
        BTS_1 ==>|大電流驅動| M_LEG[JGB37-520<br>減速馬達 x2]
    end

    %% ================= 子控板 2 =================
    subgraph Slave2 [ESP32-C3 #2：大圓盤與 Z 軸子系統]
        C3_2[ESP32-C3 #2<br>控制板] ==>|PWM 控制訊號| BTS_23[BTS7960<br>驅動器 #3 & #4]
        BTS_23 ==>|大電流驅動| M_DISCZ[大圓盤旋轉馬達<br>& Z軸升降馬達]

    end

    %% ================= 子控板 3 =================
    subgraph Slave3 [ESP32-C3 #3：伺服馬達與夾爪子系統]
        C3_3[ESP32-C3 #3<br>控制板] ==>|PWM 控制訊號| SRV_ALL[MG996R<br>伺服馬達 x4]

    end

    %% ================= 無線通訊網路 =================
    ESP32_M -.->|ESP-NOW 指令分發| C3_1
    ESP32_M -.->|ESP-NOW 指令分發| C3_2
    ESP32_M -.->|ESP-NOW 指令分發| C3_3

    %% ================= 系統共地提示 =================


    %% ================= 套用樣式 =================
    class BATT,XL4015,LM2596,BTS_VMOT,SERVO_PWR,MCU_PWR power;
    class APP,ESP32_M,C3_1,C3_2,C3_3 mcu;
    class M_LEG,M_DISCZ,SRV_ALL motor;