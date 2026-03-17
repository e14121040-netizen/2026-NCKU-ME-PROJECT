#!/usr/bin/env pybricks-micropython
"""
=====================================================
 2026 NCKU 機械專題實作 — 運輸機器人（EV3 全自動）
 平台：LEGO EV3 + Pybricks MicroPython
=====================================================

 任務流程：
 1. 等待啟動（按鈕 or 觸碰感測器）
 2. PID 循跡沿黑線行駛
 3. 偵測到黃色停止線 → 停車 → 放置零件
 4. 繼續循跡或返回起點
 5. 偵測到紅色停止線 → 停車等待下一輪

 感測器配置：
 - Port 1：前下顏色感測器（循跡與標線偵測）
 - Port 4：觸碰感測器（啟動/零件偵測）

 馬達配置：
 - Port A：放置機構馬達（中馬達）
 - Port B：左輪馬達（大馬達）
 - Port C：右輪馬達（大馬達）
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# =====================================================
#  硬體初始化
# =====================================================
ev3 = EV3Brick()

# 馬達
motor_left  = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.C, Direction.CLOCKWISE)
motor_place = Motor(Port.A)  # 放置機構

# 底盤（DriveBase 簡化直線/轉向控制）
# 參數需依實際輪子直徑和軸距調整
WHEEL_DIAMETER = 56   # mm（EV3 大輪）
AXLE_TRACK     = 120  # mm（左右輪距）
robot = DriveBase(motor_left, motor_right, WHEEL_DIAMETER, AXLE_TRACK)

# 感測器
sensor_line = ColorSensor(Port.S1)    # 循跡與停止線偵測
touch_sensor = TouchSensor(Port.S4)   # 觸碰感測器

# =====================================================
#  參數設定（需實際調校）
# =====================================================
# PID 循跡參數
KP = 1.2       # 比例增益
KI = 0.01      # 積分增益
KD = 0.5       # 微分增益
BASE_SPEED = 100  # 基礎行駛速度 (mm/s)

# 循跡閾值（可由 calibrate_sensors() 自動計算）
LINE_THRESHOLD = 20  # 反射值低於此為黑線

# 電量檢查間隔 (ms)
BATTERY_CHECK_INTERVAL = 10000

# 放置機構參數
PLACE_ANGLE = 90      # 放置機構旋轉角度
PLACE_SPEED = 200     # 放置機構速度 (deg/s)

# 放置區數量
NUM_ZONES = 4

# 安全參數
LINE_FOLLOW_TIMEOUT = 30000   # 循跡逾時 (ms)，超過此時間沒偵測到停止線則停車
MIN_BATTERY_MV = 7000         # EV3 低電量警戒 (mV)

# =====================================================
#  PID 循跡控制器
# =====================================================
class PIDController:
    """PID 控制器用於循跡"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
    
    def compute(self, error):
        self.integral += error
        # 防止積分飽和
        self.integral = max(-1000, min(1000, self.integral))
        derivative = error - self.last_error
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
    
    def reset(self):
        self.integral = 0
        self.last_error = 0


pid = PIDController(KP, KI, KD)

# =====================================================
#  輔助函式
# =====================================================
def show_status(line1, line2="", line3=""):
    """在 EV3 螢幕顯示狀態（含電量）"""
    ev3.screen.clear()
    battery_mv = ev3.battery.voltage()
    battery_v = battery_mv / 1000.0
    ev3.screen.print("Bat: {:.1f}V".format(battery_v))
    ev3.screen.print(line1)
    if line2:
        ev3.screen.print(line2)
    if line3:
        ev3.screen.print(line3)


def check_battery():
    """檢測電池電量，低於警戒值則節拍提醒"""
    if ev3.battery.voltage() < MIN_BATTERY_MV:
        ev3.speaker.beep(frequency=300, duration=500)
        show_status("LOW BATTERY!", "Charge EV3")
        return True
    return False


def safe_stop(message="Stopped"):
    """安全停車：停止所有馬達，顯示訊息"""
    robot.stop()
    motor_place.stop()
    ev3.speaker.beep(frequency=400, duration=300)
    show_status(message)


def calibrate_sensors():
    """
    啟動時校準感測器：讀取白色地板和黑線的反射值，
    自動計算 LINE_THRESHOLD。
    """
    global LINE_THRESHOLD
    show_status("Calibration", "Place on WHITE", "Press Center")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    wait(300)  # debounce
    white_val = sensor_line.reflection()
    ev3.speaker.beep(frequency=600, duration=200)

    show_status("Calibration", "Place on BLACK", "Press Center")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    wait(300)  # debounce
    black_val = sensor_line.reflection()
    ev3.speaker.beep(frequency=800, duration=200)

    # 取白與黑的中間值作為閾值
    LINE_THRESHOLD = int((white_val + black_val) / 2)

    show_status(
        "Cal Done!",
        "W:{} B:{}".format(int(white_val), int(black_val)),
        "Thr:{}".format(LINE_THRESHOLD)
    )
    wait(1500)


# =====================================================
#  核心函式
# =====================================================
def read_line_position():
    """
    讀取感測器的反射值，計算與目標閾值的誤差。
    左側邊緣循跡：
    - 偏白 (reflection > threshold) -> 誤差為正 -> 需右轉 (正修正)
    - 偏黑 (reflection < threshold) -> 誤差為負 -> 需左轉 (負修正)
    """
    val = sensor_line.reflection()
    error = val - LINE_THRESHOLD
    return error, val

def check_stop_color():
    """
    檢測目前感測器下方的顏色。
    回傳：Color.YELLOW, Color.RED, 或 None
    """
    c = sensor_line.color()
    if c == Color.YELLOW:
        return Color.YELLOW
    elif c == Color.RED:
        return Color.RED
    return None

def follow_line():
    """
    執行一步單感測器 PID 左緣循跡。
    回傳偵測到的停止線顏色，沒有則回傳 None。
    為了避免頻繁切換 reflection/color 導致卡頓，
    只在 reflection 值極度偏離黑白典型值（例如遇到了黃色或紅色）
    或定時檢測時才讀取 color。
    """
    error, ref_val = read_line_position()
    correction = pid.compute(error)
    
    # 執行轉向 (速度, 轉向率)
    robot.drive(BASE_SPEED, correction * 2)
    
    # 簡易特徵過濾：黃色/紅色的反射率通常不會是完美的 0 (黑) 或 100 (白)，
    # 這裡我們選擇每 10 次執行 1 次顏色檢測，或是當反射值處於曖昧區間時檢測。
    # 為了程式穩定性，最安全的做法是直接在循跡中定期讀取，但 EV3 顏色切換較慢。
    # 折衷方案：直接讓主迴圈呼叫 check_stop_color。
    # 這裡將顏色判斷保留為獨立函式，或在此處強制偵測。
    return check_stop_color()


def place_parts():
    """
    驅動放置機構，將零件放到放置區平台上。
    """
    ev3.speaker.beep(frequency=800, duration=200)
    
    # 放置動作：旋轉放置機構
    motor_place.run_angle(PLACE_SPEED, PLACE_ANGLE, then=Stop.HOLD, wait=True)
    wait(500)
    
    # 復位
    motor_place.run_angle(PLACE_SPEED, -PLACE_ANGLE, then=Stop.HOLD, wait=True)
    wait(300)



# =====================================================
#  主程式
# =====================================================
def main():
    ev3.speaker.beep(frequency=500, duration=300)
    show_status("Transport Robot", "Ready!", "Press Touch/Center")
    
    # ---- 等待啟動 ----
    while True:
        if touch_sensor.pressed() or Button.CENTER in ev3.buttons.pressed():
            break
        wait(50)
    
    # 初始電量檢測
    if check_battery():
        wait(5000)
        # 低電量但仍允許啟動，只是警告
    
    # ---- 感測器校準（可選）----
    # 按下 EV3 上方按鈕（UP）跳過校準，按中心鈕開始校準
    show_status("Calibrate?", "Center=Yes", "Up=Skip")
    cal_timer = StopWatch()
    do_calibrate = False
    while cal_timer.time() < 3000:
        val = sensor_line.reflection()
        show_status(
            "Calibrate? C=Yes",
            "Ref: {}".format(val),
            "Thr:{} U=Skip".format(LINE_THRESHOLD)
        )
        if Button.CENTER in ev3.buttons.pressed():
            do_calibrate = True
            break
        if Button.UP in ev3.buttons.pressed():
            break
        wait(100)
    
    if do_calibrate:
        calibrate_sensors()
    
    ev3.speaker.beep(frequency=600, duration=200)
    wait(1000)  # 啟動延遲
    
    # ---- 主循環 ----
    zones_visited = 0
    battery_timer = StopWatch()  # 獨立電量檢查計時器
    
    while zones_visited < NUM_ZONES:
        show_status(
            "Running...",
            "Zone: {}/{}".format(zones_visited + 1, NUM_ZONES)
        )
        
        pid.reset()
        segment_timer = StopWatch()
        
        # ---- 循跡行駛 ----
        while True:
            stop_color = follow_line()
            
            if stop_color == Color.YELLOW:
                # 到達放置區
                robot.stop()
                wait(300)
                
                # 放置零件
                place_parts()
                zones_visited += 1
                
                show_status(
                    "Placed!",
                    "Zone {}/{}".format(zones_visited, NUM_ZONES)
                )
                wait(500)
                break
            
            elif stop_color == Color.RED:
                # 回到起點
                robot.stop()
                show_status("At Start", "Waiting...")
                
                # 等待新一批零件（觸碰感測器觸發）
                while not touch_sensor.pressed():
                    # 等待時定期更新電量顯示
                    show_status("At Start", "Waiting...")
                    wait(2000)
                wait(500)
                break
            
            # Timeout 保護：循跡過久表示可能脱軌
            if segment_timer.time() > LINE_FOLLOW_TIMEOUT:
                robot.stop()
                ev3.speaker.beep(frequency=200, duration=1000)
                show_status("Timeout!", "Retrying...")
                wait(2000)
                # 重置後繼續嘗試（不再呼叫 safe_stop 以避免卡住）
                pid.reset()
                segment_timer.reset()
                # 注意：下一次迴圈的 follow_line() 會重新驅動馬達
            
            # 定期檢測電量（使用獨立計時器，可靠觸發）
            if battery_timer.time() > BATTERY_CHECK_INTERVAL:
                battery_timer.reset()
                check_battery()
            
            wait(10)  # 循跡迴圈延遲
    
    # ---- 任務完成 ----
    robot.stop()
    show_status(
        "Mission Complete!",
        "Zones: {}".format(zones_visited)
    )
    ev3.speaker.beep(frequency=1000, duration=1000)


# 執行主程式
try:
    main()
except Exception as e:
    # 安全停車：任何異常都先停下馬達
    safe_stop("ERROR!")
    ev3.screen.print(str(e))
    wait(10000)
