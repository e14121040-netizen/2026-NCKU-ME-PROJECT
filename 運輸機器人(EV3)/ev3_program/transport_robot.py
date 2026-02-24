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
 - Port 1：左顏色感測器（循跡用）
 - Port 2：右顏色感測器（循跡用）
 - Port 3：前方顏色感測器（停止線偵測）
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
sensor_left  = ColorSensor(Port.S1)   # 左循跡
sensor_right = ColorSensor(Port.S2)   # 右循跡
sensor_front = ColorSensor(Port.S3)   # 停止線偵測
touch_sensor = TouchSensor(Port.S4)   # 觸碰感測器

# =====================================================
#  參數設定（需實際調校）
# =====================================================
# PID 循跡參數
KP = 1.2       # 比例增益
KI = 0.01      # 積分增益
KD = 0.5       # 微分增益
BASE_SPEED = 100  # 基礎行駛速度 (mm/s)

# 循跡閾值
LINE_THRESHOLD = 20  # 反射值低於此為黑線

# 放置機構參數
PLACE_ANGLE = 90      # 放置機構旋轉角度
PLACE_SPEED = 200     # 放置機構速度 (deg/s)

# 放置區數量
NUM_ZONES = 4

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
#  核心函式
# =====================================================
def read_line_position():
    """
    讀取左右感測器的反射值，計算偏差。
    回傳值：負 = 偏左, 0 = 正中, 正 = 偏右
    """
    left_val  = sensor_left.reflection()
    right_val = sensor_right.reflection()
    error = left_val - right_val
    return error


def check_stop_line():
    """
    檢測前方感測器是否偵測到停止線。
    回傳顏色：Color.YELLOW, Color.RED, 或 None
    """
    color = sensor_front.color()
    if color == Color.YELLOW:
        return Color.YELLOW
    elif color == Color.RED:
        return Color.RED
    return None


def follow_line():
    """
    執行一步 PID 循跡。
    回傳偵測到的停止線顏色，沒有則回傳 None。
    """
    error = read_line_position()
    correction = pid.compute(error)
    
    # 計算左右輪速度
    left_speed  = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction
    
    robot.drive(BASE_SPEED, correction * 2)  # DriveBase: (speed, turn_rate)
    
    return check_stop_line()


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


def return_to_start():
    """
    循跡返回起點（偵測到紅色停止線停下）。
    注意：需要掉頭或走回路到起點，依場地軌跡設計調整。
    """
    pid.reset()
    
    while True:
        stop_color = follow_line()
        
        if stop_color == Color.RED:
            robot.stop()
            ev3.speaker.beep(frequency=1000, duration=500)
            return
        
        wait(10)


# =====================================================
#  主程式
# =====================================================
def main():
    ev3.speaker.beep(frequency=500, duration=300)
    ev3.screen.clear()
    ev3.screen.print("Transport Robot")
    ev3.screen.print("Ready!")
    ev3.screen.print("")
    ev3.screen.print("Press Touch or")
    ev3.screen.print("Center Button")
    
    # ---- 等待啟動 ----
    while True:
        if touch_sensor.pressed() or Button.CENTER in ev3.buttons.pressed():
            break
        wait(50)
    
    ev3.speaker.beep(frequency=600, duration=200)
    wait(1000)  # 啟動延遲
    
    # ---- 主循環 ----
    zones_visited = 0
    timer = StopWatch()
    
    while zones_visited < NUM_ZONES:
        ev3.screen.clear()
        ev3.screen.print("Running...")
        ev3.screen.print("Zone: {}/{}".format(zones_visited + 1, NUM_ZONES))
        
        pid.reset()
        
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
                
                ev3.screen.clear()
                ev3.screen.print("Placed at zone")
                ev3.screen.print(str(zones_visited))
                wait(500)
                break
            
            elif stop_color == Color.RED:
                # 回到起點
                robot.stop()
                ev3.screen.clear()
                ev3.screen.print("At Start")
                ev3.screen.print("Waiting...")
                
                # 等待新一批零件（觸碰感測器觸發）
                while not touch_sensor.pressed():
                    wait(50)
                wait(500)
                break
            
            wait(10)  # 循跡迴圈延遲
    
    # ---- 任務完成 ----
    robot.stop()
    ev3.screen.clear()
    ev3.screen.print("Mission Complete!")
    ev3.screen.print("Zones: {}".format(zones_visited))
    ev3.speaker.beep(frequency=1000, duration=1000)


# 執行主程式
main()
