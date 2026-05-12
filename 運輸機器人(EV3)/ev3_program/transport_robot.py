#!/usr/bin/env pybricks-micropython
"""
=====================================================
 2026 NCKU 機械專題實作 — 運輸機器人（EV3 全自動）
 平台：LEGO EV3 + Pybricks MicroPython
=====================================================

 任務流程：
 1. 等待啟動（按鈕 or 觸碰感測器）
 2. PID 循跡沿黑線行駛
 3. 偵測到黃色停止線 → 停車 → 啟動輸送帶將零件推落至放置區平台
 4. 繼續循跡或返回起點
 5. 偵測到紅色停止線 → 停車等待下一輪

 感測器配置：
 - Port 1：前下顏色感測器（循跡與標線偵測）
 - Port 4：觸碰感測器（啟動/零件偵測）

 馬達配置：
 - Port A：輸送帶馬達（中馬達，驅動頂部水平輸送帶）
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
motor_left = Motor(Port.D, Direction.CLOCKWISE)
motor_right = Motor(Port.C, Direction.CLOCKWISE)

# ── 輸送帶開關：沒有接 Port A 馬達時設為 False ──
HAS_CONVEYOR_MOTOR = False
motor_conveyor = Motor(Port.A) if HAS_CONVEYOR_MOTOR else None

# 底盤（DriveBase 簡化直線/轉向控制）
# 參數需依實際輪子直徑和軸距調整
WHEEL_DIAMETER = 56  # mm（EV3 大輪）
AXLE_TRACK = 120  # mm（左右輪距）
robot = DriveBase(motor_left, motor_right, WHEEL_DIAMETER, AXLE_TRACK)

# 感測器
sensor_line = ColorSensor(Port.S1)  # 循跡與停止線偵測
touch_sensor = TouchSensor(Port.S4)  # 觸碰感測器

# =====================================================
#  參數設定（需實際調校）
# =====================================================
# PID 循跡參數
KP = 0.6  # 比例增益（擺動大→降低，跟線不緊→提高）
KI = 0.0  # 積分增益（先設0，穩定後再微調）
KD = 0.3  # 微分增益（抑制震盪）
BASE_SPEED = 120  # 基礎行駛速度 (mm/s)（速度慢更容易調參）
SPEED_DROP_RATE = 1.5
MIN_SPEED = 40
MAX_ABS_CORRECTION = 7

# 循跡閾值（可由 calibrate_sensors() 自動計算）
LINE_THRESHOLD = 50  # 反射值低於此為黑線

# 電量檢查間隔 (ms)
BATTERY_CHECK_INTERVAL = 10000

# 輸送帶參數
CONVEYOR_SPEED = 300  # 輸送帶馬達轉速 (deg/s)
CONVEYOR_RUN_TIME = 3000  # 輸送帶運轉時間 (ms)，需依實際帶長調整

# 放置區數量
NUM_ZONES = 15

# 安全參數
LINE_FOLLOW_TIMEOUT = 60000  # 循跡逾時 (ms)，超過此時間沒偵測到停止線則停車
MIN_BATTERY_MV = 7000  # EV3 低電量警戒 (mV)

# 顏色校準參數
COLOR_DIST_THRESHOLD = (
    35  # RGB Manhattan 距離閾值（|dR|+|dG|+|dB| ≤ 此值則判定為該顏色）
)
COLOR_MIN_R = 15  # 最低 R 通道值（排除黑/白地面誤判）
CALIBRATION_SAMPLES = 10  # 每次校準取樣次數
red_rgb_ref = (0, 0, 0)  # 紅線 RGB 參考值（校準後更新）
yellow_rgb_ref = (0, 0, 0)  # 黃線 RGB 參考值（校準後更新）
colors_calibrated = False  # 是否已完成顏色校準


# =====================================================
#  PID 循跡控制器
# =====================================================
class PIDController:
    """PID 控制器用於循跡

    PID（比例-積分-微分）控制器根據當前誤差、累積誤差和誤差變化率
    計算修正值，用於調整機器人的轉向以保持沿黑線行駛。
    """

    def __init__(self, kp, ki, kd):
        """初始化 PID 控制器

        Args:
            kp (float): 比例增益
            ki (float): 積分增益
            kd (float): 微分增益
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0  # 積分項：累積誤差
        self.last_error = 0  # 上一次誤差，用於計算微分項

    def compute(self, error):
        """計算 PID 輸出值

        Args:
            error (float): 當前誤差值（感測器反射值與閾值之差）

        Returns:
            float: PID 計算出的修正值，用於調整轉向
        """
        # 積分項累加當前誤差
        self.integral += error
        # 防止積分飽和（避免積分項過大導致系統不穩定）
        self.integral = max(-1000, min(1000, self.integral))
        # 微分項：誤差變化率
        derivative = error - self.last_error
        # 更新上一次誤差
        self.last_error = error
        # PID 公式：P + I + D
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        """重置 PID 控制器狀態

        清除積分項和上一次誤差，通常在開始新任務時呼叫
        """
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
    if motor_conveyor is not None:
        motor_conveyor.stop()
    ev3.speaker.beep(frequency=400, duration=300)
    show_status(message)


def wait_button_press(confirmed: Button = Button.CENTER, reject: Button | None = None):
    """等待指定的按鈕被按下。

    Args:
        confirmed: 按下該按鈕時返回 True。
        reject: 按下該按鈕時返回 False（可為 None 表示忽略）。

    Returns:
        按下 confirmed 時回 True；若設定 reject 且按下時回 False。
    """
    while True:
        pressed = ev3.buttons.pressed()
        if confirmed in pressed:
            wait(300)  # debounce
            return True
        if reject is not None and reject in pressed:
            return False
        wait(50)


def rgb_multi_sampling(sensor: ColorSensor, samples=CALIBRATION_SAMPLES):
    """多次採樣以獲得較準確的 RGB 值。

    Args:
        sensor: 進行取樣的顏色感測器。
        samples: 取樣次數。

    Returns:
        平均後的 RGB 值（整數 tuple）。
    """
    r_sum, g_sum, b_sum = 0, 0, 0
    for _ in range(samples):
        r, g, b = sensor.rgb()
        r_sum += r
        g_sum += g
        b_sum += b
        wait(50)
    return (r_sum // samples, g_sum // samples, b_sum // samples)


def calibrate_sensors():
    """
    啟動時校準感測器：讀取白色地板和黑線的反射值，
    """
    global LINE_THRESHOLD
    show_status("Cal Line Thr?", "OK: Press Center", "SKIP: Press UP")
    if wait_button_press(Button.CENTER, Button.UP):
        # 檢查反射值

        # 讀取白色反射值
        show_status("Calibration", "Place on WHITE", "Press Center")
        wait_button_press(Button.CENTER)
        white_val = sensor_line.reflection()
        ev3.speaker.beep(frequency=600, duration=200)

        # 讀取黑色反射值
        show_status("Calibration", "Place on BLACK", "Press Center")
        wait_button_press(Button.CENTER)
        black_val = sensor_line.reflection()
        ev3.speaker.beep(frequency=800, duration=200)

        # 取 1/2 白與 1/2 黑 作為閾值
        LINE_THRESHOLD = int((white_val + black_val) / 2)

        show_status(
            "Line Cal Done!",
            "W:{} B:{}".format(int(white_val), int(black_val)),
            "Thr:{}".format(LINE_THRESHOLD),
        )
        wait(3000)


def calibrate_colors():
    """
    啟動時校準感測器：讀取紅線和黃線的 RGB 值。
    """
    global RGB_REF_RED, RGB_REF_YELLOW, COLORS_CALIBRATED

    show_status("Cal Color?", "OK: Press Center", "SKIP: Press UP")
    if wait_button_press(Button.CENTER, Button.UP):
        # 檢查顏色

        # 讀取紅色 RGB
        show_status("Calibration", "Place on RED", "Press Center")
        wait_button_press(Button.CENTER)
        RGB_REF_RED = rgb_multi_sampling(sensor_line)
        ev3.speaker.beep(frequency=600, duration=200)

        # 讀取黃色 RGB
        show_status("Calibration", "Place on YELLOW", "Press Center")
        wait_button_press(Button.CENTER)
        RGB_REF_YELLOW = rgb_multi_sampling(sensor_line)
        ev3.speaker.beep(frequency=600, duration=200)

        COLORS_CALIBRATED = True

        show_status(
            "Color Cal Done!",
            "R:{},{},{}".format(*RGB_REF_RED),
            "Y:{},{},{}".format(*RGB_REF_YELLOW),
        )
        wait(3000)


# =====================================================
#  核心函式
# =====================================================
def read_line_position():
    """
    讀取感測器的反射值，計算與目標閾值的誤差。

    使用左側邊緣循跡策略：
    - 當感測器在白色區域（reflection > threshold）時，誤差為正，表示需要右轉修正
    - 當感測器在黑色線上（reflection < threshold）時，誤差為負，表示需要左轉修正

    Returns:
        tuple: (error, reflection_value)
            - error (float): 誤差值，用於 PID 計算
            - reflection_value (int): 原始反射值 (0-100)
    """
    val = sensor_line.reflection()
    error = LINE_THRESHOLD - val
    return error, val


def check_stop_color():
    """
    檢測目前感測器下方的顏色。
    若已校準顏色，使用 RGB Manhattan 距離比對；否則使用內建 color()。
    回傳：Color.YELLOW, Color.RED, 或 None
    """
    if colors_calibrated:
        r, g, b = sensor_line.rgb()
        # 排除黑/白地面（R 通道太低表示不是彩色線）
        if r < COLOR_MIN_R:
            return None
        # 計算與紅線參考值的 Manhattan 距離
        rr, rg, rb = red_rgb_ref
        dist_red = abs(r - rr) + abs(g - rg) + abs(b - rb)
        # 計算與黃線參考值的 Manhattan 距離
        yr, yg, yb = yellow_rgb_ref
        dist_yellow = abs(r - yr) + abs(g - yg) + abs(b - yb)
        # 取距離最近且在閾值內的顏色
        if dist_red <= dist_yellow and dist_red <= COLOR_DIST_THRESHOLD:
            return Color.RED
        if dist_yellow < dist_red and dist_yellow <= COLOR_DIST_THRESHOLD:
            return Color.YELLOW
        return None
    else:
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
    correction = pid.compute(error) * -2
    current_speed = max(MIN_SPEED, BASE_SPEED - int(abs(error) * SPEED_DROP_RATE))

    # 防止卡死
    if correction > 0:
        correction = min(current_speed - MAX_ABS_CORRECTION, correction)
    elif correction < 0:
        correction = max(-(current_speed - MAX_ABS_CORRECTION), correction)

    robot.drive(current_speed, correction)

    # 簡易特徵過濾：黃色/紅色的反射率通常不會是完美的 0 (黑) 或 100 (白)，
    # 這裡我們選擇每 10 次執行 1 次顏色檢測，或是當反射值處於曖昧區間時檢測。
    # 為了程式穩定性，最安全的做法是直接在循跡中定期讀取，但 EV3 顏色切換較慢。
    # 折衷方案：直接讓主迴圈呼叫 check_stop_color。
    # 這裡將顏色判斷保留為獨立函式，或在此處強制偵測。
    return check_stop_color()


def place_parts():
    """
    啟動頂部輸送帶，將帶面上所有零件推落至放置區平台。
    馬達以 CONVEYOR_SPEED 正轉 CONVEYOR_RUN_TIME 毫秒後停止。
    """
    ev3.speaker.beep(frequency=800, duration=200)

    if motor_conveyor is not None:
        # 啟動輸送帶：正轉將零件往外側推落
        motor_conveyor.run(CONVEYOR_SPEED)
        wait(CONVEYOR_RUN_TIME)
        motor_conveyor.stop()
        wait(300)
    else:
        # 無輸送帶馬達：僅停車等待模擬放置
        wait(800)


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
    wait(150)

    calibrate_sensors()

    calibrate_colors()

    ev3.speaker.beep(frequency=600, duration=200)
    wait(1000)  # 啟動延遲

    # ---- 主循環（無限執行，直到手動停止）----
    zones_visited = 0
    battery_timer = StopWatch()  # 獨立電量檢查計時器

    while True:
        show_status("Running...", "Placed: {}".format(zones_visited))

        pid.reset()
        segment_timer = StopWatch()

        # ---- 循跡行駛 ----
        yellow_count = 0  # 黃線連續偵測計數（防誤判）
        while True:
            stop_color = follow_line()

            if stop_color == Color.YELLOW:
                yellow_count += 1
                if yellow_count >= 1:  # 連續 1 次才確認為黃線
                    # 到達放置區
                    robot.stop()
                    wait(300)

                    # 放置零件
                    place_parts()
                    zones_visited += 1

                    show_status("Placed!", "Total: {}".format(zones_visited))

                    # 前進跨越黃線，避免下次循跡立刻再偵測到黃色
                    robot.straight(10)
                    wait(300)
                    break
            else:
                yellow_count = 0  # 非黃色則重置計數器

            if stop_color == Color.RED:
                # 回到起點
                robot.stop()
                show_status("At Start", "Waiting...")

                # 等待觸碰感測器啟動下一輪
                while not touch_sensor.pressed():
                    show_status("At Start", "Waiting...")
                    for _ in range(20):  # 20 x 100ms = 2s
                        wait(100)
                        if touch_sensor.pressed():
                            break
                for _ in range(5):  # debounce 500ms
                    wait(100)
                # 前進跨越紅線，避免下次循跡立刻再偵測到紅色
                robot.straight(10)
                wait(300)
                break

            # Timeout 保護：循跡過久表示可能脱軌
            if segment_timer.time() > LINE_FOLLOW_TIMEOUT:
                robot.stop()
                ev3.speaker.beep(frequency=200, duration=500)
                show_status("Timeout!", "Retrying...")
                wait(100)
                pid.reset()
                segment_timer.reset()

            # 定期檢測電量（使用獨立計時器，可靠觸發）
            if battery_timer.time() > BATTERY_CHECK_INTERVAL:
                battery_timer.reset()
                check_battery()

            wait(10)  # 循跡迴圈延遲

    # ---- 任務完成 ----
    robot.stop()
    show_status("Mission Complete!", "Zones: {}".format(zones_visited))
    ev3.speaker.beep(frequency=1000, duration=1000)


# 執行主程式
try:
    main()
except Exception as e:
    # 安全停車：任何異常都先停下馬達
    safe_stop("ERROR!")
    ev3.screen.print(str(e))
    wait(10000)
