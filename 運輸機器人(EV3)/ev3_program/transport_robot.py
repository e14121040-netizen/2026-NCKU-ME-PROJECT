#!/usr/bin/env pybricks-micropython
"""
=====================================================
 2026 NCKU 機械專題實作 — 運輸機器人（EV3 全自動）
 平台：LEGO EV3 + Pybricks MicroPython
=====================================================

 任務流程：
 1. 啟動後可選擇校準循跡閾值與紅/黃標線 RGB
 2. PID 循跡沿黑線邊緣行駛
 3. 偵測到黃色標線 → 依目前圈數判斷是否停車放置零件
 4. 偵測到紅色標線 → 視為回到起點，等待下一輪啟動
 5. 四個放置區完成後結束任務

 感測器配置：
 - Port 1：前下顏色感測器（循跡與標線偵測）
 - Port 4：觸碰感測器（校準取消/下一輪啟動）

 馬達配置：
 - Port A：輸送帶馬達（中馬達，驅動頂部水平輸送帶）
 - Port C：右輪馬達（大馬達）
 - Port D：左輪馬達（大馬達）
"""

from contextlib import contextmanager
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# =====================================================
# 參數與常數定義
# =====================================================

# 底盤（DriveBase 簡化直線/轉向控制）
WHEEL_DIAMETER = 56  # mm（EV3 大輪）
AXLE_TRACK = 120  # mm（左右輪距）

# PID 循跡參數
KP = 0.6  # 比例增益：擺動大→降低，跟線不緊→提高
KI = 0.0  # 積分增益：先設0，穩定後再微調
KD = 0.3  # 微分增益：抑制震盪
PID_OUTPUT_SCALE = -2  # PID 輸出縮放倍率
BASE_SPEED = 120  # 基礎行駛速度 (mm/s)
TURN_SLOWDOWN_RATE = 1.5  # 轉彎時的降速倍率
MIN_SPEED = 20  # 降速後允許的最低前進速度
MIN_SPEED_MARGIN = 7  # 保留的前進速度餘量

# 放置機構參數
HAS_PLACE_MOTOR = True  # 是否已安裝輸送/放置機構馬達
PLACE_ANGLE = 225  # 放置機構單次旋轉角度
PLACE_SPEED = 100  # 放置機構轉速 (deg/s)
PLACE_WAIT = 1500  # 放置後等待時間 (ms)
YELLOW_FORWARD_DISTANCE = 40  # 偵測到黃線後繼續循線前進的距離 (mm)

# 安全參數
BATTERY_CHECK_ENABLED = True  # 開啟電量檢查
BATTERY_CHECK_INTERVAL = 10000  # 電量檢查間隔 (ms)
BATTERY_CHECK_MIN_MV = 7000  # EV3 低電量警戒 (mV)
LINE_FOLLOW_TIMEOUT = 60000  # 循跡逾時 (ms)
YELLOW_REACTION_COOLDOWN = 2000  # 黃線去重時間 (ms)

# 顏色校準參數
# Manhattan: |dR|+|dG|+|dB|
COLOR_DIST_THRESHOLD = 35  # RGB Manhattan 距離閾值
COLOR_MIN_R = 15  # 最低 R 通道值
CALIBRATION_SAMPLES = 10  # 每次校準取樣次數

# 循跡閾值常數（由 calibrate_sensors() 自動計算）
LINE_THRESHOLD = 40  # 反射值低於此為黑線
LINE_RECOVERY_MAX_ANGLE = 45  # 原地找線最大掃描角度
LINE_RECOVERY_STEP_ANGLE = 5  # 原地找線每次旋轉角度
LINE_RECOVERY_ERROR_TOLERANCE = 10  # 視為找回線的誤差容許值

# 顏色感測常數（由 calibrate_colors() 自動計算）
RGB_REF_RED = (54, 8, 0)  # 紅線 RGB 參考值
RGB_REF_YELLOW = (77, 40, 4)  # 黃線 RGB 參考值
COLORS_CALIBRATED = False  # 是否已完成顏色校準

# =====================================================
#  硬體初始化
# =====================================================
ev3 = EV3Brick()


def print_exception(e, file=None):
    if file is None:
        file = sys.stdout
    sys.print_exception(e, file)  # pyright: ignore[reportAttributeAccessIssue]


@contextmanager
def catch_init_error(port: str):
    try:
        yield
    except OSError as e:
        print_exception(e)
        ev3.screen.clear()
        ev3.screen.print("Port {} error.".format(port))
        wait(30000)
        raise SystemExit(1)


# 感測器
with catch_init_error("S1"):
    sensor_line = ColorSensor(Port.S1)  # 循跡與停止線偵測
with catch_init_error("S4"):
    touch_sensor = TouchSensor(Port.S4)  # 觸碰感測器

# 馬達
with catch_init_error("C"):
    motor_right = Motor(Port.C, Direction.CLOCKWISE)
with catch_init_error("D"):
    motor_left = Motor(Port.D, Direction.CLOCKWISE)
with catch_init_error("D"):
    motor_place = Motor(Port.A) if HAS_PLACE_MOTOR else None

try:
    # 機器人
    robot = DriveBase(motor_left, motor_right, WHEEL_DIAMETER, AXLE_TRACK)
except ValueError as e:
    print_exception(e)
    ev3.screen.clear()
    ev3.screen.print("Please reconnect")
    ev3.screen.print("motors")
    wait(30000)
    raise SystemExit(1)


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


def show_status(*lines: str):
    """
    在 EV3 螢幕顯示狀態（含電量）。

    Args:
        lines: 顯示內容
    """
    ev3.screen.clear()

    battery_mv = ev3.battery.voltage()
    battery_v = battery_mv / 1000.0
    header = "Bat: {:.1f}V".format(battery_v)
    ev3.screen.print(header)

    for line in lines:
        ev3.screen.print(line)


def check_battery():
    """
    檢測電池電量，低於警戒值則蜂鳴提醒。

    Returns:
        bool: True 表示電量沒有問題，False 表示電量過低
    """
    if BATTERY_CHECK_ENABLED:
        if ev3.battery.voltage() < BATTERY_CHECK_MIN_MV:
            ev3.speaker.beep(frequency=300, duration=500)
            show_status("LOW BATTERY!", "Charge EV3")
            return False
    return True


def safe_stop(message="Stopped"):
    """安全停車：停止所有馬達，顯示訊息"""
    robot.stop()
    if motor_place is not None:
        motor_place.stop()
    ev3.speaker.beep(frequency=400, duration=300)
    show_status(message)


def wait_button_press(accept: Button = Button.CENTER, cancel: Button | None = None):
    """等待指定的按鈕被按下。

    Args:
        accept: 按下該按鈕時返回 True。
        cancel: 按下該按鈕時返回 False（可為 None 表示忽略）。

    Returns:
        bool: 按下 accept 時回 True；若設定 cancel 且按下時回 False。
    """
    while True:
        pressed = ev3.buttons.pressed()
        if accept in pressed:
            wait(300)  # debounce
            return True
        if cancel is not None and cancel in pressed:
            return False
        wait(50)


def wait_touch_press(accept: TouchSensor):
    """等待指定的感測器被按下。

    Args:
        accept: 被按下時返回 True 的觸碰感測器。

    Returns:
        bool: 感測器被按下時回 True。
    """
    while True:
        if accept.pressed():
            wait(300)  # debounce
            return True
        wait(50)


def wait_button_or_touch_cancel(accept: Button, cancel: TouchSensor):
    """等待指定按鈕或觸碰感測器被按下。

    Args:
        accept: 按下該按鈕時返回 True。
        cancel: 按下該感測器時返回 False。

    Returns:
        bool: 按下 accept 時回 True；按下 cancel 感測器時回 False。
    """
    while True:
        pressed = ev3.buttons.pressed()
        if accept in pressed:
            wait(300)
            return True
        if cancel is not None and cancel.pressed():
            wait(300)
            return False
        wait(50)


def rgb_multi_sampling(sensor: ColorSensor, samples=CALIBRATION_SAMPLES):
    """多次採樣以獲得較準確的 RGB 值。

    Args:
        sensor: 進行取樣的顏色感測器。
        samples: 取樣次數。

    Returns:
        tuple: (r, g, b)
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
    校準循跡閾值：讀取白色地板和黑線的反射值並取中間值。
    """
    global LINE_THRESHOLD

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

    # 取白色與黑色反射值的中間值作為循跡閾值
    LINE_THRESHOLD = int((white_val + black_val) / 2)

    show_status(
        "Line Cal Done!",
        "W:{} B:{}".format(int(white_val), int(black_val)),
        "Thr:{}".format(LINE_THRESHOLD),
    )


def calibrate_colors():
    """
    啟動時校準感測器：讀取紅線和黃線的 RGB 值。
    """
    global RGB_REF_RED, RGB_REF_YELLOW, COLORS_CALIBRATED

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


# =====================================================
#  核心函式
# =====================================================
def read_line_position():
    """
    讀取感測器的反射值，計算與目標閾值的誤差。

    誤差定義為 threshold - reflection：
    - 感測器在白色區域（reflection > threshold）時，誤差為負
    - 感測器在黑色線上（reflection < threshold）時，誤差為正

    Returns:
        tuple: (error, reflection_value)
            - error (float): 誤差值，用於 PID 計算
            - reflection_value (int): 原始反射值 (0-100)
    """
    val = sensor_line.reflection()
    error = LINE_THRESHOLD - val
    return error, val


def get_color_manhattan(a: tuple[int, int, int], b: tuple[int, int, int]):
    """
    計算兩顏色的 RGB Manhattan 距離

    Args:
        a: 顏色 A
        b: 顏色 B

    Returns:
        int: 兩顏色的 RGB Manhattan 距離
    """
    ar, ag, ab = a
    br, bg, bb = b
    return abs(ar - br) + abs(ag - bg) + abs(ab - bb)


def check_stop_color():
    """
    檢測目前感測器下方的顏色。
    若已校準顏色，使用 RGB Manhattan 距離比對；否則使用內建 color()。

    Returns:
        color: Color.YELLOW, Color.RED, 或 None
    """
    if COLORS_CALIBRATED:
        rgb = sensor_line.rgb()
        r, g, b = rgb
        # 排除黑/白地面（R 通道太低表示不是彩色線）
        if r < COLOR_MIN_R:
            return None

        dist_red = get_color_manhattan(rgb, RGB_REF_RED)
        dist_yellow = get_color_manhattan(rgb, RGB_REF_YELLOW)

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


def follow_line(ignore_colors=False):
    """
    執行一步單感測器 PID 邊緣循跡。
    若偵測到停止線，直接回傳顏色交由主程式處理，避免彩色標線反射值干擾 PID。

    Returns:
        color: 停止線顏色，可能為 Color.YELLOW, Color.RED, 或 None
    """
    if not ignore_colors:
        stop_color = check_stop_color()

        if stop_color in (Color.YELLOW, Color.RED):
            return stop_color

    error, ref_val = read_line_position()
    correction = pid.compute(error) * PID_OUTPUT_SCALE
    current_speed = max(MIN_SPEED, BASE_SPEED - int(abs(error) * TURN_SLOWDOWN_RATE))

    # 限制轉向修正量，保留最低前進速度
    if correction > 0:
        correction = min(current_speed - MIN_SPEED_MARGIN, correction)
    elif correction < 0:
        correction = max(-(current_speed - MIN_SPEED_MARGIN), correction)

    robot.drive(current_speed, correction)

    return None


def place_parts():
    """
    啟動頂部輸送帶，將帶面上所有零件推落至放置區平台。
    馬達以 PLACE_SPEED 轉動 PLACE_ANGLE 度，等待 PLACE_WAIT 毫秒後再復位。
    """
    ev3.speaker.beep(frequency=800, duration=200)

    if motor_place is not None:
        # 放置動作：旋轉放置機構
        motor_place.run_angle(PLACE_SPEED, PLACE_ANGLE, then=Stop.HOLD, wait=True)
        wait(PLACE_WAIT)

        # 復位
        motor_place.run_angle(PLACE_SPEED, -PLACE_ANGLE, then=Stop.HOLD, wait=True)
        wait(300)
    else:
        # 無輸送帶馬達：僅停車等待模擬放置
        wait(800)


def rotate_until_line(direction, max_angle=LINE_RECOVERY_MAX_ANGLE):
    """以固定步進原地旋轉找線，找到線時回傳 True 與已轉角度。"""
    turned_angle = 0

    while turned_angle < max_angle:
        step_angle = min(LINE_RECOVERY_STEP_ANGLE, max_angle - turned_angle)
        robot.turn(direction * step_angle)
        turned_angle += step_angle

        err, _ = read_line_position()
        if abs(err) < LINE_RECOVERY_ERROR_TOLERANCE:
            return True, turned_angle

        wait(10)

    return False, turned_angle


def recover_line():
    """停車後微調車身，嘗試回到循跡閾值附近。"""
    pid.reset()
    start_angle = robot.angle()

    while True:
        err, val = read_line_position()
        if abs(err) < 10:
            break

        current_angle = robot.angle()
        turned_angle = abs(current_angle - start_angle)
        if turned_angle >= LINE_RECOVERY_MAX_ANGLE:
            robot.stop()
            robot.turn(start_angle - current_angle)
            pid.reset()
            break

        corr = pid.compute(err) * PID_OUTPUT_SCALE
        corr = max(-50, min(50, corr))
        robot.drive(0, corr)
        wait(10)

    robot.stop()


# =====================================================
#  主程式
# =====================================================
def main():
    ev3.speaker.beep(frequency=500, duration=300)
    show_status("Transport Robot", "Ready!", "Press Touch/Center")

    # ---- 等待啟動 ----
    wait(500)

    # 初始電量檢測
    if not check_battery():
        wait(3000)
        # 低電量但仍允許啟動，只是警告

    # 啟動前可選擇是否重新校準循跡與顏色
    show_status("Cal Line Thr & Color?", "OK: Press UP", "CANCEL: Press Touch")
    if wait_button_or_touch_cancel(Button.UP, touch_sensor):
        calibrate_sensors()

        wait(1000)

        calibrate_colors()

        wait(5000)

    ev3.speaker.beep(frequency=600, duration=200)
    wait(1000)  # 啟動延遲

    # ---- 主循環 ----
    zones_visited_all = 0  # 造訪過的全部置物區數
    zones_placed_all = 0  # 放置過的全部置物區數

    zones_visited_current = 0  # 當前圈數造訪過的置物區數
    round_finish = 0  # 已經完成的圈數

    last_yellow_seen_ms = -YELLOW_REACTION_COOLDOWN

    battery_timer = StopWatch()  # 電量檢查計時器
    yellow_seen_timer = StopWatch()  # 最後見到黃線計時器
    segment_timer = StopWatch()  # 本次循線計時器

    # ---- 循跡行駛 ----
    yellow_count = 0  # 黃線確認計數（目前 1 次即確認）
    while True:
        show_status(
            "Running...",
            "Current: {}-{}".format(round_finish + 1, zones_visited_current),
            "Visited: {}".format(zones_visited_all),
            "Placed: {}".format(zones_placed_all),
        )

        stop_color = follow_line()

        if stop_color == Color.YELLOW:
            now = yellow_seen_timer.time()
            yellow_is_new = now - last_yellow_seen_ms >= YELLOW_REACTION_COOLDOWN
            last_yellow_seen_ms = now

            if not yellow_is_new:
                yellow_count = 0
                continue

            yellow_count += 1
            if yellow_count >= 1:  # 目前偵測到 1 次就確認為黃線
                zones_visited_all += 1
                zones_visited_current += 1

                # 第 X 輪只停在第 X 個放置區，其餘黃線僅記錄通過
                if zones_visited_current != round_finish + 1:
                    continue

                zones_placed_all += 1

                # 偵測到黃線後，再向前循線一段距離
                start_dist = robot.distance()
                while (robot.distance() - start_dist) < YELLOW_FORWARD_DISTANCE:
                    follow_line(ignore_colors=True)
                    wait(10)

                # 到達放置區
                robot.stop()
                wait(300)

                # 放置零件
                place_parts()
                show_status("Placed!", "Total: {}".format(zones_placed_all))

                recover_line()
                segment_timer.reset()
                continue
        else:
            yellow_count = 0  # 非黃色則重置計數器

            if stop_color == Color.RED:
                # 紅線代表回到起點
                robot.stop()

                # 已放置到四個放置區，結束程式
                if zones_placed_all == 4:
                    break

                if zones_visited_current:
                    round_finish += 1
                    zones_visited_current = 0

                # 起點等待下一輪
                show_status(
                    "At Start",
                    "Waiting...",
                    "Current: {}-0".format(round_finish + 1),
                )

                # 等待觸碰感測器啟動下一輪
                wait_touch_press(touch_sensor)

                # 等待5秒（200 + 4800）
                ev3.speaker.beep(frequency=700, duration=200)
                wait(4800)

                # 前進跨越紅線
                robot.straight(10)
                wait(300)

                recover_line()
                segment_timer.reset()
                continue

        # Timeout 保護：循跡過久表示可能脱軌
        if segment_timer.time() > LINE_FOLLOW_TIMEOUT:
            robot.stop()
            ev3.speaker.beep(frequency=200, duration=500)
            show_status("Timeout!", "Retrying...")
            wait(100)
            pid.reset()
            segment_timer.reset()

        # 定期檢測電量
        if battery_timer.time() > BATTERY_CHECK_INTERVAL:
            battery_timer.reset()
            check_battery()

        wait(10)  # 循跡迴圈延遲

    # ---- 任務完成 ----
    robot.stop()
    show_status("Mission Complete!", "Zones: {}".format(zones_visited_all))
    ev3.speaker.beep(frequency=1000, duration=1000)


# 執行主程式
try:
    main()
except Exception as e:
    # 安全停車：任何異常都先停下馬達
    safe_stop("ERROR!")
    ev3.screen.print(str(e))
    wait(10000)
