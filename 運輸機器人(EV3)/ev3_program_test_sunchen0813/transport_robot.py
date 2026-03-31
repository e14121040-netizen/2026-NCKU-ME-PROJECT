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
- Port D：左輪馬達（大馬達）
- Port C：右輪馬達（大馬達）
"""

import threading
import queue
from typing import override

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# =====================================================
#  常數定義
# =====================================================
#
# 底盤（DriveBase 簡化直線/轉向控制）
WHEEL_DIAMETER = 56  # mm（EV3 大輪直徑）
AXLE_TRACK = 120  # mm（左右輪距）
ADDITIONAL_DISTANCE_AFTER_STOP = 0  # 檢測到線後繼續往前的距離
#
# PID 循跡參數
KP = 0.85  # 比例增益（擺動大→降低，跟線不緊→提高）
KI = 0.01  # 積分增益（先設0，穩定後再微調）
KD = 0.5  # 微分增益（抑制震盪）
BASE_SPEED = 110  # 基礎行駛速度 (mm/s)（速度慢更容易調參）
#
# 預設循跡閾值（可由 calibrate_sensors() 覆蓋）
LINE_THRESHOLD = 20  # 反射值低於此為黑線
#
# 輸送帶參數
CONVEYOR_ENABLE = False  # 是否已接上輸送帶
CONVEYOR_SPEED = 300  # 輸送帶馬達轉速 (deg/s)
CONVEYOR_RUN_TIME = 3000  # 輸送帶運轉時間 (ms)，需依實際帶長調整
#
# 安全參數
LINE_FOLLOW_TIMEOUT = 30000  # 循跡逾時 (ms)，超過此時間沒偵測到停止線則停車
BATTERY_CHECK_INTERVAL = 10000  # 電量檢查間隔 (ms)
MIN_BATTERY_MV = 7000  # EV3 低電量警戒 (mV)
#
# 顏色校準參數
COLOR_DIST_THRESHOLD = (
    35  # RGB Manhattan 距離閾值（|dR|+|dG|+|dB| ≤ 此值則判定為該顏色）
)
COLOR_MIN_R = 15  # 最低 R 通道值（排除黑/白地面誤判）
CALIBRATION_SAMPLES = 10  # 每次校準取樣次數
CALIBRATION_SAMPLES_RUNNING = 2  # 每次運行時取樣次數
RGB_REF_RED = (0, 0, 0)  # 紅線 RGB 參考值（校準後更新）
RGB_REF_YELLOW = (0, 0, 0)  # 黃線 RGB 參考值（校準後更新）
COLORS_CALIBRATED = False  # 是否已完成顏色校準


# =====================================================
#  PID 循跡控制器
# =====================================================


class PIDController:
    """PID 控制器用於循跡。"""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error: float):
        """計算本次誤差的 PID 輸出。

        Args:
            error: 當前誤差值。

        Returns:
            PID 計算後的修正量。
        """
        self.integral += error
        # 截斷以防止積分越界
        self.integral = max(-1000, min(1000, self.integral))
        derivative = error - self.last_error
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        """重置 PID 的積分與上一次誤差。"""
        self.integral = 0
        self.last_error = 0


# =====================================================
#  車輛
# =====================================================


class Drive(DriveBase):
    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        wheel_diameter: int,
        axle_track: int,
        # --------
        color_sensor: ColorSensor,
        color_sensor_lock: threading.Lock,
        pid: PIDController,
    ):
        """初始化底盤與循跡所需資源。

        Args:
            left_motor: 左輪馬達。
            right_motor: 右輪馬達。
            wheel_diameter: 輪胎直徑（mm）。
            axle_track: 輪距（mm）。
            color_sensor: 循線顏色感測器。
            color_sensor_lock: 感測器讀取鎖。
            pid: PID 控制器。
        """

        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)

        self._left_motor = left_motor
        self._right_motor = right_motor

        # 停止車輛事件
        self._pause_event = threading.Event()

        self._stop_target_distance_lock = threading.Lock()
        # 到定點後的額外移動距離
        self._stop_target_distance: int | None = None

        # 設定初始化狀態為停車
        self._pause_event.set()

        self._color_sensor = color_sensor
        self._color_sensor_lock = color_sensor_lock

        self._pid = pid

        self._line_threshold = LINE_THRESHOLD

    def _read_line_position(self):
        """讀取反射值並計算循跡誤差。

        左側邊緣循跡：
        - 偏白 (reflection > threshold) -> 誤差為正 -> 需右轉 (正修正)
        - 偏黑 (reflection < threshold) -> 誤差為負 -> 需左轉 (負修正)

        Returns:
            (error, reflection) 的 tuple。
        """
        with self._color_sensor_lock:
            val = self._color_sensor.reflection()
        error = val - self._line_threshold
        return error, val

    def follow_line(self, drive_speed=BASE_SPEED):
        """執行單感測器 PID 循跡。

        Args:
            drive_speed: 循線行駛速度（mm/s）。
        """
        error, ref_val = self._read_line_position()
        correction = self._pid.compute(error)

        turn_rate = int(correction * 1)

        # 執行轉向 (速度, 轉向率)
        self.drive(drive_speed, turn_rate)

    def loop(self, end_event: threading.Event, error_queue: queue.Queue):
        """外部呼叫的循線主迴圈。

        Args:
            end_event: 結束程式事件。
            error_queue: 執行中例外的回報佇列。
        """
        try:
            while not end_event.is_set():
                if self._pause_event.is_set():
                    self._hard_stop()
                    wait(10)
                    continue

                # 讀取由 Drive.stop() 設定的追加移動距離
                with self._stop_target_distance_lock:
                    stop_target = self._stop_target_distance

                if stop_target is not None:
                    if self.distance() >= stop_target:
                        with self._stop_target_distance_lock:
                            self._stop_target_distance = None
                        self._pause_event.set()
                        self._hard_stop()
                        wait(10)
                        continue
                    else:
                        # 準備停車了，降速防止衝過頭
                        self.follow_line(BASE_SPEED // 2)
                else:
                    self.follow_line()

                wait(10)

            super().stop()
        except Exception as ex:
            error_queue.put_nowait(ex)

    def start(self):
        """開始或繼續移動。"""
        self._pause_event.clear()

        with self._stop_target_distance_lock:
            # 捨棄掉沒走完的額外距離
            self._stop_target_distance = None

    @override
    def stop(self, additional_distance: int = 0):
        """停下機器人。

        與 `DriveBase.stop` 不同，此實作會向輪子發送停止訊號以強制停下
        （見 `Drive._hard_stop`）。

        Args:
            additional_distance: 若為 0 立刻停止；若非 0，則先繼續循線前進
                指定距離，到了再停。
        """
        if additional_distance <= 0:
            # 立即停止
            with self._stop_target_distance_lock:
                # 重置額外距離
                self._stop_target_distance = None
            self._pause_event.set()
            self._hard_stop()
            return

        # 移除暫停標記
        self._pause_event.clear()

        current_distance = self.distance()
        target_distance = current_distance + int(additional_distance)

        with self._stop_target_distance_lock:
            # 按照現在已經移動的距離計算額外距離
            self._stop_target_distance = target_distance

    def _hard_stop(self):
        """以煞車方式盡可能硬停止。

        ev3-micropython 的 `DriveBase.stop` 只會讓 `DriveBase` 停止並放開，
        因此需要對左右馬達做 brake()。
        """
        super().stop()
        self._pause_event.set()
        self._left_motor.brake()
        self._right_motor.brake()


# =====================================================
#  主程式
# =====================================================


class Main:
    """主程式控制流程。"""

    def __init__(self, end_event: threading.Event) -> None:
        """
        初始化主程式所需資源。

        Args:
            end_event: 結束程式事件。
        """
        # 硬體
        self.ev3 = EV3Brick()

        # 馬達
        self.left_motor = Motor(Port.D, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)

        self.motor_conveyor = Motor(Port.A) if CONVEYOR_ENABLE else None

        # 感測器
        self.color_sensor = ColorSensor(Port.S1)  # 顏色傳感器－循跡與停止線偵測
        self.touch_sensor = TouchSensor(Port.S4)  # 觸碰感測器

        self.robot = Drive(
            self.left_motor,
            self.right_motor,
            WHEEL_DIAMETER,
            AXLE_TRACK,
            self.color_sensor,
            self._color_sensor_lock,
            self._pid,
        )

        # pid 控制
        self._pid = PIDController(KP, KI, KD)
        self._color_sensor_lock = threading.Lock()

        # 顏色感應
        self._rgb_ref_red: tuple[int, int, int] = RGB_REF_RED
        self._rgb_ref_yellow: tuple[int, int, int] = RGB_REF_YELLOW
        self._color_calibrated = False

        self._end_event = end_event

    def initial(self):
        """啟動時的校準流程。

        1. 讀取白色地板和黑線的反射值，自動計算 line threshold。
        2. 讀取紅線與黃線，存入 RGB 參考值。
        """

        self.show_status(["Cal Line Thr?", "OK: Press Center", "SKIP: Press UP"])
        if self._wait_button_press(Button.CENTER, Button.UP):
            # 檢查反射值

            # 讀取白色反射值
            self.show_status(["Calibration", "Place on WHITE", "Press Center"])
            self._wait_button_press(Button.CENTER)
            white_val = self.color_sensor.reflection()
            self.ev3.speaker.beep(frequency=600, duration=200)

            # 讀取黑色反射值
            self.show_status(["Calibration", "Place on BLACK", "Press Center"])
            self._wait_button_press(Button.CENTER)
            black_val = self.color_sensor.reflection()
            self.ev3.speaker.beep(frequency=800, duration=200)

            # 取 1/2 白與 1/2 黑 作為閾值
            line_threshold = int((white_val + black_val) / 2)
            self.robot._line_threshold = (  # pyright: ignore[reportPrivateUsage]
                line_threshold
            )

            self.show_status(
                [
                    "Line Cal Done!",
                    "W:{} B:{}".format(int(white_val), int(black_val)),
                    "Thr:{}".format(line_threshold),
                ]
            )
            wait(1500)

        self.show_status(["Cal Color?", "OK: Press Center", "SKIP: Press UP"])
        if self._wait_button_press(Button.CENTER, Button.UP):
            # 檢查顏色

            # 讀取紅色 RGB
            self.show_status(["Calibration", "Place on RED", "Press Center"])
            self._wait_button_press(Button.CENTER)
            self._rgb_ref_red = self._rgb_multi_sampling(self.color_sensor)
            self.ev3.speaker.beep(frequency=600, duration=200)

            # 讀取黃色 RGB
            self.show_status(["Calibration", "Place on YELLOW", "Press Center"])
            self._wait_button_press(Button.CENTER)
            self._rgb_ref_yellow = self._rgb_multi_sampling(self.color_sensor)
            self.ev3.speaker.beep(frequency=600, duration=200)

            self._color_calibrated = True

            self.show_status(
                [
                    "Color Cal Done!",
                    "R:{},{},{}".format(*self._rgb_ref_red),
                    "Y:{},{},{}".format(*self._rgb_ref_yellow),
                ]
            )

    def show_status(self, lines: list[str] = []):
        """在 EV3 螢幕顯示狀態（含電量）。"""
        self.ev3.screen.clear()
        battery_mv = self.ev3.battery.voltage()
        battery_v = battery_mv / 1000.0
        lines.insert(0, "Bat: {:.1f}V".format(battery_v))
        for line in lines:
            self.ev3.screen.print(line)

    def _wait_button_press(
        self, confirmed: Button = Button.CENTER, reject: Button | None = None
    ):
        """等待指定的按鈕被按下。

        Args:
            confirmed: 按下該按鈕時返回 True。
            reject: 按下該按鈕時返回 False（可為 None 表示忽略）。

        Returns:
            按下 confirmed 時回 True；若設定 reject 且按下時回 False。
        """
        while True:
            pressed = self.ev3.buttons.pressed()
            if confirmed in pressed:
                wait(300)  # debounce
                return True
            if reject is not None and reject in pressed:
                return False
            wait(50)

    def _rgb_multi_sampling(self, sensor: ColorSensor, samples=CALIBRATION_SAMPLES):
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

    def _check_stop_color(self):
        """檢測目前感測器下方的顏色。

        若已校準顏色，使用 RGB Manhattan 距離比對；否則使用內建 color()。

        Returns:
            Color.YELLOW、Color.RED，或 None。
        """
        if self._color_calibrated:
            with self._color_sensor_lock:
                r, g, b = self._rgb_multi_sampling(
                    self.color_sensor, CALIBRATION_SAMPLES_RUNNING
                )
            # 排除黑/白地面（R 通道太低表示不是彩色線）
            if r < COLOR_MIN_R:
                return None
            # 計算與紅線參考值的 Manhattan 距離
            rr, rg, rb = self._rgb_ref_red
            dist_red = abs(r - rr) + abs(g - rg) + abs(b - rb)
            # 計算與黃線參考值的 Manhattan 距離
            yr, yg, yb = self._rgb_ref_yellow
            dist_yellow = abs(r - yr) + abs(g - yg) + abs(b - yb)
            # 取距離最近且在閾值內的顏色
            if dist_red <= dist_yellow and dist_red <= COLOR_DIST_THRESHOLD:
                return Color.RED
            if dist_yellow < dist_red and dist_yellow <= COLOR_DIST_THRESHOLD:
                return Color.YELLOW
            return None
        else:
            with self._color_sensor_lock:
                c = self.color_sensor.color()
            if c == Color.YELLOW:
                return Color.YELLOW
            elif c == Color.RED:
                return Color.RED
            return None

    def _place_parts(self):
        """啟動輸送帶將零件推落至放置區平台。

        馬達以 CONVEYOR_SPEED 正轉 CONVEYOR_RUN_TIME 毫秒後停止。
        """
        self.ev3.speaker.beep(frequency=800, duration=200)

        if self.motor_conveyor is not None:
            # 啟動輸送帶：正轉將零件往外側推落
            self.motor_conveyor.run(CONVEYOR_SPEED)
            wait(CONVEYOR_RUN_TIME)
            self.motor_conveyor.stop()
            wait(300)
        else:
            # 無輸送帶馬達：僅停車等待模擬放置
            wait(800)

    def check_battery(self):
        """檢測電池電量，低於警戒值則節拍提醒。

        Returns:
            若電量過低回 True，否則回 False。
        """
        if self.ev3.battery.voltage() < MIN_BATTERY_MV:
            self.ev3.speaker.beep(frequency=300, duration=500)
            self.show_status(["LOW BATTERY!", "Charge EV3"])
            return True
        return False

    def battery_loop(self, end_event: threading.Event, error_queue: queue.Queue):
        """電量監控迴圈。

        Args:
            end_event: 結束程式事件。
            error_queue: 執行中例外的回報佇列。
        """
        try:
            while not end_event.is_set():
                # 每等待 BATTERY_CHECK_INTERVAL 就檢查一次電量
                self.check_battery()
                wait(BATTERY_CHECK_INTERVAL)
        except Exception as ex:
            error_queue.put_nowait(ex)

    def loop(self, end_event: threading.Event, error_queue: queue.Queue):
        """主流程迴圈。

        Args:
            end_event: 結束程式事件。
            error_queue: 執行中例外的回報佇列。
        """
        try:
            yellow_count = 0

            segment_timer = StopWatch()

            while not end_event.is_set():
                # 開始新一段循跡：重置計時並確保車子可動
                segment_timer.reset()
                self.robot.start()

                # 檢查顏色與判斷
                while True:
                    self.show_status(["Running..."])

                    stop_color = self._check_stop_color()

                    if stop_color == Color.YELLOW:
                        yellow_count += 1
                        if yellow_count >= 1:  # 連續 1 次才確認為黃線
                            # 到達放置區
                            self.robot.stop(ADDITIONAL_DISTANCE_AFTER_STOP)
                            wait(300)
                            # 放置零件
                            self._place_parts()
                            self.show_status(["Placed!"])
                            wait(300)
                            # 放置完成後繼續循跡
                            self.robot.start()
                            break
                    else:
                        yellow_count = 0  # 非黃色則重置計數器

                    if stop_color == Color.RED:
                        # 回到起點
                        self.robot.stop(ADDITIONAL_DISTANCE_AFTER_STOP)
                        self.show_status(["At Start", "Waiting..."])

                        # 等待觸碰感測器啟動下一輪
                        while not self.touch_sensor.pressed():
                            self.show_status(["At Start", "Waiting..."])
                            if self.touch_sensor.pressed():
                                break
                            wait(50)

                        wait(300)
                        # 等待完啟動下一輪
                        self.robot.start()
                        break

                    # Timeout 保護：循跡過久表示可能脫軌
                    if segment_timer.time() > LINE_FOLLOW_TIMEOUT:
                        self.robot.stop()
                        self.ev3.speaker.beep(frequency=200, duration=1000)
                        self.show_status(["Timeout!", "Retrying..."])
                        wait(1000)
                        self._pid.reset()
                        segment_timer.reset()

                    wait(10)  # 循跡迴圈延遲
        except Exception as ex:
            error_queue.put_nowait(ex)

    def safe_stop(self, message="Stopped"):
        """安全停車：停止所有馬達並顯示訊息。

        Args:
            message: 顯示在螢幕上的訊息。
        """
        self._end_event.set()
        self.robot.stop()
        if self.motor_conveyor is not None:
            self.motor_conveyor.stop()
        self.ev3.speaker.beep(frequency=400, duration=300)
        self.show_status([message])


# =====================================================
#  主程式
# =====================================================
def main():
    """主程式進入點。"""
    # 初始化結束事件，用以強制結束所有循環
    end_event = threading.Event()
    # 紀錄線程內發生的錯誤
    error_queue: queue.Queue[Exception] = queue.Queue()

    main = Main(end_event)

    main.ev3.speaker.beep(frequency=500, duration=300)
    main.show_status(["Transport Robot", "Ready!", "Press Touch/Center"])

    # ---- 等待啟動 ----
    while True:
        if main.touch_sensor.pressed() or Button.CENTER in main.ev3.buttons.pressed():
            break
        wait(50)

    # 初始電量檢測
    if main.check_battery():
        wait(5000)
        # 低電量但仍允許啟動，只是警告

    # 初始化
    main.initial()

    main.ev3.speaker.beep(frequency=600, duration=200)
    wait(1000)  # 啟動延遲

    # ---- 主循環 ----

    # 初始化線程
    thread_args = (end_event, error_queue)
    threads = [
        threading.Thread(target=main.robot.loop, args=thread_args),
        threading.Thread(target=main.battery_loop, args=thread_args),
    ]
    # 主線程單獨列出來方便檢測
    main_thread = threading.Thread(target=main.loop, args=thread_args)
    threads.append(main_thread)

    # 開始所有線程
    for thread in threads:
        thread.start()

    # 等待任務結束
    # 這裡我們不在乎其他進程到底結束了沒，因為沒有意義...
    while True:
        try:
            main_thread.join(2)
        except Exception as ex:
            error_queue.put_nowait(ex)

        # 捕獲任何一條線程報錯
        if not error_queue.empty():
            end_event.set()
            ex = error_queue.get_nowait()
            main.safe_stop("ERROR!")
            main.ev3.screen.print(str(ex))
            wait(10000)
            return

        if not main_thread.is_alive():
            break

    end_event.set()  # 強制停下其他循環

    # 任務完成
    main.robot.stop()
    main.show_status(["Mission Complete!"])
    main.ev3.speaker.beep(frequency=1000, duration=1000)


if __name__ == "__main__":
    # 執行主程式
    main()
