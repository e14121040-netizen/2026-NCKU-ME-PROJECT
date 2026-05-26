% =====================================================
% EV3 運輸機器人 PID 循跡控制 MATLAB 模擬（修正版）
% =====================================================
clear; close all; clc;

%% 參數設定
KP = 0.6;
KI = 0.0;
KD = 0.3;

PID_OUTPUT_SCALE = -2;
BASE_SPEED = 120;      % mm/s
TURN_SLOWDOWN_RATE = 1.5;
MIN_SPEED = 20;
MIN_SPEED_MARGIN = 7;
LINE_THRESHOLD = 20;   % 0~100

WHEEL_DIAMETER = 56;   % 輪子直徑mm
AXLE_TRACK = 120;      % 左右輪中心距mm

SIM_TIME = 10;         % 模擬總時長s
DT = 0.01;             % 抽樣間隔時間s
TIME = 0:DT:SIM_TIME;
N = numel(TIME);%時間點計算

rng(1); % 固定亂數種子，讓結果一致

%% 軌道 / 感測器訊號
reflection_signal = zeros(1, N);

for i = 1:N
    t = TIME(i);

    % 前 0~3 秒：路況穩定，反射值幾乎固定
    if t < 3
        reflection_signal(i) = 15;

    % 3~6 秒：加入低頻擾動，模擬偏離黑線但偏差不大
    elseif t < 6
        reflection_signal(i) = 15 + 10*sin(2*pi*t);

    % 6 秒後：加入更大且更快的擾動，模擬更困難的路況
    else
        reflection_signal(i) = 15 + 25*sin(4*pi*t);
    end

    % 加入感測器雜訊，模擬真實世界中讀值不會完全平滑
    reflection_signal(i) = reflection_signal(i) + 2*randn();
end

reflection_signal = max(0, min(100, reflection_signal));

%% PID 狀態初始化
integral_term = 0;
last_error = 0;
integral_limit = 1000;

%% 紀錄陣列
error_history = zeros(1, N);
correction_history = zeros(1, N);
turn_rate_history = zeros(1, N);
left_speed_history = zeros(1, N);
right_speed_history = zeros(1, N);

%% 模擬循環
for i = 1:N
    reflection = reflection_signal(i);
    error = LINE_THRESHOLD - reflection;
    error_history(i) = error;

    % 離散 PID
    integral_term = integral_term + error * DT;
    integral_term = max(-integral_limit, min(integral_limit, integral_term));

    derivative = (error - last_error) / DT;
    last_error = error;

    correction = KP*error + KI*integral_term + KD*derivative;
    correction = correction * PID_OUTPUT_SCALE;

    current_speed = max(MIN_SPEED, BASE_SPEED - fix(abs(error) * TURN_SLOWDOWN_RATE));

    % 轉向限幅，避免轉向吃掉太多前進能力
    if correction > 0
        correction = min(current_speed - MIN_SPEED_MARGIN, correction);
    elseif correction < 0
        correction = max(-(current_speed - MIN_SPEED_MARGIN), correction);
    end

    correction_history(i) = correction;

    % correction -> turn_rate
    turn_rate = correction;   % deg/s
    turn_rate_history(i) = turn_rate;

    % 左右輪線速度
    omega = turn_rate * pi/180;   % rad/s
    v_left  = current_speed - omega*(AXLE_TRACK/2);
    v_right = current_speed + omega*(AXLE_TRACK/2);

    left_speed_history(i) = v_left;
    right_speed_history(i) = v_right;
end

%% RPM 計算
wheel_radius = WHEEL_DIAMETER/2/1000;   % m
left_rpm  = (left_speed_history/1000)  ./ (2*pi*wheel_radius) * 60;
right_rpm = (right_speed_history/1000) ./ (2*pi*wheel_radius) * 60;

%% 繪圖
figure('Name','EV3 PID 循跡控制模擬','Position',[100 100 1200 800]);

% ---- 圖1：感測器反射值 ----
subplot(3,2,1);
plot(TIME, reflection_signal, 'LineWidth', 1.5); hold on;
plot(TIME, LINE_THRESHOLD*ones(size(TIME)), '--', 'LineWidth', 1.2);
xlabel('時間 (秒)'); ylabel('反射值');
title('感測器反射值');
legend('反射值','閾值'); grid on;

subplot(3,2,2);
plot(TIME, error_history, 'LineWidth', 1.5);
xlabel('時間 (秒)'); ylabel('誤差');
title('誤差 e(t)'); grid on;

subplot(3,2,3);
plot(TIME, correction_history, 'LineWidth', 1.5);
xlabel('時間 (秒)'); ylabel('PID輸出');
title('PID 修正值'); grid on;

subplot(3,2,4);
plot(TIME, turn_rate_history, 'LineWidth', 1.5);
xlabel('時間 (秒)'); ylabel('deg/s');
title('轉向率'); grid on;

subplot(3,2,5);
plot(TIME, left_speed_history, 'LineWidth', 1.5); hold on;
plot(TIME, right_speed_history, 'LineWidth', 1.5);
plot(TIME, BASE_SPEED*ones(size(TIME)), '--', 'LineWidth', 1.2);
plot(TIME, MIN_SPEED*ones(size(TIME)), ':', 'LineWidth', 1.2);
xlabel('時間 (秒)'); ylabel('mm/s');
title('左右輪速度');
legend('左輪','右輪','基準速度','最低速度'); grid on;

subplot(3,2,6);
plot(TIME, left_rpm, 'LineWidth', 1.5); hold on;
plot(TIME, right_rpm, 'LineWidth', 1.5);
xlabel('時間 (秒)'); ylabel('RPM');
title('左右輪轉速');
legend('左馬達','右馬達'); grid on;

%% 統計
fprintf('=== EV3 PID 循跡控制模擬結果（修正版）===\n');
fprintf('模擬時間: %.2f s\n', SIM_TIME);
fprintf('PID: KP=%.3f, KI=%.3f, KD=%.3f\n', KP, KI, KD);
fprintf('PID_OUTPUT_SCALE=%.2f, TURN_SLOWDOWN_RATE=%.2f\n', PID_OUTPUT_SCALE, TURN_SLOWDOWN_RATE);
fprintf('BASE_SPEED=%.2f, MIN_SPEED=%.2f, MIN_SPEED_MARGIN=%.2f\n', BASE_SPEED, MIN_SPEED, MIN_SPEED_MARGIN);
fprintf('平均絕對誤差: %.3f\n', mean(abs(error_history)));
fprintf('最大絕對誤差: %.3f\n', max(abs(error_history)));
fprintf('左馬達 RPM 範圍: %.2f ~ %.2f\n', min(left_rpm), max(left_rpm));
fprintf('右馬達 RPM 範圍: %.2f ~ %.2f\n', min(right_rpm), max(right_rpm));
