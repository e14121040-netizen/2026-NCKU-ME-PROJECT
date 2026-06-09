% =====================================================
% EV3 運輸機器人 PID 循跡控制 MATLAB 模擬：圓角矩形地圖
% =====================================================
% 地圖外框總長為 x、總寬為 y，四個角為 90 度圓角 r 的閉合黑線。
% 因此水平直線段長度為 x-2r，垂直直線段長度為 y-2r。
% 本模型把黑線畫成中心線，顏色感測器位於車體前方，感測器距離黑線
% 中心越近反射值越低，距離越遠反射值越高。
clear; close all; clc;

%% PID / 車體參數
KP = 0.6;
KI = 0.0;
KD = 0.3;

PID_OUTPUT_SCALE = -2;
BASE_SPEED = 120;      % mm/s
TURN_SLOWDOWN_RATE = 1.5;
MIN_SPEED = 20;
MIN_SPEED_MARGIN = 7;
LINE_THRESHOLD = 40;   % 0~100

WHEEL_DIAMETER = 56;   % 輪子直徑 mm
AXLE_TRACK = 120;      % 左右輪中心距 mm

%% 圓角矩形地圖參數
MAP_X = 3000;          % 圓角矩形外框總長度 mm
MAP_Y = 590;           % 圓角矩形外框總寬度 mm
MAP_R = 180;           % 90 度圓角半徑 mm

LINE_HALF_WIDTH = 10;  % 黑線半寬 mm (線寬 20mm)
SENSOR_HALF_WIDTH = 10; % 感測器半寬 mm
SENSOR_FORWARD = 80;    % 感測器在車體中心前方距離 mm
SENSOR_LATERAL = 56;   % 感測器在車體左側距離 mm

BLACK_REF = 10;        % 感測器在黑線中心附近的反射值
WHITE_REF = 70;        % 感測器在白色底板上的反射值
SENSOR_NOISE_STD = 1.5;

%% 任務標線 / 放置區參數
% 路徑座標 s 從下方長邊左側切點開始，逆時針遞增。
% 假設：
% - 紅線位於起點 s=0。
% - 放置區 2 位於下方長邊中間。
% - 放置區 1 與 3 分別在放置區 2 前後 600 mm。
% - 放置區 4 位於右側短邊中間。
YELLOW_GAP_FROM_ZONE2 = 600;   % 放置區 1/3 與 2 的路徑距離 mm
MARKER_HALF_LENGTH = 10;       % 紅/黃橫線沿路徑方向半寬 mm (線寬 20mm)
MARKER_MAX_LINE_DISTANCE = 100; % 標線左右長度 100mm
STOP_WAIT_TIME = 6.6;          % 停下放置零件的模擬等待時間 s
START_WAIT_TIME = 5.0;         % 回到紅線等待下一圈啟動 s
MARKER_COOLDOWN_S = 2.0;       % 避免同一條標線連續觸發
YELLOW_FORWARD_DISTANCE = 40;  % 偵測到黃線後繼續循線前進的距離 mm
LOST_DISTANCE = 100;           % 脫線判定距離 (mm)
MAP_MAX_DEV = 150;             % 離軌跡太遠強制終止距離 (mm)
SIM_LAPS = 4;

RGB_REF_RED = [54, 8, 0];
RGB_REF_YELLOW = [77, 40, 4];
RGB_TOL = 15;
RED_REFLECTION = 28;
YELLOW_REFLECTION = 55;

MAX_SIM_TIME = 260;    % 安全上限；實際會在繞完 4 圈或首次脫線時停止
DT = 0.01;             % 抽樣間隔時間 s
TIME = 0:DT:MAX_SIM_TIME;
N = numel(TIME);

rng(1);

%% 建立圓角矩形中心線取樣點
[track_x, track_y] = rounded_rectangle_track(MAP_X, MAP_Y, MAP_R, 3);
track_points = [track_x(:), track_y(:)];
[track_s, track_length] = cumulative_track_distance(track_points);

zone_s = build_zone_positions(MAP_X, MAP_Y, MAP_R, YELLOW_GAP_FROM_ZONE2);

straight_x_len = MAP_X - 2*MAP_R;
straight_y_len = MAP_Y - 2*MAP_R;
quarter_arc_len = pi/2*MAP_R;
red_s = straight_x_len + quarter_arc_len + straight_y_len + quarter_arc_len + straight_x_len + quarter_arc_len + straight_y_len/2;

fprintf('放置區路徑座標 s: 1=%.1f, 2=%.1f, 3=%.1f, 4=%.1f mm\n', zone_s);

%% 初始車體姿態
% 初始感測器中心位置是在線內側邊緣上 (+10mm 往地圖中心)
start_sensor_x = -MAP_X/2 + 10;
start_sensor_y = 0;

% 車體平行於黑線前進 (沿 -y 方向)
theta = -pi/2; 

% 感測器位在車體左側 56mm，因此計算出車體位置會在線外 46mm
robot_x = start_sensor_x - SENSOR_FORWARD*cos(theta) + SENSOR_LATERAL*sin(theta);
robot_y = start_sensor_y - SENSOR_FORWARD*sin(theta) - SENSOR_LATERAL*cos(theta);

%% PID 狀態初始化
integral_term = 0;
last_error = 0;
integral_limit = 1000;

%% 紀錄陣列
robot_history = zeros(N, 3);
sensor_history = zeros(N, 2);
distance_history = zeros(1, N);
reflection_history = zeros(1, N);
error_history = zeros(1, N);
correction_history = zeros(1, N);
turn_rate_history = zeros(1, N);
left_speed_history = zeros(1, N);
right_speed_history = zeros(1, N);
lost_history = false(1, N);
path_s_history = zeros(1, N);
lap_history = zeros(1, N);
zone_history = zeros(1, N);
event_history = strings(1, N);
rgb_history = zeros(N, 3);

round_finish = 0;
zones_visited_current = 0;
zones_placed_all = 0;
last_marker_time = -Inf;
pause_until = -Inf;
stop_count_by_zone = zeros(1, 4);
marker_active = false;
simulation_failed = false;
placed_zone_done = false(1, 4);
stop_idx = N;
stop_reason = "達到最大模擬時間";

robot_distance_traveled = 0;
yellow_forward_target = -Inf;

%% 模擬循環
for i = 1:N
    sensor_x = robot_x + SENSOR_FORWARD*cos(theta) - SENSOR_LATERAL*sin(theta);
    sensor_y = robot_y + SENSOR_FORWARD*sin(theta) + SENSOR_LATERAL*cos(theta);
    sensor_history(i, :) = [sensor_x, sensor_y];
    robot_history(i, :) = [robot_x, robot_y, theta];

    [line_distance, nearest_idx] = min_distance_to_track(sensor_x, sensor_y, track_points);
    current_s = track_s(nearest_idx);
    distance_history(i) = line_distance;
    path_s_history(i) = current_s;
    lap_history(i) = round_finish + 1;

    reflection = distance_to_reflection( ...
        line_distance, LINE_HALF_WIDTH, SENSOR_HALF_WIDTH, BLACK_REF, WHITE_REF);
    marker_color = detect_marker(current_s, line_distance, red_s, zone_s, ...
        track_length, MARKER_HALF_LENGTH, MARKER_MAX_LINE_DISTANCE);

    if marker_color == "red"
        rgb = RGB_REF_RED + (2*rand(1,3)-1)*RGB_TOL;
        reflection = RED_REFLECTION;
    elseif marker_color == "yellow"
        rgb = RGB_REF_YELLOW + (2*rand(1,3)-1)*RGB_TOL;
        reflection = YELLOW_REFLECTION;
    else
        rgb = [reflection, reflection, reflection];
    end

    reflection = reflection + SENSOR_NOISE_STD*randn();
    reflection = max(0, min(100, reflection));
    reflection_history(i) = reflection;
    rgb_history(i, :) = max(0, rgb);

    if marker_color == ""
        marker_active = false;
    end

    if TIME(i) < pause_until
        error_history(i) = 0;
        correction_history(i) = 0;
        turn_rate_history(i) = 0;
        left_speed_history(i) = 0;
        right_speed_history(i) = 0;
        robot_history(i, :) = [robot_x, robot_y, theta];
        lost_history(i) = line_distance > LOST_DISTANCE;
        continue;
    end

    if yellow_forward_target > 0 && robot_distance_traveled >= yellow_forward_target
        yellow_forward_target = -Inf;
        event_history(i) = "yellow stop & place";
        pause_until = TIME(i) + STOP_WAIT_TIME;
        integral_term = 0;
        last_error = 0;
    end

    detected_color = classify_rgb(rgb_history(i, :), RGB_REF_RED, RGB_REF_YELLOW, RGB_TOL);
    if detected_color ~= "" && ~marker_active && TIME(i) - last_marker_time >= MARKER_COOLDOWN_S
        marker_active = true;
        last_marker_time = TIME(i);

        if detected_color == "red"
            integral_term = 0;
            last_error = 0;
            event_history(i) = "red reset";
            if zones_visited_current > 0
                round_finish = round_finish + 1;
                zones_visited_current = 0;
            end

            if round_finish >= SIM_LAPS
                stop_idx = i;
                stop_reason = sprintf("完成 %d 圈", SIM_LAPS);
                break;
            end

            pause_until = TIME(i) + START_WAIT_TIME;
        elseif detected_color == "yellow"
            zone_id = nearest_zone_id(current_s, zone_s, track_length);
            zone_history(i) = zone_id;
            zones_visited_current = zones_visited_current + 1;

            if zone_id == round_finish + 1 && ~placed_zone_done(zone_id)
                zones_placed_all = zones_placed_all + 1;
                placed_zone_done(zone_id) = true;
                stop_count_by_zone(zone_id) = stop_count_by_zone(zone_id) + 1;
                event_history(i) = "yellow detected";
                yellow_forward_target = robot_distance_traveled + YELLOW_FORWARD_DISTANCE;
            else
                event_history(i) = "yellow pass";
            end
        end
    end

    error = LINE_THRESHOLD - reflection;
    error_history(i) = error;

    % 離散 PID，寫法對齊 transport_robot.py (無乘除 DT)
    integral_term = integral_term + error;
    integral_term = max(-integral_limit, min(integral_limit, integral_term));

    derivative = error - last_error;
    last_error = error;

    correction = KP*error + KI*integral_term + KD*derivative;
    correction = correction * PID_OUTPUT_SCALE;

    current_speed = max(MIN_SPEED, BASE_SPEED - fix(abs(error) * TURN_SLOWDOWN_RATE));

    if correction > 0
        correction = min(current_speed - MIN_SPEED_MARGIN, correction);
    elseif correction < 0
        correction = max(-(current_speed - MIN_SPEED_MARGIN), correction);
    end

    correction_history(i) = correction;

    turn_rate = correction;   % deg/s，對應 robot.drive(speed, turn_rate)
    turn_rate_history(i) = turn_rate;

    omega = turn_rate * pi/180;   % rad/s
    % Pybricks DriveBase: turn_rate > 0 表示向右轉 (v_left > v_right)
    v_left  = current_speed + omega*(AXLE_TRACK/2);
    v_right = current_speed - omega*(AXLE_TRACK/2);

    left_speed_history(i) = v_left;
    right_speed_history(i) = v_right;

    robot_history(i, :) = [robot_x, robot_y, theta];
    lost_history(i) = line_distance > LOST_DISTANCE;

    if lost_history(i) && ~simulation_failed
        simulation_failed = true;
        event_history(i) = "line lost";
        stop_idx = i;
        stop_reason = "首次脫線";
        break;
    end

    % 差速車簡化運動學
    robot_x = robot_x + current_speed*cos(theta)*DT;
    robot_y = robot_y + current_speed*sin(theta)*DT;
    % Pybricks turn_rate > 0 (向右轉) 代表數學 heading 角度減少
    theta = wrapToPiLocal(theta - omega*DT);
    robot_distance_traveled = robot_distance_traveled + current_speed*DT;
end

%% 裁切到實際模擬停止時間
valid_idx = 1:stop_idx;
TIME = TIME(valid_idx);
robot_history = robot_history(valid_idx, :);
sensor_history = sensor_history(valid_idx, :);
distance_history = distance_history(valid_idx);
reflection_history = reflection_history(valid_idx);
error_history = error_history(valid_idx);
correction_history = correction_history(valid_idx);
turn_rate_history = turn_rate_history(valid_idx);
left_speed_history = left_speed_history(valid_idx);
right_speed_history = right_speed_history(valid_idx);
lost_history = lost_history(valid_idx);
path_s_history = path_s_history(valid_idx);
lap_history = lap_history(valid_idx);
zone_history = zone_history(valid_idx);
event_history = event_history(valid_idx);
rgb_history = rgb_history(valid_idx, :);

%% RPM 計算
wheel_radius = WHEEL_DIAMETER/2/1000;   % m
left_rpm  = (left_speed_history/1000)  ./ (2*pi*wheel_radius) * 60;
right_rpm = (right_speed_history/1000) ./ (2*pi*wheel_radius) * 60;

%% 循線能力評估
required_corner_turn_rate = BASE_SPEED / MAP_R * 180/pi;
max_commanded_turn_rate = max(abs(turn_rate_history));
lost_indices = find(lost_history);
is_lost = ~isempty(lost_indices);

if is_lost
    first_lost_time = TIME(lost_indices(1));
else
    first_lost_time = NaN;
end

%% 繪圖
figure('Name','EV3 PID 圓角矩形循跡模擬','Position',[100 100 1300 850]);

subplot(3,3,[1 4 7]);
plot(track_y, track_x, 'k-', 'LineWidth', 3); hold on;
plot(sensor_history(:,2), sensor_history(:,1), 'r-', 'LineWidth', 1.2);
plot(robot_history(:,2), robot_history(:,1), 'b-', 'LineWidth', 1.0);
plot(robot_history(1,2), robot_history(1,1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
plot(robot_history(end,2), robot_history(end,1), 'mx', 'LineWidth', 2, 'MarkerSize', 9);
arrow_idx = 1:max(1, round(numel(TIME)/35)):numel(TIME);
quiver(robot_history(arrow_idx,2), robot_history(arrow_idx,1), ...
    80*sin(robot_history(arrow_idx,3)), 80*cos(robot_history(arrow_idx,3)), ...
    0, 'Color', [0.1 0.25 0.8], 'LineWidth', 0.8, 'MaxHeadSize', 1.5);
scatter_marker_positions([track_points(:,2), track_points(:,1)], track_s, red_s, zone_s, track_length);
axis equal; grid on;
xlabel('y (mm)'); ylabel('x (mm)');
title('圓角矩形地圖與機器人軌跡');
legend('黑線中心','感測器軌跡','車體中心','起點','停止點','車體方向','紅線','黃線','Location','best');

subplot(3,3,2);
plot(TIME, reflection_history, 'LineWidth', 1.3); hold on;
plot(TIME, LINE_THRESHOLD*ones(size(TIME)), '--', 'LineWidth', 1.1);
xlabel('時間 (秒)'); ylabel('反射值');
title('感測器反射值'); grid on;
legend('反射值','閾值');

subplot(3,3,3);
plot(TIME, distance_history, 'LineWidth', 1.3); hold on;
plot(TIME, LOST_DISTANCE*ones(size(TIME)), 'r--', 'LineWidth', 1.1);
xlabel('時間 (秒)'); ylabel('mm');
title('感測器至黑線中心距離'); grid on;
legend('距離','失線判定');

subplot(3,3,5);
plot(TIME, error_history, 'LineWidth', 1.3);
xlabel('時間 (秒)'); ylabel('誤差');
title('誤差 e(t)'); grid on;

subplot(3,3,6);
plot(TIME, correction_history, 'LineWidth', 1.3);
xlabel('時間 (秒)'); ylabel('PID輸出');
title('PID 修正值'); grid on;

subplot(3,3,8);
plot(TIME, turn_rate_history, 'LineWidth', 1.3); hold on;
plot(TIME, required_corner_turn_rate*ones(size(TIME)), 'k--', 'LineWidth', 1.1);
xlabel('時間 (秒)'); ylabel('deg/s');
title('轉向率'); grid on;
legend('命令轉向率','圓角理論需求');

subplot(3,3,9);
plot(TIME, left_rpm, 'LineWidth', 1.3); hold on;
plot(TIME, right_rpm, 'LineWidth', 1.3);
xlabel('時間 (秒)'); ylabel('RPM');
title('左右輪轉速'); grid on;
legend('左馬達','右馬達');

%% 統計
fprintf('=== EV3 PID 圓角矩形循跡模擬結果 ===\n');
fprintf('地圖: 外框總長 x=%.1f mm, 外框總寬 y=%.1f mm, 圓角 r=%.1f mm\n', MAP_X, MAP_Y, MAP_R);
fprintf('直線段: 水平 %.1f mm, 垂直 %.1f mm\n', MAP_X - 2*MAP_R, MAP_Y - 2*MAP_R);
fprintf('路徑總長: %.1f mm\n', track_length);
fprintf('實際模擬時間: %.2f s, DT=%.3f s\n', TIME(end), DT);
fprintf('停止原因: %s\n', stop_reason);
fprintf('PID: KP=%.3f, KI=%.3f, KD=%.3f, PID_OUTPUT_SCALE=%.2f\n', KP, KI, KD, PID_OUTPUT_SCALE);
fprintf('BASE_SPEED=%.2f mm/s, LINE_THRESHOLD=%.2f\n', BASE_SPEED, LINE_THRESHOLD);
fprintf('平均感測器離線距離: %.3f mm\n', mean(distance_history));
fprintf('最大感測器離線距離: %.3f mm\n', max(distance_history));
fprintf('平均絕對誤差: %.3f\n', mean(abs(error_history)));
fprintf('最大絕對誤差: %.3f\n', max(abs(error_history)));
fprintf('圓角理論需求轉向率: %.2f deg/s\n', required_corner_turn_rate);
fprintf('模擬中最大命令轉向率: %.2f deg/s\n', max_commanded_turn_rate);
fprintf('左馬達 RPM 範圍: %.2f ~ %.2f\n', min(left_rpm), max(left_rpm));
fprintf('右馬達 RPM 範圍: %.2f ~ %.2f\n', min(right_rpm), max(right_rpm));
fprintf('放置停止次數: zone1=%d, zone2=%d, zone3=%d, zone4=%d\n', stop_count_by_zone);
fprintf('完成放置總數: %d\n', zones_placed_all);

if is_lost
    fprintf('判斷: 可能會中途跑掉，%.2f 秒時感測器距離黑線中心首次超過 %.1f mm。\n', ...
        first_lost_time, LOST_DISTANCE);
else
    fprintf('判斷: 在此參數下沒有超過失線距離，理論上可正常循線。\n');
end

if required_corner_turn_rate > max(BASE_SPEED - MIN_SPEED_MARGIN, max_commanded_turn_rate)
    fprintf('提醒: 圓角半徑偏小或速度偏高，彎道需求轉向率接近/超過控制上限。\n');
end

%% 區域函式
function [x, y] = rounded_rectangle_track(total_x, total_y, radius, ds)
    if radius <= 0
        error('MAP_R 必須大於 0。');
    end
    if total_x <= 2*radius || total_y <= 2*radius
        error('MAP_X 與 MAP_Y 是外框總長寬，必須都大於 2*MAP_R。');
    end

    straight_x = total_x - 2*radius;
    straight_y = total_y - 2*radius;

    n_line_x = max(2, ceil(straight_x/ds));
    n_line_y = max(2, ceil(straight_y/ds));
    n_arc = max(8, ceil((pi/2*radius)/ds));

    cx = total_x/2 - radius;
    cy = total_y/2 - radius;

    % 逆時針：下直線 -> 右下圓角 -> 右直線 -> 右上圓角 ...
    xb = linspace(-cx, cx, n_line_x);
    yb = -total_y/2 * ones(size(xb));

    a1 = linspace(-pi/2, 0, n_arc);
    x1 = cx + radius*cos(a1);
    y1 = -cy + radius*sin(a1);

    yr = linspace(-cy, cy, n_line_y);
    xr = total_x/2 * ones(size(yr));

    a2 = linspace(0, pi/2, n_arc);
    x2 = cx + radius*cos(a2);
    y2 = cy + radius*sin(a2);

    xt = linspace(cx, -cx, n_line_x);
    yt = total_y/2 * ones(size(xt));

    a3 = linspace(pi/2, pi, n_arc);
    x3 = -cx + radius*cos(a3);
    y3 = cy + radius*sin(a3);

    yl = linspace(cy, -cy, n_line_y);
    xl = -total_x/2 * ones(size(yl));

    a4 = linspace(pi, 3*pi/2, n_arc);
    x4 = -cx + radius*cos(a4);
    y4 = -cy + radius*sin(a4);

    x = [xb, x1, xr, x2, xt, x3, xl, x4];
    y = [yb, y1, yr, y2, yt, y3, yl, y4];
end

function [d, idx] = min_distance_to_track(px, py, points)
    dx = points(:,1) - px;
    dy = points(:,2) - py;
    [d, idx] = min(sqrt(dx.^2 + dy.^2));
end

function reflection = distance_to_reflection(distance, line_half_width, sensor_half_width, black_ref, white_ref)
    % 以圓形感測器計算面積重疊比例 (S-curve) 以更符合真實物理感測狀況
    R = sensor_half_width;
    h = abs(distance - line_half_width);
    
    if distance <= line_half_width - R
        white_ratio = 0;
    elseif distance >= line_half_width + R
        white_ratio = 1;
    else
        theta = 2 * acos(h / R);
        segment_area = 0.5 * R^2 * (theta - sin(theta));
        circle_area = pi * R^2;
        
        if distance < line_half_width
            white_ratio = segment_area / circle_area;
        else
            white_ratio = 1 - (segment_area / circle_area);
        end
    end
    
    reflection = black_ref + white_ratio * (white_ref - black_ref);
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function [s, track_length] = cumulative_track_distance(points)
    segment = sqrt(sum(diff(points, 1, 1).^2, 2));
    close_segment = norm(points(1,:) - points(end,:));
    s = [0; cumsum(segment)];
    track_length = s(end) + close_segment;
end

function zone_s = build_zone_positions(total_x, total_y, radius, gap_from_zone2)
    straight_x = total_x - 2*radius;
    straight_y = total_y - 2*radius;
    bottom_straight = straight_x;
    quarter_arc = pi/2*radius;

    zone2 = bottom_straight/2;
    zone1 = zone2 - gap_from_zone2;
    zone3 = zone2 + gap_from_zone2;
    zone4 = bottom_straight + quarter_arc + straight_y/2;

    zone_s = [zone1, zone2, zone3, zone4];
end

function marker_color = detect_marker(current_s, line_distance, red_s, zone_s, ...
        track_length, marker_half_length, marker_max_line_distance)
    marker_color = "";
    if line_distance > marker_max_line_distance
        return;
    end

    if circular_distance(current_s, red_s, track_length) <= marker_half_length
        marker_color = "red";
        return;
    end

    for k = 1:numel(zone_s)
        if circular_distance(current_s, zone_s(k), track_length) <= marker_half_length
            marker_color = "yellow";
            return;
        end
    end
end

function color_name = classify_rgb(rgb, red_ref, yellow_ref, tol)
    color_name = "";
    dist_red = sum(abs(rgb - red_ref));
    dist_yellow = sum(abs(rgb - yellow_ref));
    threshold = 3*tol;

    if dist_red <= dist_yellow && dist_red <= threshold
        color_name = "red";
    elseif dist_yellow < dist_red && dist_yellow <= threshold
        color_name = "yellow";
    end
end

function zone_id = nearest_zone_id(current_s, zone_s, track_length)
    distances = zeros(size(zone_s));
    for k = 1:numel(zone_s)
        distances(k) = circular_distance(current_s, zone_s(k), track_length);
    end
    [~, zone_id] = min(distances);
end

function d = circular_distance(a, b, period)
    raw = abs(a - b);
    d = min(raw, period - raw);
end

function scatter_marker_positions(points, track_s, red_s, zone_s, track_length)
    red_idx = nearest_s_index(track_s, red_s, track_length);
    yellow_idx = zeros(size(zone_s));
    for k = 1:numel(zone_s)
        yellow_idx(k) = nearest_s_index(track_s, zone_s(k), track_length);
    end

    scatter(points(red_idx,1), points(red_idx,2), 80, 'r', 'filled');
    scatter(points(yellow_idx,1), points(yellow_idx,2), 60, [1 0.75 0], 'filled');

    for k = 1:numel(zone_s)
        text(points(yellow_idx(k),1), points(yellow_idx(k),2), ...
            sprintf('  %d', k), 'Color', [0.45 0.32 0], 'FontWeight', 'bold');
    end
end

function idx = nearest_s_index(track_s, target_s, track_length)
    ds = abs(track_s - target_s);
    ds = min(ds, track_length - ds);
    [~, idx] = min(ds);
end
