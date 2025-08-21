clear;
close all;
clc;

% =========================================================================
% 1. 시뮬레이션 파라미터 설정
% =========================================================================
speed = 10.0;             % 목표 속도 (mm/s)
dt = 0.001;               % 시간 주기 (1kHz -> 0.001s)
blend_distance = 10.0;    % 블렌딩을 위해 각 경로의 끝에서 잘라낼 거리 (mm)

% =========================================================================
% 2. [수정됨] 'XY평면 직선 -> XZ평면 원호' 경로 정의
% =========================================================================
% --- 직선부 정의 (XY 평면, Z=0) ---
P_line_start = [0, 0, 0];
P_line_end   = [0, 50, 0]; % 코너점

% --- 원호부 정의 (Y=50인 XZ 평면, 아래로 볼록) ---
arc_radius = 20.0;         % 원호의 반지름
% 아래로 볼록한(+X, -Z 방향) 원호를 위해, 중심을 +X 방향으로 이동
arc_center = [P_line_end(1) + arc_radius, P_line_end(2), P_line_end(3)];
% 원호는 P_line_end 지점(180도)에서 시작하여 270도 방향으로 이동
arc_start_angle = pi;      % 180도
arc_end_angle   = 3*pi/2;  % 270도

% =========================================================================
% 3. 3차 스플라인 블렌딩 적용 (이전 코드와 로직 동일)
% =========================================================================
fprintf('--- 3차 스플라인 방식으로 블렌딩을 적용합니다 ---\n');

% --- 1. 경로 '잘라내기(Trim)' ---
T_spline_start_unit = (P_line_end - P_line_start) / norm(P_line_end - P_line_start);
P_spline_start = P_line_end - blend_distance * T_spline_start_unit;

angle_to_trim = blend_distance / arc_radius;
main_arc_new_start_angle = arc_start_angle + angle_to_trim;
p_x = arc_center(1) + arc_radius * cos(main_arc_new_start_angle);
p_y = arc_center(2);
p_z = arc_center(3) + arc_radius * sin(main_arc_new_start_angle);
P_spline_end = [p_x, p_y, p_z];

% --- 2. 스플라인 경계 조건 정의 ---
T_spline_start = T_spline_start_unit; % [0, 1, 0]
% 원호 시작점에서의 접선 벡터 (XZ 평면)
tangent_x = -arc_radius * sin(main_arc_new_start_angle);
tangent_y = 0;
tangent_z =  arc_radius * cos(main_arc_new_start_angle);
T_spline_end = [tangent_x, tangent_y, tangent_z] / norm([tangent_x, tangent_y, tangent_z]);

% --- 3. 최종 경로 세그먼트 구성 ---
seg1 = struct('type', 'line', 'start', P_line_start, 'end', P_spline_start);
seg1.length = norm(seg1.end - seg1.start);

spline_points = hermite_spline(P_spline_start, P_spline_end, T_spline_start, T_spline_end, 100);
seg2 = struct('type', 'spline', 'points', spline_points);
seg2.length = sum(vecnorm(diff(spline_points,1,1), 2, 2));

seg3 = struct('type', 'main_arc', 'center', arc_center, 'radius', arc_radius, ...
    'start_angle', main_arc_new_start_angle, 'end_angle', arc_end_angle);
end_angle_normalized = atan2(sin(seg3.end_angle), cos(seg3.end_angle));
delta_angle = end_angle_normalized - seg3.start_angle;
if delta_angle < 0, delta_angle = delta_angle + 2*pi; end
seg3.length = seg3.radius * delta_angle;

% --- 전체 경로 리샘플링 ---
L_total = seg1.length + seg2.length + seg3.length;
total_time = L_total / speed;
time_vec = (0:dt:total_time)';
dist_vec = time_vec * speed;
resampled_path = zeros(length(dist_vec), 3);

for i = 1:length(dist_vec)
    d = dist_vec(i);
    if d <= seg1.length
        p = seg1.start + (d / seg1.length) * (seg1.end - seg1.start);
    elseif d <= (seg1.length + seg2.length)
        dist_on_spline = d - seg1.length;
        spline_dist_cumulative = [0; cumsum(vecnorm(diff(seg2.points,1,1), 2, 2))];
        p = interp1(spline_dist_cumulative, seg2.points, dist_on_spline);
    else
        dist_on_arc = d - seg1.length - seg2.length;
        angle_on_arc = dist_on_arc / seg3.radius;
        current_angle = seg3.start_angle + angle_on_arc;
        % XZ 평면에서 원호 생성
        p_x = seg3.center(1) + seg3.radius * cos(current_angle);
        p_y = seg3.center(2); % Y는 고정
        p_z = seg3.center(3) + seg3.radius * sin(current_angle);
        p = [p_x, p_y, p_z];
    end
    resampled_path(i, :) = p;
end

% =========================================================================
% 4. 결과 시각화
% =========================================================================
figure; hold on; grid on; axis equal; view(45, 25);
% 원본 경로
plot3([P_line_start(1), P_line_end(1)], [P_line_start(2), P_line_end(2)], [P_line_start(3), P_line_end(3)], 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 직선부(XY평면)');
theta_vis = linspace(arc_start_angle, arc_end_angle, 100);
plot3(arc_center(1)+arc_radius*cos(theta_vis), arc_center(2)*ones(1,100), arc_center(3)+arc_radius*sin(theta_vis), 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 원호부(XZ평면)');
% 최종 경로
plot3(resampled_path(:,1), resampled_path(:,2), resampled_path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', '최종 블렌딩 경로');
plot3(P_spline_start(1), P_spline_start(2), P_spline_start(3), 'go', 'MarkerFaceColor','g', 'DisplayName','블렌드 시작');
plot3(P_spline_end(1), P_spline_end(2), P_spline_end(3), 'bo', 'MarkerFaceColor','b', 'DisplayName','블렌드 끝');
title('XY평면 직선 -> XZ평면 원호 블렌딩');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Location', 'best'); rotate3d on;

% =========================================================================
% 3차 에르미트 스플라인 생성 함수 (3D)
% =========================================================================
function points = hermite_spline(P0, P1, T0, T1, num_points)
    t = linspace(0, 1, num_points);
    h00 = 2*t.^3 - 3*t.^2 + 1; h10 = t.^3 - 2*t.^2 + t;
    h01 = -2*t.^3 + 3*t.^2; h11 = t.^3 - t.^2;
    scale = norm(P1 - P0);
    T0_scaled = scale * T0; T1_scaled = scale * T1;
    points_x = h00.*P0(1) + h10.*T0_scaled(1) + h01.*P1(1) + h11.*T1_scaled(1);
    points_y = h00.*P0(2) + h10.*T0_scaled(2) + h01.*P1(2) + h11.*T1_scaled(2);
    points_z = h00.*P0(3) + h10.*T0_scaled(3) + h01.*P1(3) + h11.*T1_scaled(3);
    points = [points_x', points_y', points_z'];
end