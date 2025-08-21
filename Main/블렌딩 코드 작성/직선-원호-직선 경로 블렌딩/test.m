clear;
close all;
clc;

% =========================================================================
% 1. 시뮬레이션 파라미터 설정
% =========================================================================
speed = 10.0;             % 목표 속도 (mm/s)
dt = 0.001;               % 시간 주기 (1kHz -> 0.001s)
blend_distance = 8.0;     % 블렌딩을 위해 각 경로의 끝에서 잘라낼 거리 (mm)
epsilon = 0.5;            % RDP 허용 오차 (mm)
min_blend_angle_deg = 20.0; % 블렌딩을 적용할 최소 코너 각도 (degree)

% =========================================================================
% 2. [입력] 조밀한 원본 경로 생성 (직선-원호-직선)
% =========================================================================
p_dense = [];
p_dense = [p_dense; linspace(0,0,300)', linspace(0,50,300)', linspace(0,0,300)'];
arc_radius = 20.0;
arc_center = [p_dense(end,1) + arc_radius, p_dense(end,2), p_dense(end,3)];
theta = linspace(pi, 3*pi/2, 500);
p_dense = [p_dense; arc_center(1)+arc_radius*cos(theta)', arc_center(2)*ones(500,1), arc_center(3)+arc_radius*sin(theta)'];
last_point = p_dense(end,:);
p_dense = [p_dense; linspace(last_point(1), last_point(1)+30, 200)', linspace(last_point(2), last_point(2), 200)', linspace(last_point(3), last_point(3), 200)'];
fprintf('원본 경로: %d개의 조밀한 포인트 생성\n', size(p_dense, 1));

% =========================================================================
% 3. [1단계: 코너 감지] RDP 알고리즘으로 핵심 Waypoint 추출
% =========================================================================
simplified_indices = RDP(p_dense, epsilon);
waypoints = p_dense(simplified_indices, :);
fprintf('RDP 적용 후: %d개의 핵심 Waypoint 추출\n', size(waypoints, 1));

% =========================================================================
% 4. [2단계: 선택적 스플라인 블렌딩]
% =========================================================================
num_waypoints = size(waypoints, 1);
num_corners = num_waypoints - 2;
path_segments = {};
current_pos = waypoints(1,:);

if num_corners < 1
    disp('블렌딩할 코너가 감지되지 않았습니다.');
    path_segments{end+1} = struct('type', 'line', 'start', waypoints(1,:), 'end', waypoints(end,:));
else
    for i = 1:num_corners
        P_prev = waypoints(i, :); P_curr = waypoints(i+1, :); P_next = waypoints(i+2, :);
        U_in = (P_curr - P_prev) / norm(P_curr - P_prev);
        U_out = (P_next - P_curr) / norm(P_next - P_curr);
        
        turn_angle_rad = acos(max(min(dot(U_in, U_out), 1), -1));
        turn_angle_deg = rad2deg(turn_angle_rad);
        
        if turn_angle_deg > min_blend_angle_deg
            % --- 스플라인 블렌딩 적용 ---
            P_spline_start = P_curr - blend_distance * U_in;
            P_spline_end = P_curr + blend_distance * U_out;
            
            path_segments{end+1} = struct('type', 'line', 'start', current_pos, 'end', P_spline_start);
            
            spline_points = hermite_spline(P_spline_start, P_spline_end, U_in, U_out, 100);
            path_segments{end+1} = struct('type', 'spline', 'points', spline_points);
            
            current_pos = P_spline_end;
        end
    end
    path_segments{end+1} = struct('type', 'line', 'start', current_pos, 'end', waypoints(end,:));
end

% =========================================================================
% 5. [3단계: 리샘플링] 최종 블렌딩 경로 리샘플링
% =========================================================================
L_total = 0;
for i = 1:length(path_segments)
    if strcmp(path_segments{i}.type, 'line')
        path_segments{i}.length = norm(path_segments{i}.end - path_segments{i}.start);
    else % spline
        path_segments{i}.length = sum(vecnorm(diff(path_segments{i}.points,1,1), 2, 2));
    end
    L_total = L_total + path_segments{i}.length;
end
total_time = L_total / speed;
time_vec = (0:dt:total_time)';
dist_vec = time_vec * speed;
resampled_path = zeros(length(dist_vec), 3);
dist_cumulative = 0; segment_idx = 1;

for i = 1:length(dist_vec)
    d = dist_vec(i);
    while segment_idx < length(path_segments) && d > (dist_cumulative + path_segments{segment_idx}.length + 1e-9)
        dist_cumulative = dist_cumulative + path_segments{segment_idx}.length;
        segment_idx = segment_idx + 1;
    end
    segment = path_segments{segment_idx};
    dist_on_segment = d - dist_cumulative;
    
    if strcmp(segment.type, 'line')
        if segment.length > 1e-9, p = segment.start + (dist_on_segment / segment.length) * (segment.end - segment.start);
        else, p = segment.start; end
    else % spline
        spline_dist_cumulative = [0; cumsum(vecnorm(diff(segment.points,1,1), 2, 2))];
        p = interp1(spline_dist_cumulative, segment.points, dist_on_segment);
    end
    resampled_path(i,:) = p;
end

% =========================================================================
% 6. 결과 시각화
% =========================================================================
figure; hold on; grid on; axis equal; view(45, 25);
plot3(p_dense(:,1), p_dense(:,2), p_dense(:,3), 'c.', 'MarkerSize', 4, 'DisplayName', '원본 조밀 경로');
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'k.--', 'LineWidth', 1.5, 'MarkerSize', 15, 'DisplayName', 'RDP 추출 Waypoint');
plot3(resampled_path(:,1), resampled_path(:,2), resampled_path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', '최종 블렌딩 경로');
title('RDP + 3차 스플라인을 이용한 일반화된 블렌딩');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Location', 'best'); rotate3d on;

% =========================================================================
% RDP 알고리즘 함수
% =========================================================================
function indices = RDP(points, epsilon)
    indices = RDP_recursive(points, epsilon, 1, size(points, 1));
end
function indices = RDP_recursive(points, epsilon, start_idx, end_idx)
    d_max = 0; index = 0;
    for i = start_idx+1 : end_idx-1
        d = point_to_line_dist(points(i,:), points(start_idx,:), points(end_idx,:));
        if d > d_max, d_max = d; index = i; end
    end
    if d_max > epsilon
        rec_indices1 = RDP_recursive(points, epsilon, start_idx, index);
        rec_indices2 = RDP_recursive(points, epsilon, index, end_idx);
        indices = [rec_indices1(1:end-1); rec_indices2];
    else, indices = [start_idx; end_idx]; end
end
function dist = point_to_line_dist(p, v, w)
    l2 = sum((v - w).^2); if l2 == 0, dist = norm(p - v); return; end
    t = max(0, min(1, dot(p - v, w - v) / l2));
    projection = v + t * (w - v); dist = norm(p - projection);
end

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