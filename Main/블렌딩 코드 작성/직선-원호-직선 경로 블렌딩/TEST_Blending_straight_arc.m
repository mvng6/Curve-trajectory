clear;
close all;
clc;

% =========================================================================
% 1. 시뮬레이션 파라미터 설정
% =========================================================================
speed = 5.0;              % 목표 속도 (mm/s)
dt = 0.001;               % 시간 주기 (1kHz -> 0.001s)
blend_radius = 10.0;       % 블렌딩 반경 (mm)

% [핵심 파라미터] RDP 알고리즘의 허용 오차 (epsilon)
% 이 값이 작을수록 -> 더 많은 코너를 감지 (경로가 복잡해짐)
% 이 값이 클수록 -> 더 적은 코너를 감지 (경로가 단순화됨)
epsilon = 5.0; % 1.0mm

% =========================================================================
% 2. [가정] 무수히 많은 점으로 이루어진 원본 경로 생성
% =========================================================================
% 'ㄹ'자 형태의 경로를 촘촘한 점(1000개)으로 생성
p_dense = [];
p_dense = [p_dense; linspace(0,0,250)', linspace(0,50,250)', linspace(0,0,250)'];
p_dense = [p_dense; linspace(0,50,250)', linspace(50,50,250)', linspace(0,-10,250)'];
p_dense = [p_dense; linspace(50,50,250)', linspace(50,100,250)', linspace(-10,-10,250)'];
p_dense = [p_dense; linspace(50,100,250)', linspace(100,100,250)', linspace(-10,-5,250)'];
p_dense = [p_dense; linspace(100,100,250)', linspace(100,-50,250)', linspace(-5,-5,250)'];
p_dense = [p_dense; linspace(100,150,250)', linspace(-50,-50,250)', linspace(-5,-5,250)'];

fprintf('원본 경로: %d개의 조밀한 포인트 생성\n', size(p_dense, 1));

% =========================================================================
% 3. [코너 감지] RDP 알고리즘을 이용한 경로 단순화
% =========================================================================
simplified_indices = RDP(p_dense, epsilon);
waypoints = p_dense(simplified_indices, :);

fprintf('RDP 적용 후: %d개의 핵심 Waypoint 추출\n', size(waypoints, 1));

% =========================================================================
% 4. [블렌딩] 추출된 Waypoint에 다중 코너 블렌딩 적용
% (이전 답변의 '다중 코너 블렌딩' 코드와 동일)
% =========================================================================
num_waypoints = size(waypoints, 1);
num_corners = num_waypoints - 2;

blend_starts = zeros(num_corners, 3);
blend_ends = zeros(num_corners, 3);
arc_centers = zeros(num_corners, 3);
arc_angles = zeros(num_corners, 1);
arc_axes = zeros(num_corners, 3);

for i = 1:num_corners
    P_prev = waypoints(i, :); P_curr = waypoints(i+1, :); P_next = waypoints(i+2, :);
    U_in = (P_curr - P_prev) / norm(P_curr - P_prev);
    U_out = (P_next - P_curr) / norm(P_next - P_curr);
    min_dist = min(norm(P_curr-P_prev)/2, norm(P_next-P_curr)/2);
    current_radius = min(blend_radius, min_dist);
    blend_starts(i,:) = P_curr - current_radius * U_in;
    blend_ends(i,:) = P_curr + current_radius * U_out;
    arc_centers(i,:) = blend_starts(i,:) + current_radius * U_out;
    arc_angles(i) = acos(max(min(dot(U_in, -U_out), 1), -1));
    axis = cross(U_in, U_out);
    arc_axes(i,:) = axis / norm(axis);
end

path_segments = {};
current_pos = waypoints(1,:);
path_segments{end+1} = struct('type', 'line', 'start', current_pos, 'end', blend_starts(1,:));
current_pos = blend_starts(1,:);
for i = 1:num_corners
    path_segments{end+1} = struct('type', 'arc', 'start', current_pos, 'end', blend_ends(i,:), ...
        'center', arc_centers(i,:), 'radius', min(blend_radius, min_dist), 'angle', arc_angles(i), 'axis', arc_axes(i,:));
    current_pos = blend_ends(i,:);
    if i < num_corners
        path_segments{end+1} = struct('type', 'line', 'start', current_pos, 'end', blend_starts(i+1,:));
        current_pos = blend_starts(i+1,:);
    end
end
path_segments{end+1} = struct('type', 'line', 'start', current_pos, 'end', waypoints(end,:));

L_total = 0;
for i = 1:length(path_segments)
    if strcmp(path_segments{i}.type, 'line')
        path_segments{i}.length = norm(path_segments{i}.end - path_segments{i}.start);
    else, path_segments{i}.length = path_segments{i}.radius * path_segments{i}.angle; end
    L_total = L_total + path_segments{i}.length;
end

total_time = L_total / speed;
time_vec = (0:dt:total_time)';
dist_vec = time_vec * speed;
resampled_path = zeros(length(dist_vec), 3);
dist_cumulative = 0;
segment_idx = 1;
for i = 1:length(dist_vec)
    d = dist_vec(i);
    while d > (dist_cumulative + path_segments{segment_idx}.length + 1e-9)
        dist_cumulative = dist_cumulative + path_segments{segment_idx}.length;
        segment_idx = segment_idx + 1;
    end
    segment = path_segments{segment_idx};
    dist_on_segment = d - dist_cumulative;
    if strcmp(segment.type, 'line')
        p = segment.start + (dist_on_segment / segment.length) * (segment.end - segment.start);
    else
        angle_on_arc = dist_on_segment / segment.radius;
        v_start = segment.start - segment.center;
        v_rotated = v_start * cos(angle_on_arc) + cross(segment.axis, v_start) * sin(angle_on_arc) + ...
                    segment.axis * dot(segment.axis, v_start) * (1 - cos(angle_on_arc));
        p = segment.center + v_rotated;
    end
    resampled_path(i,:) = p;
end

% =========================================================================
% 5. 결과 시각화
% =========================================================================
figure; hold on; grid on; axis equal; view(3);
plot3(p_dense(:,1), p_dense(:,2), p_dense(:,3), 'c.', 'MarkerSize', 2, 'DisplayName', '원본 조밀 경로');
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'k.--', 'LineWidth', 1.5, 'MarkerSize', 15, 'DisplayName', 'RDP 추출 Waypoint');
plot3(resampled_path(:,1), resampled_path(:,2), resampled_path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', '최종 블렌딩 경로');
title('RDP를 이용한 코너 감지 및 블렌딩'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Location', 'best'); rotate3d on;
