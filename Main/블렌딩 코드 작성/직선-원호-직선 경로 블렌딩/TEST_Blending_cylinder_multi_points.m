clear;
close all;
clc;

% =========================================================================
% 1. 시뮬레이션 파라미터 및 테스트 경로 생성
% =========================================================================
blend_distance = 20.0; % 블렌딩을 위해 코너로부터 잘라낼 거리 (mm)
corner_angle_threshold = 90; % 코너로 판단할 최소 각도 (degrees)

fprintf('--- 테스트를 위한 촘촘한 경로점을 생성합니다 ---\n');

P_line_start = [0, 0, 0];
P_line_end = [0, 50, 0];

path_len = 100;
path_1 = zeros(3,path_len);
path_1(2,:) = linspace(P_line_start(2),P_line_end(2),path_len);

arc_radius = 20.0;
arc_center = [path_1(1,end) + arc_radius, path_1(2,end), path_1(3,end)];

arc_start_angle = pi;
arc_end_angle = 3*pi/2;

theta_vis = linspace(arc_start_angle, arc_end_angle, path_len);
path_2 = [arc_center(1) + arc_radius*cos(theta_vis); 
    arc_center(2)*ones(1,100);
    arc_center(3)+arc_radius*sin(theta_vis)];

original_path = [path_1, path_2]';

fprintf('총 %d개의 경로점으로 구성된 원본 경로 생성 완료.\n', size(original_path, 1));

%%
figure;
grid on; axis equal; view(45, 25);
plot3(original_path(:,1), original_path(:,2), original_path(:,3), 'k:', 'LineWidth', 1.5)
rotate3d on;

%%

% =========================================================================
% 2. 코너점 자동 탐지
% =========================================================================
fprintf('--- 경로 내 코너점을 자동으로 탐지합니다 ---\n');
corner_indices = [];
for i = 2 : (size(original_path, 1) - 1)
    % 코너를 중심으로 들어오는 벡터와 나가는 벡터 계산
    v1 = original_path(i,:) - original_path(i-1,:);
    v2 = original_path(i+1,:) - original_path(i,:);
    
    % 두 벡터 사이의 각도 계산 (라디안 -> 도)
    angle = acosd(dot(v1, v2) / (norm(v1) * norm(v2)));
    
    % 각도가 임계값보다 크면 코너로 판단
    if angle > corner_angle_threshold
        corner_indices = [corner_indices, i];
        fprintf('인덱스 %d 에서 %.1f도의 코너를 발견했습니다.\n', i, angle);
    end
end

if isempty(corner_indices)
    error('경로에서 코너를 찾지 못했습니다. 임계값을 조절해보세요.');
end


% =========================================================================
% 3. 탐지된 모든 코너에 대해 블렌딩 적용
% =========================================================================
blended_path = original_path;
% 여러 코너가 있을 수 있으므로, 경로가 변경되는 것을 고려하여 역순으로 처리
for i = length(corner_indices):-1:1
    corner_idx = corner_indices(i);
    fprintf('--- 인덱스 %d 코너에 대해 블렌딩을 적용합니다 ---\n', corner_idx);
    blended_path = blend_dense_path(blended_path, corner_idx, blend_distance);
end


% =========================================================================
% 4. 결과 시각화
% =========================================================================
figure; 
hold on; 
grid on; 
axis equal;
view(30, 45);

plot3(original_path(:,1), original_path(:,2), original_path(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 경로');
plot3(blended_path(:,1), blended_path(:,2), blended_path(:,3), 'r-', 'LineWidth', 2.5, 'DisplayName', '최종 블렌딩 경로');

title('촘촘한 경로점에 대한 3차 스플라인 블렌딩');
xlabel('X (mm)'); 
ylabel('Y (mm)'); 
zlabel('Z (mm)');
legend('show', 'Location', 'best');
rotate3d on;
fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');


% =========================================================================
% 촘촘한 경로를 블렌딩하는 메인 함수
% =========================================================================
function final_path = blend_dense_path(path_points, corner_idx, blend_dist)
    % --- [단계 2] 블렌드 시작점 찾기 (코너에서 역방향) ---
    dist_sum = 0;
    start_seg_idx = 0; % 블렌드 시작점이 놓일 세그먼트의 시작 인덱스
    for i = corner_idx:-1:2
        segment_len = norm(path_points(i,:) - path_points(i-1,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            P_spline_start = path_points(i-1,:) + ratio * (path_points(i,:) - path_points(i-1,:));
            start_seg_idx = i - 1;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if start_seg_idx == 0 % 경로 시작까지의 거리가 blend_dist보다 짧은 경우
        error('블렌딩 거리(blend_distance)가 코너 이전 경로의 길이보다 깁니다.');
    end

    % --- [단계 2] 블렌드 끝점 찾기 (코너에서 순방향) ---
    dist_sum = 0;
    end_seg_idx = 0; % 블렌드 끝점이 놓일 세그먼트의 시작 인덱스
    for i = corner_idx:(size(path_points, 1) - 1)
        segment_len = norm(path_points(i+1,:) - path_points(i,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            P_spline_end = path_points(i,:) + ratio * (path_points(i+1,:) - path_points(i,:));
            end_seg_idx = i;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if end_seg_idx == 0 % 경로 끝까지의 거리가 blend_dist보다 짧은 경우
        error('블렌딩 거리(blend_distance)가 코너 이후 경로의 길이보다 깁니다.');
    end
    
    % --- [단계 3] 경계 조건 (접선 벡터) 계산 ---
    T_spline_start_vec = path_points(start_seg_idx+1,:) - path_points(start_seg_idx,:);
    T_spline_start = T_spline_start_vec / norm(T_spline_start_vec);
    
    T_spline_end_vec = path_points(end_seg_idx+1,:) - path_points(end_seg_idx,:);
    T_spline_end = T_spline_end_vec / norm(T_spline_end_vec);

    % --- [단계 4] 3차 에르미트 스플라인 생성 ---
    spline_points = hermite_spline(P_spline_start, P_spline_end, T_spline_start, T_spline_end, 100);

    % --- [단계 5] 최종 경로 결합 ---
    path_before = path_points(1:start_seg_idx, :);
    path_after = path_points(end_seg_idx+1:end, :);
    
    final_path = [path_before; spline_points; path_after];
    
    % 블렌딩 시작/끝점을 시각화하기 위한 코드 (디버깅용)
    hold on;
    plot3(P_spline_start(1), P_spline_start(2), P_spline_start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g', 'DisplayName','블렌드 시작');
    plot3(P_spline_end(1), P_spline_end(2), P_spline_end(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor','b', 'DisplayName','블렌드 끝');
end


% =========================================================================
% 3차 에르미트 스플라인 생성 함수 (3D) - 원본 코드와 동일
% =========================================================================
function points = hermite_spline(P0, P1, T0, T1, num_points)
    t = linspace(0, 1, num_points)';
    h00 = 2*t.^3 - 3*t.^2 + 1;
    h10 = t.^3 - 2*t.^2 + t;
    h01 = -2*t.^3 + 3*t.^2;
    h11 = t.^3 - t.^2;
    
    % 접선 벡터의 크기(magnitude)를 스플라인에 반영하여 곡률을 조절
    scale = norm(P1 - P0); 
    T0_scaled = scale * T0; 
    T1_scaled = scale * T1;
    
    points_x = h00*P0(1) + h10*T0_scaled(1) + h01*P1(1) + h11*T1_scaled(1);
    points_y = h00*P0(2) + h10*T0_scaled(2) + h01*P1(2) + h11*T1_scaled(2);
    points_z = h00*P0(3) + h10*T0_scaled(3) + h01*P1(3) + h11*T1_scaled(3);
    
    points = [points_x, points_y, points_z];
end