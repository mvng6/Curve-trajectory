clear;
close all;
clc;

%%
% =========================================================================
% 1. 시뮬레이션 파라미터 및 경로 생성 (사용자님 코드 원본 유지)
% =========================================================================
blend_distance = 10.0; % 블렌딩을 위해 코너로부터 잘라낼 거리 (mm)

fprintf('--- 사용자님께서 제공한 코드로 경로를 생성합니다 ---\n');

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

path_3 = ones(3,path_len);
path_3(1,:) = path_3(1,:) .* path_2(1,end);
path_3(2,:) = linspace(path_2(2,end),P_line_start(2),path_len);
path_3(3,:) = path_3(3,:) .* path_2(3,end);

arc_start_angle = pi;
arc_end_angle = 3*pi/2;

path_4 = [arc_center(1) + arc_radius*cos(theta_vis); 
    arc_center(2)*ones(1,100);
    arc_center(3)+arc_radius*sin(theta_vis)];


path_4(1,:) = path_4(1,:) + path_3(1,end);
path_4(2,:) = path_4(2,:) .* path_3(2,end);
path_4(3,:) = path_4(3,:) + path_3(3,end);

original_path = [path_1, path_2, path_3, path_4]';

fprintf('총 %d개의 경로점으로 구성된 원본 경로 생성 완료.\n', size(original_path, 1));


%%
% =========================================================================
% 2. 코너점 자동 탐지 (★★★ 개선된 'Look-ahead' 로직 적용 ★★★)
% =========================================================================
fprintf('--- 경로 내 코너점을 자동으로 탐지합니다 ---\n');

% 코너 탐지를 위해 얼마나 멀리 떨어져 있는 점을 볼 것인가 (점의 개수)
lookahead_points = 10; 

corner_angle_threshold = 30; % 코너로 판단할 최소 각도 (degrees), 90도보다 약간 낮게 설정
corner_indices = []; 

% 각 경로 세그먼트의 경계 인덱스
junction_1 = size(path_1, 2);
junction_2 = junction_1 + size(path_2, 2);
junction_3 = junction_2 + size(path_3, 2);
junctions = [junction_1, junction_2, junction_3];

% lookahead_points 만큼의 여유를 두고 경로 순회
for i = (1 + lookahead_points) : (size(original_path, 1) - lookahead_points)
    % 현재 점(i)을 기준으로 lookahead_points 만큼 떨어진 이전/이후 점을 선택
    prev_point = original_path(i - lookahead_points, :);
    current_point = original_path(i, :);
    next_point = original_path(i + lookahead_points, :);
    
    v1 = current_point - prev_point;
    v2 = next_point - current_point;
    
    if norm(v1) > 1e-6 && norm(v2) > 1e-6
        angle = acosd(dot(v1, v2) / (norm(v1) * norm(v2)));
        
        if angle > corner_angle_threshold
            % 동일한 코너가 중복 탐지되는 것을 방지
            if isempty(corner_indices) || (i - corner_indices(end)) > (lookahead_points * 2)
                 % 실제 코너점은 i가 아닌, 가장 가까운 junction 인덱스로 지정
                [~, min_idx] = min(abs(junctions - i));
                corner_idx_to_add = junctions(min_idx);

                % 이미 추가된 코너가 아니면 추가
                if ~ismember(corner_idx_to_add, corner_indices)
                    corner_indices = [corner_indices, corner_idx_to_add];
                    fprintf('인덱스 %d 근방에서 코너 발견 (실제 코너 인덱스: %d, 각도: %.1f도)\n', i, corner_idx_to_add, angle);
                end
            end
        end
    end
end

% 코너 인덱스를 오름차순으로 정렬
corner_indices = sort(corner_indices, 'asc');

if isempty(corner_indices)
    error('경로에서 코너를 찾지 못했습니다. corner_angle_threshold나 lookahead_points 값을 조절해보세요.');
end


%%
% =========================================================================
% 3. 탐지된 모든 코너에 대해 블렌딩 적용 (기존 코드와 동일)
% =========================================================================
blended_path = original_path;
for i = length(corner_indices):-1:1
    corner_idx = corner_indices(i);
    fprintf('--- 인덱스 %d 코너에 대해 블렌딩을 적용합니다 ---\n', corner_idx);
    blended_path = blend_dense_path(blended_path, corner_idx, blend_distance);
end


%%
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

title('사용자 경로 기반 3차 스플라인 블렌딩');
xlabel('X (mm)'); 
ylabel('Y (mm)'); 
zlabel('Z (mm)');
legend('show', 'Location', 'best');
rotate3d on;
fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');


%%
% =========================================================================
% 함수 영역 (기존 코드와 동일)
% =========================================================================
function final_path = blend_dense_path(path_points, corner_idx, blend_dist)
    % --- [단계 2] 블렌드 시작점 찾기 (코너에서 역방향) ---
    dist_sum = 0;
    start_seg_idx = 0;
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
    if start_seg_idx == 0
        error('블렌딩 거리(blend_distance)가 코너 이전 경로의 길이보다 깁니다.');
    end

    % --- [단계 2] 블렌드 끝점 찾기 (코너에서 순방향) ---
    dist_sum = 0;
    end_seg_idx = 0;
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
    if end_seg_idx == 0
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
end

function points = hermite_spline(P0, P1, T0, T1, num_points)
    t = linspace(0, 1, num_points)';
    h00 = 2*t.^3 - 3*t.^2 + 1;
    h10 = t.^3 - 2*t.^2 + t;
    h01 = -2*t.^3 + 3*t.^2;
    h11 = t.^3 - t.^2;
    
    scale = norm(P1 - P0); 
    T0_scaled = scale * T0; 
    T1_scaled = scale * T1;
    
    points_x = h00*P0(1) + h10*T0_scaled(1) + h01*P1(1) + h11*T1_scaled(1);
    points_y = h00*P0(2) + h10*T0_scaled(2) + h01*P1(2) + h11*T1_scaled(2);
    points_z = h00*P0(3) + h10*T0_scaled(3) + h01*P1(3) + h11*T1_scaled(3);
    
    points = [points_x, points_y, points_z];
end