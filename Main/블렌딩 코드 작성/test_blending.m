clear;
close all;
clc;

% =========================================================================
% 1. 시뮬레이션 파라미터 설정
% =========================================================================
speed = 5.0;              % 목표 속도 (mm/s)
dt = 0.001;               % 시간 주기 (1kHz -> 0.001s)
blend_radius = 10.0;      % 블렌딩 반경 (mm), 이 값을 조절하여 곡률을 변경

% =========================================================================
% 2. 원본 경로(Waypoint) 정의
% =========================================================================
P0 = [0, 0, 0];
P1 = [0, 50, 0];
P2 = P1 + [50, 0, -50 * tand(5)];

% =========================================================================
% 3. '직선-호-직선' 경로 생성 (오류 수정 및 검증 버전)
% =========================================================================
fprintf('--- 기하학적 방식 (코너 안쪽)으로 블렌딩 경로를 생성합니다 ---\n');

% --- 기하학적 정보 계산 ---
% 진입/진출 단위 벡터
U_in = (P1 - P0) / norm(P1 - P0);
U_out = (P2 - P1) / norm(P2 - P1);

% 블렌딩 시작점(호의 시작), 끝점(호의 끝)
P_blend_start = P1 - blend_radius * U_in;
P_blend_end = P1 + blend_radius * U_out;

% 호(Arc)의 중심점
arc_center = P_blend_start + blend_radius * U_out;

% 각 세그먼트의 길이
L1 = norm(P_blend_start - P0);
arc_angle = acos(max(min(dot(U_in, -U_out), 1), -1)); % 수치 안정성 확보
L_arc = blend_radius * arc_angle;
L2 = norm(P2 - P_blend_end);
L_total = L1 + L_arc + L2;

fprintf('전체 경로 길이: %.4f mm\n', L_total);

% --- 시간 기반 리샘플링 ---
total_time = L_total / speed;
time_vec = (0:dt:total_time)';
dist_vec = time_vec * speed;

resampled_path = zeros(length(dist_vec), 3);

for i = 1:length(dist_vec)
    d = dist_vec(i);
    
    if d <= L1
        % Case 1: 첫 번째 직선 구간
        p = P0 + (d / L1) * (P_blend_start - P0);
        
    elseif d <= (L1 + L_arc)
        % Case 2: 호(Arc) 구간
        dist_on_arc = d - L1;
        angle_on_arc = dist_on_arc / blend_radius;
        
        % 시작점부터 중심점을 향하는 벡터
        v_start = P_blend_start - arc_center;
        
        % 올바른 회전 축 계산
        axis_of_rotation = cross(U_in, U_out);
        if norm(axis_of_rotation) > 1e-9
             axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
        else
            % 두 벡터가 평행한 경우 (예외 처리)
            % 이 예제에서는 발생하지 않음
            axis_of_rotation = [0 0 1]'; % 임의의 축
        end
        
        % 로드리게스 회전 공식을 이용한 정확한 회전 계산
        v_rotated = v_start * cos(angle_on_arc) + ...
                    cross(axis_of_rotation, v_start) * sin(angle_on_arc) + ...
                    axis_of_rotation * dot(axis_of_rotation, v_start) * (1 - cos(angle_on_arc));
                    
        p = arc_center + v_rotated;
        
    else
        % Case 3: 마지막 직선 구간
        dist_on_line2 = d - L1 - L_arc;
        p = P_blend_end + (dist_on_line2 / L2) * (P2 - P_blend_end);
    end
    resampled_path(i, :) = p;
end

fprintf('완료: %d개의 경로점 생성\n', size(resampled_path, 1));

% =========================================================================
% 4. 결과 시각화
% =========================================================================
figure;
hold on; grid on; axis equal; view(3);

% 원본 경로 (뾰족한 코너)
plot3([P0(1), P1(1), P2(1)], [P0(2), P1(2), P2(2)], [P0(3), P1(3), P2(3)], ...
    'k--', 'LineWidth', 1.5, 'DisplayName', '원본 경로');
plot3(P1(1), P1(2), P1(3), 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '코너점(P1)');

% 블렌딩 시작/끝/중심점 표시 (디버깅용)
plot3(P_blend_start(1), P_blend_start(2), P_blend_start(3), 'g>', 'MarkerSize', 8, 'DisplayName', '블렌드 시작');
plot3(P_blend_end(1), P_blend_end(2), P_blend_end(3), 'b<', 'MarkerSize', 8, 'DisplayName', '블렌드 끝');
plot3(arc_center(1), arc_center(2), arc_center(3), 'm+', 'MarkerSize', 10, 'DisplayName', '호 중심');

% 최종 생성된 블렌딩 경로
plot3(resampled_path(:,1), resampled_path(:,2), resampled_path(:,3), ...
    'r.', 'MarkerSize', 2, 'DisplayName', '블렌딩 적용 경로');
    
title(sprintf('수정된 블렌딩 경로 (반경: %.1fmm)', blend_radius));
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Location', 'best');
rotate3d on;