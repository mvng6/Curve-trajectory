% =========================================================================
% 로봇 폴리싱 경로를 위한 SpeedL 속도 명령 생성 메인 스크립트
% =========================================================================
% 기능: 
%   - path_type에 따라 수학적으로 정의된 경로(반구) 또는 파일 기반 경로를 생성
%   - 경로의 기하학적 정보(접선, 곡률)를 계산
%   - 두산 로봇의 speedl 함수에 사용할 연속적인 속도 명령을 생성하고 시각화
%
% 필요 함수:
%   - define_path_xaxis.m
%   - define_path_yaxis.m
%   - fun_define_path_curve.m
% =========================================================================

clear; clc; close all;

%% 1. 경로 타입 선택 및 파라미터 설정
% =========================================================================

% 생성할 경로 타입을 선택합니다.
% 'hemi_xaxis': X축 기준 반구 경로
% 'hemi_yaxis': Y축 기준 반구 경로
% 'mold_from_file': 파일에서 금형 데이터 불러오기
path_type = 'mold_from_file'; % <-- 📝 원하는 경로 타입으로 변경하세요.

% --- 공통 파라미터 ---
V = 10.0; % 목표 선속도 (mm/s)

% --- 파일 기반 경로용 파라미터 ---
filename = 'test_curve_traj.csv'; % <-- 📝 파일 경로일 경우, 파일명을 입력하세요.

% --- 수학적 경로용 파라미터 ---
R = 200.0;         % 반구 반지름 (mm)
C = [0, 370, 0];   % 반구 중심 위치 (x, y, z)
dt = 0.01;         % 제어 주기 (초) (수학적 경로 생성 시 사용)

%% 2. 경로 데이터 생성 (선택된 타입에 따라 분기)
% =========================================================================
fprintf('선택된 경로 타입: %s\n', path_type);

if strcmp(path_type, 'hemi_xaxis') || strcmp(path_type, 'hemi_yaxis')
    % --- 수학적 경로 생성 (반구) ---
    total_time = (pi * R) / V;
    t = (0:dt:total_time)';
    
    if strcmp(path_type, 'hemi_xaxis')
        [P_raw, N_raw] = fun_define_path_xaxis(C, R, t, V);
    else % hemi_yaxis
        [P_raw, N_raw] = fun_define_path_yaxis(C, R, t, V);
    end
    
    % 데이터 형상 변환 (3xN -> Ns3) 및 추가 정보 계산
    P = P_raw';
    N_surface = N_raw';
    n_points = size(P, 1);

    % 경로 접선(Tangent) 벡터 계산
    T_path = zeros(n_points, 3);
    for i = 2:n_points-1
        tangent_vec = P(i+1, :) - P(i-1, :);
        T_path(i, :) = tangent_vec / norm(tangent_vec);
    end
    T_path(1, :) = (P(2, :) - P(1, :)) / norm(P(2, :) - P(1, :));
    T_path(n_points, :) = (P(n_points, :) - P(n_points-1, :)) / norm(P(n_points, :) - P(n_points-1, :));

    % 경로 곡률(Curvature) 계산
    kappa = zeros(n_points, 1);
    for i = 2:n_points-1
        v_i = P(i+1, :) - P(i-1, :);
        a_i = P(i+1, :) - 2*P(i, :) + P(i-1, :);
        if norm(v_i) > 1e-6
            kappa(i) = norm(cross(v_i, a_i)) / (norm(v_i)^3);
        else
            kappa(i) = 0;
        end
    end
    
elseif strcmp(path_type, 'mold_from_file')
    % --- 파일 기반 경로 생성 ---
    [P, N_surface, T_path, kappa] = fun_define_path_curve(filename);
    
else
    error('잘못된 경로 타입입니다. (''hemi_xaxis'', ''hemi_yaxis'', ''mold_from_file'')');
end

%% 3. 각 지점별 SpeedL 명령 생성
% =========================================================================
n_points = size(P, 1);
speedl_commands = zeros(n_points, 6);

for i = 1:n_points
    % 현재 지점의 기하학적 정보 추출
    Pi = P(i, :)';
    Ni = N_surface(i, :)';
    Ti = T_path(i, :)';
    ki = kappa(i);
    
    % 툴 좌표계 정의 (X:접선, Z:표면 안쪽, Y:외적)
    X_tool = Ti;
    Z_tool = -Ni; 
    Y_tool = cross(Z_tool, X_tool);
    
    % 각속도 계산
    if i < n_points && ki > 1e-6
        T_next = T_path(i+1, :)';
        rotation_axis = cross(Ti, T_next);
        rotation_axis_norm = norm(rotation_axis);
        
        % 접선 벡터가 거의 변하지 않는 직선 구간 처리
        if rotation_axis_norm > 1e-9
            rotation_axis = rotation_axis / rotation_axis_norm;
            omega_magnitude = V * ki; % 각속도 크기 (rad/s)
            omega_base = omega_magnitude * rotation_axis; % 베이스 기준 각속도 벡터
        else
            omega_base = [0; 0; 0]; % 직선 구간
        end
    else
        omega_base = [0; 0; 0]; % 마지막 점 또는 직선 구간
    end
    
    % 베이스 기준 -> 툴 기준 각속도 변환
    R_tool_to_base = [X_tool, Y_tool, Z_tool];
    omega_tool_rad = R_tool_to_base' * omega_base; % (rad/s)
    
    % 최종 speedl 명령 생성
    vx = V; vy = 0; vz = 0;
    wx = omega_tool_rad(1) * (180/pi);
    wy = omega_tool_rad(2) * (180/pi);
    wz = omega_tool_rad(3) * (180/pi);
    
    speedl_commands(i, :) = [vx, vy, vz, wx, wy, wz];
end

fprintf('계산 완료! 최종 SpeedL 명령 행렬 (처음 5개):\n');
disp(speedl_commands(1:5, :));

%%
for i = 1:length(N_surface)
    N_surface(i,:) = N_surface(i,:) / norm(N_surface(i,:));
end

%% 4. 3D 시각화 (결과 검증)
% =========================================================================
figure('Name', 'SpeedL Path Generation Result', 'NumberTitle', 'off');
hold on; grid on; axis equal; view(3);
title(['Path Type: ', strrep(path_type, '_', ' ')]);
xlabel('X_base (mm)'); ylabel('Y_base (mm)'); zlabel('Z_base (mm)');

% 경로 및 원본 법선 벡터 그리기
plot3(P(:,1), P(:,2), P(:,3), 'b-', 'LineWidth', 2);
quiver3(P(:,1), P(:,2), P(:,3), ...
        N_surface(:,1), N_surface(:,2), N_surface(:,3), 0.01*R, ...
        'Color', [0.5 0.5 0.5], 'LineStyle', ':');

% 계산된 툴 좌표계 표시
step = floor(n_points / 15); % 약 15개의 툴 자세를 표시
for i = 1:step:n_points
    pos = P(i, :)';
    % 툴 좌표계 축 계산 (Z축은 -N 방향)
    xt = T_path(i, :)';
    zt = -N_surface(i, :)';
    yt = cross(zt, xt);
    R_mat = [xt, yt, zt];
    
    % 화살표로 툴 좌표계 표시
    quiver3(pos(1), pos(2), pos(3), R_mat(1,1), R_mat(2,1), R_mat(3,1), 0.15*R, 'r', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), R_mat(1,2), R_mat(2,2), R_mat(3,2), 0.15*R, 'g', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), R_mat(1,3), R_mat(2,3), R_mat(3,3), 0.15*R, 'k', 'LineWidth', 2);
end
legend('Path', 'Surface Normals (Outward)', 'Tool X (Tangent)', 'Tool Y', 'Tool Z (Inward)');
rotate3d on;