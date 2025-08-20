clear; clc; close all;

Initialization;

%% 1. 기본 파라미터 설정
R = 200.0;                       % 반구 반지름 (mm)
C = [0, 370, 0];                 % 반구 중심 위치 (x, y, z)
V = 5.0;                        % 목표 선속도 (mm/s)
dt = 0.001;                       % 제어 주기 (초)

% 경로 선택 ('x_axis' 또는 'y_axis')
path_type = 'x_axis'; 

%% 2. 경로 정보 생성 (선택에 따라 함수 호출)
total_time = (pi * R) / V;
t = 0:dt:total_time;

if strcmp(path_type, 'x_axis')
    [P, T, N] = fun_define_path_xaxis(C, R, t, V);
    disp('X축 기준 경로를 생성합니다.');
elseif strcmp(path_type, 'y_axis')
    [P, T, N] = fun_define_path_yaxis(C, R, t, V);
    disp('Y축 기준 경로를 생성합니다.');
else
    error('잘못된 경로 타입입니다.');
end

%% 3. SpeedL 명령 계산 루프
num_steps = length(t);
speedl_commands = zeros(num_steps, 6);

for i = 1:num_steps
    % --- 3.1: 툴 좌표계 정의 ---
    X_tool = T(:, i); % 접선 벡터
    Z_tool = N(:, i); % 법선 벡터
    Y_tool = cross(Z_tool, X_tool); % 외적

    % --- 3.2: 베이스 기준 각속도 계산 ---
    % 경로의 곡률 반경은 R로 일정, 속도는 V
    omega_magnitude = V / R; % (rad/s)
    
    % 경로에 따른 회전축 결정
    if strcmp(path_type, 'x_axis')
        omega_base = [0; omega_magnitude; 0]; % Y축 중심 회전
    else % y_axis
        omega_base = [omega_magnitude; 0; 0]; % X축 중심 회전
    end

    % --- 3.3: 베이스 기준 -> 툴 기준 각속도 변환 ---
    R_tool_to_base = [X_tool, Y_tool, Z_tool];
    omega_tool_rad = R_tool_to_base' * omega_base; % (rad/s)
    
    % --- 3.4: 최종 speedl 명령 생성 ---
    vx = V;
    vy = 0;
    vz = 0;
    
    wx_deg = omega_tool_rad(1) * (180/pi);
    wy_deg = omega_tool_rad(2) * (180/pi);
    wz_deg = omega_tool_rad(3) * (180/pi);
    
    speedl_commands(i, :) = [vx, vy, vz, wx_deg, wy_deg, wz_deg];
end

fprintf('계산된 SpeedL 명령 (첫 5개 스텝):\n');
disp(speedl_commands(1:5, :));

%% 4. 3D 시각화
figure;
hold on; grid on; axis equal; view(3);
title([strrep(path_type, '_', ' '), ' Path and Tool Orientation']);
xlabel('X_base'); ylabel('Y_base'); zlabel('Z_base');

% 반구 표면 그리기
[sx, sy, sz] = sphere;
surf(R*sx + C(1), R*sy + C(2), R*sz + C(3), 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 경로 그리기
plot3(P(1,:), P(2,:), P(3,:), 'b-', 'LineWidth', 2);

% 20 스텝마다 툴 방향 표시
for i = 1:2000:num_steps
    pos = P(:, i);
    X_ax = T(:, i);
    Z_ax = N(:, i);
    Y_ax = cross(Z_ax, X_ax);
    
    quiver3(pos(1), pos(2), pos(3), X_ax(1), X_ax(2), X_ax(3), 0.2*R, 'r', 'LineWidth', 1.5); % X_tool
    quiver3(pos(1), pos(2), pos(3), Y_ax(1), Y_ax(2), Y_ax(3), 0.2*R, 'g', 'LineWidth', 1.5); % Y_tool
    quiver3(pos(1), pos(2), pos(3), Z_ax(1), Z_ax(2), Z_ax(3), 0.2*R, 'k', 'LineWidth', 1.5); % Z_tool
end
legend('Hemisphere', 'Path', 'Tool X', 'Tool Y', 'Tool Z');