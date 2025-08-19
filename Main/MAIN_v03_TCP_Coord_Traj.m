% =========================================================================
% 곡면에 대한 로봇 폴리싱을 위한 경로 계획
%   - 툴 좌표계 기준 로봇 구동을 진행 (각속도에 대한 부분만 변경하면 곡면 추종 가능)
%   - 곡면에 대한 z방향 법선 벡터를 기준으로 각속도 경로 데이터를 추출
%
% == 각속도 계산 및 경로 생성
% 
%
% 최종 수정: 2025-08-13
% 작성자: LDJ
% =========================================================================

%% Initialization
clear; close all; clc;

Initialization;
Figure_setup;

%% 0. 경로 생성을 위한 조건 설정
robot_speed = 5;
control_freq_hz = 1000;
straight_move_len_mm = 30;
curve_move_len_mm = 50;

%% 1. 원본 경로 리샘플링 & 보간
disp('1. 원본 금형 경로를 리샘플링 및 보간합니다...');
traj_file_name = 'transformed_data.csv';
dt = 1/control_freq_hz;

[interp_time_s, interp_position_spline, interp_normals_slerp, interp_dist_mm] = fun_resample_interp(traj_file_name, robot_speed, dt);

fprintf('==> 원본 금형 경로에 대한 리샘플링 및 보간 완료!\n\n');

%% 보간된 경로 데이터의 각속도 계산
disp('2. 보간된 경로 데이터의 각속도를 계산합니다...');

v1_slerp = interp_normals_slerp(1:end-1, :);
v2_slerp = interp_normals_slerp(2:end,:);
dot_slerp = max(min(sum(v1_slerp .* v2_slerp, 2), 1), -1);
omega_slerp = acosd(dot_slerp) / dt;
omega_slerp = [omega_slerp; 0];
fprintf('==> 보간된 경로 데이터의 각속도 계산 완료!\n\n');

%% 경로 시작 위치 설정
%   - 실제 금형 위치에 맞춰 시작 위치 평행 이동
interp_position_spline(:,1) = interp_position_spline(:,1) - interp_position_spline(1,1) - 180.0;
interp_position_spline(:,2) = interp_position_spline(:,2) - interp_position_spline(1,2) + 370.0;
interp_position_spline(:,3) = interp_position_spline(:,3) - interp_position_spline(1,3) + 380.0;

%% 경로 생성
disp('3. 경로 생성을 시작합니다...');

% ===============================
% 직선 구간
% 첫 번째 경로 지점 생성
start_position = [interp_position_spline(1,:), interp_normals_slerp(1,:)];
direction = [0, 1, 0, 0, 0, 0];
[straight_pos_traj, straight_vel_traj] = fun_generate_straight_traj(start_position, ...
    straight_move_len_mm, robot_speed, direction, dt);

traj_pos_profile = straight_pos_traj;
traj_vel_profile = straight_vel_traj;

point = [traj_pos_profile(1,:);traj_pos_profile(end,:)];
fprintf('==> 첫 번째 경로 생성 완료! (직선구간 +y)\n');

% ===============================
% 곡선 구간
% 두 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, 0, 0, 0, -1, 0];
offset = straight_move_len_mm;
home_flag = 0;
[curve_pos_traj, curve_vel_traj] = fun_generate_curve_traj(start_position, ...
    curve_move_len_mm, robot_speed, direction, ...
    interp_position_spline, interp_normals_slerp, omega_slerp, ...
    interp_dist_mm, offset, home_flag);

traj_pos_profile = [traj_pos_profile; curve_pos_traj];
traj_vel_profile = [traj_vel_profile; curve_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 두 번째 경로 생성 완료! (곡선구간 +x)\n');

% ===============================
% 직선 구간
% 세 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, -1, 0, 0, 0, 0];
[straight_pos_traj, straight_vel_traj] = fun_generate_straight_traj(start_position, ...
    straight_move_len_mm, robot_speed, direction, dt);

traj_pos_profile = [traj_pos_profile; straight_pos_traj];
traj_vel_profile = [traj_vel_profile; straight_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 세 번째 경로 생성 완료! (직선구간 -y)\n');

% ===============================
% 곡선 구간
% 네 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, 0, 0, 0, -1, 0];
offset = 0;
home_flag = 0;
[curve_pos_traj, curve_vel_traj] = fun_generate_curve_traj(start_position, ...
    curve_move_len_mm, robot_speed, direction, ...
    interp_position_spline, interp_normals_slerp, omega_slerp, ...
    interp_dist_mm, offset, home_flag);

traj_pos_profile = [traj_pos_profile; curve_pos_traj];
traj_vel_profile = [traj_vel_profile; curve_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 네 번째 경로 생성 완료! (곡선구간 +x)\n');

% ===============================
% 직선 구간
% 다섯 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, 1, 0, 0, 0, 0];
[straight_pos_traj, straight_vel_traj] = fun_generate_straight_traj(start_position, ...
    straight_move_len_mm, robot_speed, direction, dt);

traj_pos_profile = [traj_pos_profile; straight_pos_traj];
traj_vel_profile = [traj_vel_profile; straight_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 다섯 번째 경로 생성 완료! (직선구간 +y)\n');

% ===============================
% 곡선 구간
% 여섯 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, 0, 0, 0, -1, 0];
offset = 30;
home_flag = 0;
[curve_pos_traj, curve_vel_traj] = fun_generate_curve_traj(start_position, ...
    curve_move_len_mm, robot_speed, direction, ...
    interp_position_spline, interp_normals_slerp, omega_slerp, ...
    interp_dist_mm, offset, home_flag);

traj_pos_profile = [traj_pos_profile; curve_pos_traj];
traj_vel_profile = [traj_vel_profile; curve_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 여섯 번째 경로 생성 완료! (곡선구간 +x)\n');

% ===============================
% 직선 구간
% 일곱 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, -1, 0, 0, 0, 0];
[straight_pos_traj, straight_vel_traj] = fun_generate_straight_traj(start_position, ...
    straight_move_len_mm, robot_speed, direction, dt);

traj_pos_profile = [traj_pos_profile; straight_pos_traj];
traj_vel_profile = [traj_vel_profile; straight_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 일곱 번째 경로 생성 완료! (직선구간 -y)\n');

% ===============================
% 곡선 구간 (초기 위치로 이동)
% 여덟 번째 경로 지점 생성
start_position = traj_pos_profile(end,:);
direction = [0, 0, 0, 0, 1, 0];
offset = 0;
home_flag = 1;
[curve_pos_traj, curve_vel_traj] = fun_generate_curve_traj(start_position, ...
    curve_move_len_mm, robot_speed, direction, ...
    interp_position_spline, interp_normals_slerp, omega_slerp, ...
    interp_dist_mm, offset, home_flag);

traj_pos_profile = [traj_pos_profile; curve_pos_traj];
traj_vel_profile = [traj_vel_profile; curve_vel_traj];

point = [point;traj_pos_profile(end,:)];
fprintf('==> 여섯 번째 경로 생성 완료! (곡선구간 -x)\n\n');

%% 시각화
disp('4. 생성된 경로에 대한 시각화를 시작합니다...');

figure;
plot3(traj_pos_profile(:,1),traj_pos_profile(:,2),traj_pos_profile(:,3));
hold on;
scatter3(point(:,1),point(:,2),point(:,3),'ro','MarkerFaceColor','r');
scatter3(point(end,1),point(end,2),point(end,3),'bo','MarkerFaceColor','b')
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
fprintf('==> 위치 경로 그래프 시각화 완료!\n');

figure('Name', 'Linear and Angular Velocity Profile');
% 선속도에 대한 그래프 좌측에 생성
yyaxis left;
plot(traj_vel_profile(:,2), 'b-', 'LineWidth', 1.5); 
xlim([0, length(traj_vel_profile)]);
ylim([-10, 10]);
ylabel('Linear Velocity (mm/s)'); 
ax = gca;
ax.YColor = 'b';

% 각속도에 대한 그래프 우측에 생성
yyaxis right;
plot(traj_vel_profile(:,5), 'r--', 'LineWidth', 1.5);
xlim([0, length(traj_vel_profile)]);
ylim([-10, 10]);
ylabel('Angular Velocity (deg/s)');
ax.YColor = 'r';

% 그래프 요소 설정
title('Robot TCP Velocity Profile');
xlabel('Data Points (Time Steps)');
% legend('Linear Velocity (v_y)', 'Angular Velocity (w_y)', 'Location', 'best');
legend('Linear Velocity ($v_y$)', 'Angular Velocity ($w_y$)', 'Interpreter', 'latex', 'Location', 'best');
hold off;

fprintf('==> 속도 경로 그래프 시각화 완료!\n\n');

%% 속도 경로 데이터 저장
writematrix(traj_vel_profile, fullfile(data_cd, 'Robot_TCP_Velocity_profile.txt'), 'Delimiter', '\t');
