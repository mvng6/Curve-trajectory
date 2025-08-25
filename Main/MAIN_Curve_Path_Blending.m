clear;
close all;
clc;

Initialization;

%% 1. 구동할 경로를 생성
disp('CSV 파일에서 경로 데이터를 로딩합니다...');
try
    data = readmatrix('test_curve_traj.csv');
    P = data(:,1:3); % 위치 데이터 (N x 3)
    N = data(:,4:6); % 법선 벡터 데이터 (N x 3)
    
    % 법선 벡터가 단위 벡터가 아닐 수 있으므로 정규화를 수행합니다. (안전 장치)
    N = N ./ vecnorm(N, 2, 2);
    
    fprintf('성공: %d개의 포인트 데이터를 로드했습니다.\n\n', size(P, 1));
catch ME
    error('오류: test_curve_traj.csv 파일을 찾을 수 없거나 읽을 수 없습니다. 파일 경로를 확인해주세요.\n오류 메시지: %s', ME.message);
end

blend_flag = false;

%% 2. 블렌딩 적용
% true & false를 이용해 블렌딩을 적용할지 말지를 선택

if (blend_flag == true)
    config.blending.distance = 5.0; % 블렌딩 거리 (mm)

    config.detection.lookaheadPoints = 30; % 코너 탐지 시 살펴볼 점의 거리
    config.detection.angleThreshold = 80;  % 코너로 인식할 최소 각도 (degrees)

    % 3-1. 경로에서 코너점 찾기
    corner_indices = fun_find_path_corners(P, config.detection);

    % 3-2. 찾은 코너점에 블렌딩 적용하기
    [new_P, new_N] = fun_apply_path_blending_normals(P, N, corner_indices, config.blending.distance);
else
    new_P = P;
    new_N = N;
end

%% 3. 경로 리샘플링
% 최종 경로에 대하여 주기(Hz)와 TCP 구동 속도(Vx)에 대하여 경로를 리샘플링
resampling_time_step = 0.001;
speed = 5;  % [mm/s]

fprintf('--- 블렌딩된 경로를 등속 리샘플링합니다 ---\n');
[points, normals, time] = fun_resamplePathByTime(new_P, new_N, speed, resampling_time_step);
fprintf('  -> 총 %d개의 등속 경로점으로 재구성했습니다.\n\n', size(points, 1));

fprintf('--- [경로 리샘플링 결과 출력] ---\n');
fprintf('고정된 시간 주기: %.3f 초 (%.1f Hz)\n', resampling_time_step, 1/resampling_time_step);
fprintf('------------------------------------------\n');
fprintf('[Case: 속도 = %.1f mm/s]\n', speed);
fprintf('  - 총 이동 시간: %.4f 초\n', time);
fprintf('  - 생성된 포인트 개수: %d 개\n', size(points, 1));
fprintf('  - 포인트 간 평균 거리: %.4f mm\n', mean(vecnorm(diff(points), 2, 2)));

disp('결과를 시각화합니다...');
figure('Name', 'Resampling of Trajectory Result', 'NumberTitle', 'off');
title('Resampling of Trajectory Result');
hold on; grid on; axis equal;
plot3(new_P(:,1), new_P(:,2), new_P(:,3), 'k--','DisplayName','Original Traj');

n_points = length(points);
step = floor(n_points / 5000);
plot3(points(:,1), points(:,2), points(:,3), 'ro-', 'LineWidth', 1.5, 'DisplayName', 'Resampled Traj');
quiver3(points(:,1), points(:,2), points(:,3), ...
    normals(:,1)*10, normals(:,2)*10, normals(:,3)*10, ...
    'k','LineWidth',1,'DisplayName','Normal Vector');
title(sprintf('Speed = %d mm/s (Point %d)', speed, size(points,1)));
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Original Traj','Resampled Traj',' Normals');
view(3);
disp('시각화 완료.');

%% 4. 경로의 위치, 법선 및 접선 벡터 및 곡률반경 계산
% 리샘플링된 경로의 P, N, T, kappa를 계산 (각속도 계산용)

%% 5. TCP 좌표계 기준 속도 제어를 위한 각속도를 계산
% vx값은 항상 고정
fprintf('--- 최종 경로의 선속도와 각속도를 계산합니다 ---\n');

num_pts = size(points, 1);
dt = resampling_time_step;

% 최종 결과를 저장할 구조체 초기화
final_trajectory.time = (0:dt:(num_pts-1)*dt)';
final_trajectory.position = points;
final_trajectory.normal = normals;
final_trajectory.vel_tool = zeros(num_pts, 3);
final_trajectory.omega_tool = zeros(num_pts, 3);

% --- 베이스 좌표계 기준 선속도 계산 ---
v_base = diff(points) / dt;
v_base = [v_base(1,:); v_base]; % 첫 번째 점의 속도를 채워넣어 크기를 맞춤

% --- 각 점을 순회하며 툴 좌표계 기준 속도/각속도 계산 ---
for i = 1:(num_pts - 1)
    % 현재 시점(i)의 툴 좌표계(Rotation Matrix) 구성
    z_axis_i = -final_trajectory.normal(i,:);
    tangent_i = v_base(i,:);
    if norm(tangent_i) < 1e-6, tangent_i = [1 0 0]; end % 속도가 0일 경우 대비
    x_axis_i = tangent_i / norm(tangent_i);
    y_axis_i = cross(z_axis_i, x_axis_i);
    R_i = [x_axis_i', y_axis_i', z_axis_i'];
    
    % 다음 시점(i+1)의 툴 좌표계 구성
    z_axis_ip1 = -final_trajectory.normal(i+1,:);
    tangent_ip1 = v_base(i+1,:);
    if norm(tangent_ip1) < 1e-6, tangent_ip1 = [1 0 0]; end
    x_axis_ip1 = tangent_ip1 / norm(tangent_ip1);
    y_axis_ip1 = cross(z_axis_ip1, x_axis_ip1);
    R_ip1 = [x_axis_ip1', y_axis_ip1', z_axis_ip1'];

    % 선속도를 현재 툴 좌표계 기준으로 변환
    final_trajectory.vel_tool(i,:) = (R_i' * v_base(i,:)')';

    % 각속도 계산
    delta_R = R_i' * R_ip1; % 두 방향 사이의 상대 회전
    [angle, axis] = fun_rotationMatrixToAngleAxis(delta_R); % 축-각도로 변환
    final_trajectory.omega_tool(i,:) = (axis * angle) / dt;
end

% 마지막 점은 이전 값으로 채움
final_trajectory.vel_tool(end,:) = final_trajectory.vel_tool(end-1,:);
final_trajectory.omega_tool(end,:) = final_trajectory.omega_tool(end-1,:);

fprintf('  -> 계산 완료.\n\n');

%%
% =========================================================================
% ★★★ 3단계: 시각화 코드 업데이트 ★★★
% =========================================================================
fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');
figure('Name', '최종 경로 및 속도 프로파일', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 1. 3D 경로 플롯
subplot(2,2,[1,3]); % 왼쪽 공간을 합쳐서 사용
hold on; grid on; axis equal; view(30, 45);
plot3(P(:,1), P(:,2), P(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 경로');
plot3(final_trajectory.position(:,1), final_trajectory.position(:,2), final_trajectory.position(:,3), 'r-', 'LineWidth', 2, 'DisplayName', '최종 경로');
quiver3(final_trajectory.position(1:20:end,1), final_trajectory.position(1:20:end,2), final_trajectory.position(1:20:end,3), ...
        -final_trajectory.normal(1:20:end,1), -final_trajectory.normal(1:20:end,2), -final_trajectory.normal(1:20:end,3), ...
        5, 'b', 'DisplayName', '툴 방향');
title('최종 3D 경로');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend;

% 2. 툴 기준 선속도 플롯
subplot(2,2,2);
plot(final_trajectory.time, final_trajectory.vel_tool);
title('툴 좌표계 기준 선속도');
xlabel('시간 (s)'); ylabel('속도 (mm/s)');
legend('v_x', 'v_y', 'v_z', 'Location', 'best');
grid on;

% 3. 툴 기준 각속도 플롯
subplot(2,2,4);
plot(final_trajectory.time, rad2deg(final_trajectory.omega_tool)); % 각도를 deg/s로 변환
title('툴 좌표계 기준 각속도');
xlabel('시간 (s)'); ylabel('각속도 (deg/s)');
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');
grid on;

%%
traj_pos_profile = [final_trajectory.position, final_trajectory.normal];
traj_vel_profile = [final_trajectory.vel_tool, final_trajectory.omega_tool];

writematrix(traj_pos_profile, fullfile(data_cd, 'Robot_TCP_Pos_profile_test.txt'), 'Delimiter', '\t');
writematrix(traj_vel_profile, fullfile(data_cd, 'Robot_TCP_Velocity_profile_test.txt'), 'Delimiter', '\t');

disp('5. 생성된 속도 경로를 바이너리 파일로 저장합니다...');
try
    vel_binary_file = fullfile(data_cd, 'Robot_TCP_Velocity_profile_test.bin');
    fileID_vel = fopen(vel_binary_file, 'w');
    if fileID_vel == -1, error('속도 프로파일 파일을 열 수 없습니다.'); end

    fwrite(fileID_vel, traj_vel_profile', 'single'); % 데이터 전치하여 저장
    fclose(fileID_vel);

    fprintf('==> 속도 경로 바이너리 파일 저장 완료!\n   - 파일 위치: %s\n', vel_binary_file);
catch ME
    fprintf('==> 속도 바이너리 파일 저장 중 오류 발생:\n');
    disp(ME.message);
    if exist('fileID_vel', 'var') && fileID_vel ~= -1, fclose(fileID_vel); end
end