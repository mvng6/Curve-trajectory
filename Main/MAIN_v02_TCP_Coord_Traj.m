% =========================================================================
% 곡면에 대한 로봇 폴리싱을 위한 경로 계획
%   - 툴 좌표계 기준 로봇 구동을 진행 (각속도에 대한 부분만 변경하면 곡면 추종 가능)
%   - 곡면에 대한 z방향 법선 벡터를 기준으로 각속도 경로 데이터를 추출
%
% == 특정 이동 거리만큼 곡면 금형 구동 시 변곡점 부분까지 포함되는지 여부 확인용 코드
% 
%
% 최종 수정: 2025-08-12
% 작성자: LDJ
% =========================================================================

%% Initialization
clear; close all; clc;

opengl hardware;

Initialization;
Figure_setup;

%% ========================================================================
% 0. 경로 데이터 추출용 설정값
% =========================================================================
robot_speed_mmps = 5;       % 원하는 로봇 이동 속도 설정 [mm/s]
control_freq_hz = 1000;     % 제어 주기 설정[Hz]
start_idx_set = 1;
desiredLength_mm = 150;      % 원하는 이동 거리 설정 [mm]

%% ========================================================================
% 1. 데이터 불러오기 및 변수 설정
% =========================================================================
disp('1. 경로 데이터를 불러옵니다...');

data_filename = 'transformed_data.csv'; % 입력 데이터 파일 이름

% --- 제어 주기 계산 ---
dt = 1 / control_freq_hz;       % 제어 시간 간격 (1kHz -> 0.001s)

% --- 데이터 로드 ---
% readmatrix는 숫자 행렬을 불러옵니다. data는 6개의 열(X,Y,Z,Nx,Ny,Nz)을 가집니다.
data = readmatrix(data_filename);

% --- 데이터 분리 ---
% 코드를 명확하게 하기 위해 위치와 법선 벡터로 분리합니다.
pre_positions_mm = data(:, 1:3);         % [X, Y, Z]      (mm)
pre_normals = data(:, 4:6);              % [Nx, Ny, Nz]

% x와 y 교체 (곡면 구동 시 x축을 기준으로 이동하기 위함)
positions_mm = [pre_positions_mm(:,2), pre_positions_mm(:,1), pre_positions_mm(:,3)];
normals = [pre_normals(:,2), pre_normals(:,1), pre_normals(:,3)];

% 데이터 로드 확인
fprintf('==> %d개의 경로 지점을 성공적으로 불러왔습니다.\n\n', size(data, 1));

%% ========================================================================
% 2. 경로 데이터 리샘플링 (1kHz)
% =========================================================================
disp('2. 원본 데이터 보간을 위한 기준 축을 생성합니다...');

% (1) 경로 전체 길이 및 이동 소요 시간 계산
delta_s = vecnorm(diff(positions_mm), 2, 2);
cumulative_dist_mm = [0; cumsum(delta_s)];
original_time_s = cumulative_dist_mm / robot_speed_mmps;

fprintf('==> 곡선의 전체 길이는 %.3fmm 입니다.\n', cumulative_dist_mm(end));
fprintf('==> 곡선 전체를 이동하는데 걸리는 시간은 %.3f초 입니다.\n\n', original_time_s(end));

% (2) 1kHz 간격의 새로운 시간/거리 축 생성
total_duration_s = original_time_s(end);
interp_time_s = (0:dt:total_duration_s);
interp_dist_mm = interp_time_s * robot_speed_mmps;
num_new_points = length(interp_time_s);

fprintf('=== %.0fHz로 리샘플링을 위한 시간/거리축 생성 ===\n',control_freq_hz);
fprintf('==> %d개의 새로운 지점을 생성 완료\n\n', num_new_points);

%% ========================================================================
% 3. 위치 및 법선 벡터 보간
% =========================================================================
disp('3. 보간을 수행합니다...');
disp('Position : Spline 보간 적용');
disp('Orientation : SLERP 보간 적용');

% (1) Position 보간 (Spline)
interp_position_spline = spline(original_time_s, positions_mm', interp_time_s)';

% (2) Orientation 보간 (SLERP)
intper_normals_slerp = zeros(num_new_points,3);

% SLERP 헬퍼 함수 정의 (수학 공식 기반)
slerp = @(v0, v1, t) ...
    (sin((1-t)*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v0 + ...
    (sin(t*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v1;

% 1kHz 시간 스텝마다 루프를 돌며 SLERP 수행
for i = 1:num_new_points
    t_current = interp_time_s(i);

    % 현재 시간이 기존 경로 데이터에서 어떤 시간 구간에 속하는지 탐색
    k = find(original_time_s <= t_current, 1, 'last');
    
    % 마지막 지점을 넘어가면 마지막 법선 벡터를 그대로 사용
    if k >= length(original_time_s)
        intper_normals_slerp(i,:) = normals(end,:);
        continue;
    end

    % 현재 보간 구간 내에서의 진행률(t) 계산 (0~1 사이)
    t_segment = (t_current - original_time_s(k)) / (original_time_s(k+1) - original_time_s(k));
    
    v0 = normals(k,:);
    v1 = normals(k+1,:);
    
    % 두 벡터가 거의 동일한 경우 선형 보간을 수행 (NaN을 방지하기 위함)
    if abs(dot(v0, v1)) > 0.9999
        interp_normals_slerp(i,:) = (1 - t_segment)*v0 + t_segment*v1;
    else
        interp_normals_slerp(i,:) = slerp(v0, v1, t_segment);
    end
end

% 수치 오류 보정을 위한 재정규화
interp_normals_slerp = interp_normals_slerp ./ vecnorm(interp_normals_slerp, 2, 2);

fprintf('==> 경로 데이터의 리샘플링 및 보간 완료\n\n');

%% ========================================================================
% 4. 보간된 경로 데이터의 각속도 계산
% =========================================================================
disp('4. 보간된 경로 데이터의 각속도를 계산합니다...');

v1_slerp = interp_normals_slerp(1:end-1, :);
v2_slerp = interp_normals_slerp(2:end,:);
dot_slerp = max(min(sum(v1_slerp .* v2_slerp, 2), 1), -1);
omega_slerp = acosd(dot_slerp) / dt;
omega_slerp = [omega_slerp; 0];
fprintf('==> 보간된 경로 데이터의 각속도 계산 완료\n\n');

%% ========================================================================
% 5. 원하는 이동 거리만큼의 인덱스 추출
% =========================================================================
disp('5. 원하는 이동 거리에 대한 인덱스를 추출합니다...');

% (1) 입력된 시작 위치와 가장 가까운 점의 인덱스를 보간된 경로 데이터에서 찾기
start_position_mm = interp_position_spline(start_idx_set,:);
distances_from_start = vecnorm(interp_position_spline - start_position_mm, 2, 2);
[~, start_index] = min(distances_from_start);

% (2) 시작 인덱스에 해당하는 누적거리 찾기
start_dist_mm = interp_dist_mm(start_index);

% (3) 최종 목표 누적 거리 계산
target_dist_mm = start_dist_mm + desiredLength_mm;

% (4) 목표 누적 거리에 가장 가까운 점의 인덱스 찾기
[~, final_index] = min(abs(interp_dist_mm - target_dist_mm));

% --- 경로 끝을 초과하는 경우에 대한 경계 처리 ---
if final_index >= length(interp_dist_mm)
    warning('경로의 끝을 초과했습니다. 마지막 지점을 선택합니다.');
    final_index = length(interp_dist_mm);
end

% --- 결과 출력 ---
fprintf('==> 입력된 시작 좌표: [%.3f, %.3f, %.3f]\n', start_position_mm(1), start_position_mm(2), start_position_mm(3));
fprintf('==> 경로상 가장 가까운 시작점: [%.3f, %.3f, %.3f] (인덱스: %d)\n', interp_position_spline(start_index, 1), interp_position_spline(start_index, 2), interp_position_spline(start_index, 3), start_index);
fprintf('==> 원하는 로봇 이동 거리: %.3f mm\n', desiredLength_mm);
fprintf('==> 최종 목표 지점의 인덱스는 %d 입니다.\n', final_index);
fprintf('==> 시작점부터 목표점까지의 이동 소요 시간은 %.3f초 입니다.\n', interp_time_s(final_index) - interp_time_s(start_index));
fprintf('==> 목표 위치의 좌표: [%.3f, %.3f, %.3f]\n', interp_position_spline(final_index, 1), interp_position_spline(final_index, 2), interp_position_spline(final_index, 3));

%% ========================================================================
% Result. 결과 시각화
% =========================================================================
figure;
plot(interp_time_s, omega_slerp, 'LineWidth', 1.5);

% 그래프에 사각형 영역 생성 (시작점과 끝점을 반영)
y_limits = ylim; % 현재 y축 범위 자동 감지
if isempty(y_limits) || y_limits(2) < 5 % 데이터가 없을 경우 기본값 설정
    y_limits = [0 5];
end

y_min = y_limits(1);
y_max = y_limits(2);
x_patch = [interp_time_s(start_index), interp_time_s(final_index), interp_time_s(final_index), interp_time_s(start_index)];
y_patch = [y_min, y_min, y_max, y_max];
patch(x_patch, y_patch, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% 그래프 스타일 및 레이블 설정
xlim([0 interp_time_s(end)]);
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
legend('Angular Velocity', 'Desired Moving Section');

grid on;