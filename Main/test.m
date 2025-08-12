%% ========================================================================
% 경로 보간 비교: 스플라인(Spline) vs. SLERP
%   - 1kHz 궤적 생성 시, 두 가지 보간 방법의 결과를 비교 분석합니다.
% =========================================================================

%% ========================================================================
% 섹션 0: 초기화 및 파라미터 설정
% =========================================================================
clear; close all; clc;

% --- 사용자 설정 파라미터 ---
robot_speed_mps = 0.01; % 로봇 TCP 평균 이동 속도 (단위: m/s)
control_frequency_hz = 1000; % 로봇 제어 주파수 (Hz)
data_filename = 'transformed_data.csv'; % 원본 데이터 파일

dt = 1 / control_frequency_hz;


%% ========================================================================
% 섹션 1: 원본 데이터 로드 및 기준 축 생성
% =========================================================================
disp('1. 원본 데이터를 로드하고 보간을 위한 기준 축을 생성합니다...');
data = readmatrix(data_filename);
positions_m = data(:, 1:3) / 1000;
normals = data(:, 4:6);

% --- 원본 경로의 누적 거리 및 시간 축 생성 ---
delta_s_m = vecnorm(diff(positions_m), 2, 2);
cumulative_dist_m = [0; cumsum(delta_s_m)];
original_time_s = cumulative_dist_m / robot_speed_mps; % 각 원본 지점의 도달 시간

% --- 1kHz 간격의 새로운 시간/거리 축 생성 ---
total_duration_s = original_time_s(end);
interp_time_s = (0:dt:total_duration_s)';
interp_dist_m = interp_time_s * robot_speed_mps;
num_new_points = length(interp_time_s);
fprintf('%d개의 새로운 지점을 생성합니다.\n', num_new_points);


%% ========================================================================
% 섹션 2: 두 가지 방법으로 위치 및 법선 벡터 보간
% =========================================================================
disp('2. Spline과 SLERP 두 가지 방식으로 보간을 수행합니다...');

% --- 방법 1: 스플라인(Spline) 보간 ---
interp_positions_spline = spline(original_time_s, positions_m', interp_time_s)';
interp_normals_spline_raw = spline(original_time_s, normals', interp_time_s)';
interp_normals_spline = interp_normals_spline_raw ./ vecnorm(interp_normals_spline_raw, 2, 2);

% --- 방법 2: SLERP(구면 선형 보간) ---
% SLERP는 위치가 아닌 방향 보간에만 사용됩니다. 위치는 스플라인 결과를 그대로 사용합니다.
interp_positions_slerp = interp_positions_spline; 
interp_normals_slerp = zeros(num_new_points, 3); % 결과를 저장할 배열 초기화

% SLERP 헬퍼 함수 정의 (수학 공식 기반)
slerp = @(v0, v1, t) ...
    (sin((1-t)*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v0 + ...
    (sin(t*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v1;

% 1kHz 시간 스텝마다 루프를 돌며 SLERP 수행
for i = 1:num_new_points
    t_current = interp_time_s(i);
    
    % 현재 시간이 어떤 원본 시간 구간에 속하는지 찾기
    k = find(original_time_s <= t_current, 1, 'last');
    
    % 마지막 지점을 넘어가면 마지막 법선 벡터를 그대로 사용
    if k >= length(original_time_s)
        interp_normals_slerp(i, :) = normals(end, :);
        continue;
    end
    
    % 해당 구간 내에서의 진행률(t) 계산 (0~1 사이)
    t_segment = (t_current - original_time_s(k)) / (original_time_s(k+1) - original_time_s(k));
    
    v0 = normals(k, :);
    v1 = normals(k+1, :);
    
    % 두 벡터가 거의 동일하면 선형 보간 수행 (NaN 방지)
    if abs(dot(v0, v1)) > 0.9999
        interp_normals_slerp(i, :) = (1-t_segment)*v0 + t_segment*v1;
    else
        interp_normals_slerp(i, :) = slerp(v0, v1, t_segment);
    end
end
% 재정규화 (수치 오류 보정)
interp_normals_slerp = interp_normals_slerp ./ vecnorm(interp_normals_slerp, 2, 2);


%% ========================================================================
% 섹션 3: 각 보간 방식의 각속도 계산 및 비교
% =========================================================================
disp('3. 각 보간 방식의 각속도를 계산하고 비교합니다...');

% --- 스플라인 기반 각속도 ---
v1_spline = interp_normals_spline(1:end-1, :);
v2_spline = interp_normals_spline(2:end, :);
dot_spline = max(min(sum(v1_spline .* v2_spline, 2), 1), -1);
omega_spline = acosd(dot_spline) / dt;

% --- SLERP 기반 각속도 ---
v1_slerp = interp_normals_slerp(1:end-1, :);
v2_slerp = interp_normals_slerp(2:end, :);
dot_slerp = max(min(sum(v1_slerp .* v2_slerp, 2), 1), -1);
omega_slerp = acosd(dot_slerp) / dt;

% --- 시각화 ---
figure('Name', 'Spline vs. SLERP 각속도 비교', 'NumberTitle', 'off', 'WindowState', 'maximized');
plot(interp_time_s(1:end-1), omega_spline, 'r-', 'DisplayName', 'Spline 보간 각속도');
hold on;
plot(interp_time_s(1:end-1), omega_slerp, 'b-', 'LineWidth', 1.5, 'DisplayName', 'SLERP 보간 각속도');
hold off;
title('보간 방식에 따른 각속도 프로파일 비교');
xlabel('시간 (s)'); ylabel('각속도 (도/초)');
grid on; legend;
xlim([0, total_duration_s]); % 전체 시간 범위 표시