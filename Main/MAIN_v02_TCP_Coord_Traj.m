% =========================================================================
% 곡면에 대한 로봇 폴리싱을 위한 경로 계획
%   - 툴 좌표계 기준 로봇 구동을 진행 (각속도에 대한 부분만 변경하면 곡면 추종 가능)
%   - 곡면에 대한 z방향 법선 벡터를 기준으로 각속도 경로 데이터를 추출
%
% == 전처리된 금형 데이터 리샘플링(1kHz) 및 각속도 계산
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
% 1. 데이터 불러오기 및 변수 설정
% =========================================================================
disp('1. 경로 데이터를 불러옵니다...');

% --- 사용자 설정 파라미터 ---
robot_speed_mps = 0.01; % 로봇 TCP 이동 속도 (단위: m/s)
control_freq_hz = 1000;
data_filename = 'transformed_data.csv'; % 입력 데이터 파일 이름

% --- 제어 주기 계산 ---
dt = 1 / control_freq_hz;       % 제어 시간 간격 (1kHz -> 0.001s)

% --- 데이터 로드 ---
% readmatrix는 숫자 행렬을 불러옵니다. data는 6개의 열(X,Y,Z,Nx,Ny,Nz)을 가집니다.
data = readmatrix(data_filename);

% --- 데이터 분리 ---
% 코드를 명확하게 하기 위해 위치와 법선 벡터로 분리합니다.
positions_m = data(:, 1:3) / 1000; % [X, Y, Z] (mm -> m 단위로 변환)
normals = data(:, 4:6);           % [Nx, Ny, Nz]

% 데이터 로드 확인
fprintf('%d개의 경로 지점을 성공적으로 불러왔습니다.\n\n', size(data, 1));

%% ========================================================================
% 2. 경로 분석: 각도 차이 및 각속도 계산
% =========================================================================
disp('2. 경로 분석을 시작합니다 (각도 차이 및 각속도 계산)...');

% --- 2-1. 연속된 법선 벡터 간 각도 차이(Δθ) 계산 ---
v1 = normals(1:end-1, :); % 현재 벡터 세트
v2 = normals(2:end, :);   % 다음 벡터 세트

% 내적(dot product) 계산 (벡터화 방식)
dot_products = sum(v1 .* v2, 2);
% 수치적 안정성을 위해 내적 결과를 -1과 1 사이로 제한
dot_products = max(min(dot_products, 1), -1);

% 역코사인(acosd)을 이용해 각도를 '도(degree)' 단위로 계산
angle_diff_deg = acosd(dot_products);


% --- 2-2. 평균 각속도(ω) 계산 ---
% 연속된 지점 간의 거리(Δs) 계산
p1 = positions_m(1:end-1, :);
p2 = positions_m(2:end, :);
delta_s_m = vecnorm(p2 - p1, 2, 2); % 각 행별로 유클리드 거리(m) 계산

% 각 구간을 이동하는 데 걸리는 시간(Δt) 계산 (t = d/v)
delta_t_s = delta_s_m / robot_speed_mps;

% 평균 각속도 계산 (단위: 도/초, dps)
% 0으로 나누는 것을 방지하기 위해 delta_t가 매우 작은 경우를 처리
delta_t_s(delta_t_s < 1e-9) = 1e-9; % 0에 가까운 값은 작은 값으로 대체
angular_velocity_dps = angle_diff_deg ./ delta_t_s;

fprintf('경로 분석이 완료되었습니다.\n\n');

%% ========================================================================
% 3. 결과 정리 및 파일 저장
% =========================================================================
disp('3. 분석 결과를 파일로 저장합니다...');

% 1. 파일 이름 지정
fileName = 'path_analysis_results.csv';

% --- 핵심 로직 ---
% data_cd 변수가 존재하는지 확인합니다.
if ~exist('data_cd', 'var')
    error('오류: "data_cd" 변수가 작업 공간에 정의되어 있지 않습니다. 경로를 먼저 지정해주세요.');
end

% 2. data_cd에 지정된 폴더가 실제로 존재하는지 확인하고, 없으면 생성
if ~exist(data_cd, 'dir')
   mkdir(data_cd);
   fprintf('지정된 경로에 폴더가 없어 새로 생성했습니다:\n%s\n', data_cd);
end

fullPath = fullfile(data_cd, fileName);

% --- 계산 결과를 원본 데이터와 병합 ---
% 첫 번째 지점은 비교 대상이 없으므로 계산 결과에 NaN을 추가하여 크기를 맞춥니다.
results_matrix = [data, [NaN; angle_diff_deg], [NaN; angular_velocity_dps]];

% --- 테이블 형식으로 변환 (헤더 추가) ---
header = {'X_mm', 'Y_mm', 'Z_mm', 'Nx', 'Ny', 'Nz', 'AngleDiff_deg', 'AngularVelocity_dps'};
results_table = array2table(results_matrix, 'VariableNames', header);

% --- CSV 파일로 저장 ---
writetable(results_table, fullPath);

fprintf('분석 결과가 "%s" 파일로 저장되었습니다.\n\n', fileName);

%% ========================================================================
% 4. 결과 시각화
% =========================================================================
disp('4. 분석 결과를 시각화합니다...');

figure('Name', '금형 경로 분석 결과', 'NumberTitle', 'off', 'WindowState', 'maximized');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); % 2x2 타일 레이아웃

% --- Plot 1: 3D 경로 ---
ax1 = nexttile;
plot3(ax1, positions_m(:,1), positions_m(:,2), positions_m(:,3), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 3);
hold(ax1, 'on');
% 법선 벡터 시각화 (너무 많으면 복잡하므로 5개 지점마다 하나씩 표시)
quiver3(ax1, positions_m(1:5:end,1), positions_m(1:5:end,2), positions_m(1:5:end,3), ...
        normals(1:5:end,1), normals(1:5:end,2), normals(1:5:end,3), 0.02, 'r'); % 벡터 길이 0.02m (20mm)
hold(ax1, 'off');
title(ax1, '3D 경로 및 법선 벡터');
xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)');
axis(ax1, 'equal'); grid(ax1, 'on'); view(ax1, 30, 20);

% --- Plot 2: Y축에 따른 Z 위치 ---
ax2 = nexttile;
plot(ax2, positions_m(:,2), positions_m(:,3), 'k-', 'LineWidth', 1.5);
title(ax2, 'Y축에 따른 Z 위치 (경로 단면)');
xlabel(ax2, 'Y (m)'); ylabel(ax2, 'Z (m)');
grid(ax2, 'on');

% --- Plot 3: 각도 차이 ---
ax3 = nexttile;
plot(ax3, results_table.AngleDiff_deg, 'g-');
title(ax3, 'Normal vector Angular Diff.');
xlabel(ax3, 'Traj index'); ylabel(ax3, 'Angular diff (deg)');
grid(ax3, 'on');

% --- Plot 4: 각속도 ---
ax4 = nexttile;
plot(ax4, results_table.AngularVelocity_dps, 'm-');
title(ax4, 'Mean Angular velocity');
xlabel(ax4, 'Traj index'); ylabel(ax4, 'Angular velocity (deg/s)');
grid(ax4, 'on');