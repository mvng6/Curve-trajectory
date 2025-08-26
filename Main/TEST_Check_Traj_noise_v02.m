%% 경로 곡률 분석 스크립트
% 설명: 리샘플링된 경로 데이터를 읽어 각 점에서의 곡률(Curvature)과
%       곡률반경(Radius of Curvature)을 계산하고 시각화합니다.

clear;
close all;
clc;

%% 1. 리샘플링된 경로 데이터 로딩
disp('1. 리샘플링된 경로 파일(Process_Curve_Plate_Traj.csv)을 로딩합니다...');
try
    % MAIN_Curve_Path_Blending.m 에서 저장한 파일을 읽어옵니다.
    data = readmatrix('Process_Curve_Plate_Traj.csv');
    points = data(:, 1:3); % 위치 데이터
    dt = 0.001; % MAIN 스크립트에서 설정한 resampling_time_step 값
    
    fprintf('  -> 성공: %d개의 포인트 데이터를 로드했습니다.\n\n', size(points, 1));
catch ME
    error('오류: Process_Curve_Plate_Traj.csv 파일을 찾을 수 없습니다. MAIN 스크립트를 먼저 실행하여 파일을 생성해주세요.');
end

%% 2. 곡률 계산
disp('2. 경로의 곡률을 수치적으로 계산합니다...');

% 수치 미분을 이용한 속도 및 가속도 벡터 근사
vel_vectors = diff(points) / dt;
accel_vectors = diff(points, 2) / (dt^2);

% 계산을 위해 벡터 길이를 맞춰줍니다.
vel_vectors = [vel_vectors; vel_vectors(end,:)];
accel_vectors = [accel_vectors; accel_vectors(end-1:end,:)];

% 곡률(kappa) 계산 공식: kappa = |v x a| / |v|^3
cross_prod_va = cross(vel_vectors, accel_vectors, 2);
norm_cross_prod = vecnorm(cross_prod_va, 2, 2);
norm_vel = vecnorm(vel_vectors, 2, 2);

% 분모가 0이 되는 것을 방지하기 위해 작은 값(epsilon)을 더함
epsilon = 1e-12;
curvature = norm_cross_prod ./ (norm_vel.^3 + epsilon);

% 곡률반경(Radius of Curvature) 계산
radius_of_curvature = 1 ./ (curvature + epsilon);

% 시각화를 위해 최대 곡률반경 값을 제한 (값이 무한대로 튀는 것을 방지)
max_radius_for_plot = 10000; % 10,000 mm (10미터)
radius_of_curvature(radius_of_curvature > max_radius_for_plot) = max_radius_for_plot;

fprintf('  -> 곡률 및 곡률반경 계산 완료.\n\n');

%% 3. 결과 시각화
disp('3. 분석 결과를 시각화합니다...');
time_axis = (0:dt:(size(points,1)-1)*dt)';

figure('Name', '경로 곡률 분석', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);

% 1. 곡률 그래프
subplot(2, 1, 1);
plot(time_axis, curvature, 'b-');
title('경로의 곡률(Curvature) 변화');
xlabel('시간 (s)');
ylabel('곡률 (1/mm)');
grid on;
xlim([0 time_axis(end)]);

% 2. 곡률반경 그래프
subplot(2, 1, 2);
plot(time_axis, radius_of_curvature, 'r-');
title('경로의 곡률반경(Radius of Curvature) 변화');
xlabel('시간 (s)');
ylabel('곡률반경 (mm)');
grid on;
xlim([0 time_axis(end)]);
ylim([0 max_radius_for_plot]); % Y축 범위 제한

disp('==> 완료. 곡률 그래프와 각속도 그래프의 형태를 비교해보세요.');%% 경로 곡률 분석 스크립트
% 설명: 리샘플링된 경로 데이터를 읽어 각 점에서의 곡률(Curvature)과
%       곡률반경(Radius of Curvature)을 계산하고 시각화합니다.

clear;
close all;
clc;

%% 1. 리샘플링된 경로 데이터 로딩
disp('1. 리샘플링된 경로 파일(Process_Curve_Plate_Traj.csv)을 로딩합니다...');
try
    % MAIN_Curve_Path_Blending.m 에서 저장한 파일을 읽어옵니다.
    data = readmatrix('Process_Curve_Plate_Traj.csv');
    points = data(:, 1:3); % 위치 데이터
    dt = 0.001; % MAIN 스크립트에서 설정한 resampling_time_step 값
    
    fprintf('  -> 성공: %d개의 포인트 데이터를 로드했습니다.\n\n', size(points, 1));
catch ME
    error('오류: Process_Curve_Plate_Traj.csv 파일을 찾을 수 없습니다. MAIN 스크립트를 먼저 실행하여 파일을 생성해주세요.');
end

%% 2. 곡률 계산
disp('2. 경로의 곡률을 수치적으로 계산합니다...');

% 수치 미분을 이용한 속도 및 가속도 벡터 근사
vel_vectors = diff(points) / dt;
accel_vectors = diff(points, 2) / (dt^2);

% 계산을 위해 벡터 길이를 맞춰줍니다.
vel_vectors = [vel_vectors; vel_vectors(end,:)];
accel_vectors = [accel_vectors; accel_vectors(end-1:end,:)];

% 곡률(kappa) 계산 공식: kappa = |v x a| / |v|^3
cross_prod_va = cross(vel_vectors, accel_vectors, 2);
norm_cross_prod = vecnorm(cross_prod_va, 2, 2);
norm_vel = vecnorm(vel_vectors, 2, 2);

% 분모가 0이 되는 것을 방지하기 위해 작은 값(epsilon)을 더함
epsilon = 1e-12;
curvature = norm_cross_prod ./ (norm_vel.^3 + epsilon);

% 곡률반경(Radius of Curvature) 계산
radius_of_curvature = 1 ./ (curvature + epsilon);

% 시각화를 위해 최대 곡률반경 값을 제한 (값이 무한대로 튀는 것을 방지)
max_radius_for_plot = 10000; % 10,000 mm (10미터)
radius_of_curvature(radius_of_curvature > max_radius_for_plot) = max_radius_for_plot;

fprintf('  -> 곡률 및 곡률반경 계산 완료.\n\n');

%% 3. 결과 시각화
disp('3. 분석 결과를 시각화합니다...');
time_axis = (0:dt:(size(points,1)-1)*dt)';

figure('Name', '경로 곡률 분석', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);

% 1. 곡률 그래프
subplot(2, 1, 1);
plot(time_axis, curvature, 'b-');
title('경로의 곡률(Curvature) 변화');
xlabel('시간 (s)');
ylabel('곡률 (1/mm)');
grid on;
xlim([0 time_axis(end)]);

% 2. 곡률반경 그래프
subplot(2, 1, 2);
plot(time_axis, radius_of_curvature, 'r-');
title('경로의 곡률반경(Radius of Curvature) 변화');
xlabel('시간 (s)');
ylabel('곡률반경 (mm)');
grid on;
xlim([0 time_axis(end)]);
ylim([0 max_radius_for_plot]); % Y축 범위 제한

disp('==> 완료. 곡률 그래프와 각속도 그래프의 형태를 비교해보세요.');