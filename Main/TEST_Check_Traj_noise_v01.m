%% 경로 데이터 노이즈 분석 스크립트
% 설명: 'test_curve_traj.csv' 파일을 읽어들여 경로의 기하학적 특성을 분석하고,
%       특히 가속도 프로파일을 시각화하여 데이터에 포함된 노이즈를 확인합니다.

clear;
close all;
clc;

Initialization;
Figure_setup;

%% 1. 데이터 로딩
disp('1. 원본 금형 경로 데이터(''test_curve_traj.csv'')를 로딩합니다...');
fprintf('   주의: 분석하려는 원본 금형 경로 파일의 이름을 ''test_curve_traj.csv''로 맞춰주세요.\n\n');

try
    data = readmatrix('Process_Curve_Plate_Traj.csv');
    P = data(:, 1:3); % 위치 데이터 (N x 3)
    fprintf('  -> 성공: %d개의 포인트 데이터를 로드했습니다.\n\n', size(P, 1));
catch ME
    error('오류: test_curve_traj.csv 파일을 찾을 수 없거나 읽을 수 없습니다. 파일 경로와 이름을 확인해주세요.\n오류 메시지: %s', ME.message);
end

%% 2. 기하학적 특성 계산
disp('2. 경로의 기하학적 특성을 계산합니다...');

% 2-1. 점 사이의 거리 (Point-to-Point Distance) 계산
% - 이 값이 일정하면 경로가 등간격으로 샘플링되었음을 의미합니다.
point_distances = vecnorm(diff(P), 2, 2);
fprintf('  - 점 사이의 평균 거리: %.4f mm\n', mean(point_distances));

% 2-2. 가속도 근사치 (Acceleration Approximation) 계산
% - 2차 중앙차분을 이용하여 가속도를 근사합니다. 노이즈에 매우 민감하게 반응합니다.
% - diff(P, 2)는 P(i+2) - 2*P(i+1) + P(i)를 계산합니다.
acceleration_vectors = diff(P, 2);
acceleration_magnitudes = vecnorm(acceleration_vectors, 2, 2);
fprintf('  - 가속도 프로파일 계산 완료.\n\n');


%% 3. 결과 시각화
disp('3. 분석 결과를 시각화합니다...');
figure('Name', '경로 노이즈 분석 결과', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 600]);

% 3-1. 원본 3D 경로 플롯
subplot(1, 3, 1);
plot3(P(:,1), P(:,2), P(:,3), 'b-');
title('원본 3D 경로');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on;

% 3-2. 점 사이의 거리 플롯
subplot(1, 3, 2);
plot(point_distances);
title('점 사이의 거리 변화');
xlabel('포인트 인덱스'); ylabel('거리 (mm)');
grid on;
ylim([min(point_distances)-0.1, max(point_distances)+0.1]); % Y축 범위 조절

% 3-3. 가속도 크기 플롯 (핵심 분석)
subplot(1, 3, 3);
plot(acceleration_magnitudes, 'r-');
title('가속도 프로파일 (노이즈 지표)');
xlabel('포인트 인덱스'); ylabel('가속도 크기 (근사치)');
grid on;

disp('==> 시각화 완료. ''가속도 프로파일'' 그래프를 확인하세요.');
fprintf('    그래프가 뾰족하고 거칠수록 경로에 노이즈가 많다는 의미입니다.\n');