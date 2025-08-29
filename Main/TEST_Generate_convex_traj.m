%% 테스트 경로 생성 스크립트
% 설명: 위로 볼록한 완만한 곡률을 가진 3D 경로와 법선 벡터를 생성하여
%       'test_curve_traj.csv' 파일로 저장합니다.
% 생성 경로: X축을 따라 진행하며 Z축 방향으로 부드러운 언덕 모양을 가집니다.

clear;
close all;
clc;

Initialization;
Figure_setup;

%% 1. 경로 파라미터 설정
disp('1. 경로 생성을 위한 파라미터를 설정합니다...');

path_length = 200.0;    % 경로의 총 길이 (X축 방향, mm)
amplitude = 30.0;       % 언덕의 높이 (Z축 방향, mm). 이 값이 클수록 곡률이 커집니다.
num_points = 100;      % 경로를 구성할 원본 점의 개수

fprintf('  - 경로 길이: %.1f mm\n', path_length);
fprintf('  - 언덕 높이: %.1f mm\n', amplitude);
fprintf('  - 점 개수: %d 개\n\n', num_points);

%% 2. 위치 데이터(P) 생성
disp('2. 경로의 위치 데이터를 생성합니다...');

% X축: 0부터 path_length까지 등간격으로 점 생성
x = linspace(0, path_length, num_points)';

% Y축: 0으로 고정 (XZ 평면 상의 커브)
y = zeros(num_points, 1);

% Z축: 코사인 함수를 이용하여 부드러운 언덕 모양 생성
% (1-cos) 형태를 사용하여 시작과 끝 지점의 높이와 기울기가 0이 되도록 함
z = amplitude * 0.5 * (1 - cos(2 * pi * x / path_length));

% 위치 데이터를 (N x 3) 행렬로 결합
P = [x, y, z];
fprintf('  -> 위치 데이터(P) 생성 완료.\n\n');

%% 3. 법선 벡터(N) 데이터 생성
disp('3. 경로의 법선 벡터를 계산합니다...');
% 표면이 z = f(x, y)로 정의될 때, 법선 벡터는 [-dz/dx, -dz/dy, 1]에 비례합니다.

% dz/dx: z를 x에 대해 미분 (기울기)
dz_dx = amplitude * 0.5 * (2 * pi / path_length) * sin(2 * pi * x / path_length);

% dz/dy: y는 x에 대한 함수가 아니므로 0
dz_dy = zeros(num_points, 1);

% 법선 벡터 (정규화 전)
N_unnormalized = [-dz_dx, -dz_dy, ones(num_points, 1)];

% 법선 벡터 정규화 (단위 벡터로 변환)
norms = vecnorm(N_unnormalized, 2, 2);
N = N_unnormalized ./ norms;

fprintf('  -> 법선 벡터(N) 계산 및 정규화 완료.\n\n');


%% 4. 생성된 경로 시각화
disp('4. 생성된 경로와 법선 벡터를 시각화합니다...');
figure('Name', 'Test Trajectory', 'NumberTitle', 'off');
hold on; grid on; axis equal;
plot3(P(:,1), P(:,2), P(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Generated Traj');

% 법선 벡터를 50개 점마다 하나씩 표시 (너무 많으면 보이지 않음)
quiver3(P(1:20:end,1), P(1:20:end,2), P(1:20:end,3), ...
        N(1:20:end,1), N(1:20:end,2), N(1:20:end,3), ...
        0.5, 'b', 'DisplayName', 'Normal vector'); % 벡터 길이는 0.5mm로 표현

title(sprintf('Test Trajectory (Amplitude: %.1f mm)', amplitude));
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
legend;
view(30, 20); % 3D 뷰 각도 조절
disp('  -> 시각화 완료.');


%% 5. CSV 파일로 저장
% disp('5. 경로 데이터를 CSV 파일로 저장합니다...');
% output_filename = 'test_convex_traj.csv';
% 
% % 위치(P)와 법선 벡터(N) 데이터를 하나의 행렬로 결합
% output_data = [P, N];
% 
% % CSV 파일로 저장
% writematrix(output_data, fullfile(data_cd, output_filename));
% 
% fprintf('\n==> 성공: 경로 데이터가 ''%s'' 파일로 저장되었습니다.\n', output_filename);
% fprintf('    이제 MAIN_Curve_Path_Blending.m 스크립트를 실행하여 테스트할 수 있습니다.\n');