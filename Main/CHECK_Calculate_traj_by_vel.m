%%
% Info
% 리샘플링 주기가 동일할 때 이동 속도에 따른 경로 샘플 데이터 차이 확인용 코드

%%
% =========================================================================
% 초기화
% =========================================================================
clear;      % 작업 공간의 모든 변수 삭제
close all;  % 열려 있는 모든 Figure 창 닫기
clc;        % 명령 창 내용 지우기

Initialization;
% Figure_setup;

%%
% =========================================================================
% 1. 데이터 로딩
% =========================================================================
% 'test_curve_traj.csv' 파일이 MATLAB 경로에 있는지 확인
% 파일은 (N x 6) 크기이며, 각 행은 [Px, Py, Pz, Nx, Ny, Nz] 형식이어야 합니다.
disp('CSV 파일에서 경로 데이터를 로딩합니다...');
try
    data = readmatrix('test_concave_traj.csv');
    P = data(:,1:3); % 위치 데이터 (N x 3)
    N = data(:,4:6); % 법선 벡터 데이터 (N x 3)
    
    % 법선 벡터가 단위 벡터가 아닐 수 있으므로 정규화를 수행합니다. (안전 장치)
    N = N ./ vecnorm(N, 2, 2);
    
    fprintf('성공: %d개의 포인트 데이터를 로드했습니다.\n\n', size(P, 1));
catch ME
    error('오류: test_curve_traj.csv 파일을 찾을 수 없거나 읽을 수 없습니다. 파일 경로를 확인해주세요.\n오류 메시지: %s', ME.message);
end

%%
% ========================================================================
% 2. 속도 비교를 위한 리샘플링 실행
% =========================================================================
% --- 고정 변수 설정 ---
resampling_time_step = 0.1; % 리샘플링 주기

% --- Case 1 ---
speed1 = 5; % mm/s
fprintf('[Case 1] 속도 %d mm/s에 대한 리샘플링을 수행합니다...',speed1);
[points1, normals1, time1] = fun_resamplePathByTime(P, N, speed1, resampling_time_step);
disp('완료.');

% --- Case 2 ---
speed2 = 30; % mm/s
fprintf('[Case 2] 속도 %d mm/s에 대한 리샘플링을 수행합니다...',speed2);
[points2, normals2, time2] = fun_resamplePathByTime(P, N, speed2, resampling_time_step);
disp('완료.');
fprintf('\n');

%%
% =========================================================================
% 3. 결과 출력
% =========================================================================
fprintf('--- [비교 결과] ---\n');
fprintf('고정된 시간 주기: %.3f 초 (%.1f Hz)\n', resampling_time_step, 1/resampling_time_step);
fprintf('------------------------------------------\n');
fprintf('[Case 1: 속도 = %.1f mm/s]\n', speed1);
fprintf('  - 총 이동 시간: %.4f 초\n', time1);
fprintf('  - 생성된 포인트 개수: %d 개\n', size(points1, 1));
fprintf('  - 포인트 간 평균 거리: %.4f mm\n', mean(vecnorm(diff(points1), 2, 2)));
fprintf('------------------------------------------\n');
fprintf('[Case 2: 속도 = %.1f mm/s]\n', speed2);
fprintf('  - 총 이동 시간: %.4f 초\n', time2);
fprintf('  - 생성된 포인트 개수: %d 개\n', size(points2, 1));
fprintf('  - 포인트 간 평균 거리: %.4f mm\n', mean(vecnorm(diff(points2), 2, 2)));
fprintf('------------------------------------------\n');

%%
% =========================================================================
% 4. 비교 시각화
% =========================================================================
disp('결과를 시각화합니다...');
figure('Name', '속도별 리샘플링 결과 비교', 'NumberTitle', 'off');
sgtitle('속도에 따른 리샘플링 결과 비교'); % 전체 플롯의 제목

% Subplot 1
ax1 = subplot(1, 2, 1);
hold on; grid on; axis equal;
plot3(P(:,1), P(:,2), P(:,3), 'k.--', 'DisplayName', 'Original Traj');

n_points_1 = length(points1);
step_1 = floor(n_points_1 / 100);
% for i = 1:step_1:n_points_1
%     plot3(points1(i,1), points1(i,2), points1(i,3), 'ro-', 'LineWidth', 1.5, 'DisplayName', 'Resampled Traj');
%     quiver3(points1(i,1), points1(i,2), points1(i,3), ...
%         normals1(i,1)*50, normals1(i,2)*50, normals1(i,3)*50, ...
%         'k','LineWidth',1,'DisplayName','Normal Vector');
% end
plot3(points1(:,1), points1(:,2), points1(:,3), 'ro-', 'LineWidth', 1.5, 'DisplayName', 'Resampled Traj');
quiver3(points1(:,1), points1(:,2), points1(:,3), ...
    normals1(:,1)*50, normals1(:,2)*50, normals1(:,3)*50, ...
    'k','LineWidth',1,'DisplayName','Normal Vector');
title(sprintf('Speed = %d mm/s (Point %d)', speed1, size(points1,1)));
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Original Traj','Resampled Traj',' Normals');
view(3);

% Subplot 2
ax2 = subplot(1, 2, 2);
hold on; grid on; axis equal;
plot3(P(:,1), P(:,2), P(:,3), 'k.--', 'DisplayName', 'Original Traj');

n_points_2 = length(points2);
step_2 = floor(n_points_2 / 100);
% for i = 1:step_2:n_points_2
%     plot3(points2(i,1), points2(i,2), points2(i,3), 'bo-', 'LineWidth', 1.5, 'DisplayName', 'Resampled Traj');
%     quiver3(points2(i,1), points2(i,2), points2(i,3), ...
%         normals2(i,1)*50, normals2(i,2)*50, normals2(i,3)*50, ...
%         'k','LineWidth',1,'DisplayName','Normal Vector');
% end
plot3(points2(:,1), points2(:,2), points2(:,3), 'bo-', 'LineWidth', 1.5, 'DisplayName', 'Resampled Traj');
quiver3(points2(:,1), points2(:,2), points2(:,3), ...
    normals2(:,1)*50, normals2(:,2)*50, normals2(:,3)*50, ...
    'k','LineWidth',1,'DisplayName','Normal Vector');
title(sprintf('Speed = %d mm/s (Point %d)', speed2, size(points2,1)));
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Original Traj','Resampled Traj',' Normals');
view(3);

% 두 서브플롯의 축과 시점을 동기화하여 비교하기 쉽게 만듭니다.
linkaxes([ax1, ax2], 'xyz');
linkprop([ax1, ax2], {'CameraPosition', 'CameraUpVector'});
disp('시각화 완료.');
