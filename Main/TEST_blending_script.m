% =========================================================================
% 로봇 경로 블렌딩 메인 스크립트
% =========================================================================
clear; close all; clc;

%% 1. 시뮬레이션 설정 (Configuration)
% 주요 파라미터들을 구조체(struct)로 묶어 관리하면 가독성과 확장성이 좋아집니다.
config.blending.distance = 5.0; % 블렌딩 거리 (mm)

config.detection.lookaheadPoints = 30; % 코너 탐지 시 살펴볼 점의 거리
config.detection.angleThreshold = 80;  % 코너로 인식할 최소 각도 (degrees)

%% 2. 경로 생성 (Path Generation)
% 이 부분만 교체하면 다른 어떤 경로에도 블렌딩을 적용할 수 있습니다.
fprintf('--- 사용자님께서 제공한 코드로 경로를 생성합니다 ---\n');

P_line_start = [0, 0, 0];
P_line_end = [0, 50, 0];
path_len = 100;

path_1 = [zeros(1,path_len); linspace(P_line_start(2),P_line_end(2),path_len); zeros(1,path_len)];
arc_radius = 20.0;
arc_center = [path_1(1,end) + arc_radius, path_1(2,end), path_1(3,end)];
arc_start_angle = pi;
arc_end_angle = 3*pi/2;
theta_vis = linspace(arc_start_angle, arc_end_angle, path_len);
path_2 = [arc_center(1) + arc_radius*cos(theta_vis); arc_center(2)*ones(1,100); arc_center(3)+arc_radius*sin(theta_vis)];
path_3 = [ones(1,path_len) * path_2(1,end); linspace(path_2(2,end),P_line_start(2),path_len); ones(1,path_len) * path_2(3,end)];
theta_vis_4 = linspace(pi, 3*pi/2, path_len);
path_4 = [arc_center(1) + arc_radius*cos(theta_vis_4); arc_center(2)*ones(1,100); arc_center(3)+arc_radius*sin(theta_vis_4)];
path_4(1,:) = path_4(1,:) + path_3(1,end);
path_4(2,:) = path_4(2,:) * path_3(2,end);
path_4(3,:) = path_4(3,:) + path_3(3,end);

path_5 = [ones(1,path_len).*path_4(1,end); linspace(P_line_start(2),P_line_end(2),path_len);ones(1,path_len).*path_4(3,end)];

theta_vis_6 = linspace(pi, 3*pi/2, path_len);
path_6 = [arc_center(1) + arc_radius*cos(theta_vis_6); arc_center(2)*ones(1,100); arc_center(3)+arc_radius*sin(theta_vis_6)];
path_6(1,:) = path_6(1,:) + path_5(1,end);
path_6(2,:) = path_6(2,:);
path_6(3,:) = path_6(3,:) + path_5(3,end);

original_path = [path_1, path_2, path_3, path_4, path_5, path_6]';
fprintf('총 %d개의 경로점으로 구성된 원본 경로 생성 완료.\n', size(original_path, 1));

%% 3. 핵심 로직 실행 (Core Logic Execution)
% 각 기능이 함수화되어 코드가 매우 간결해집니다.
fprintf('--- 경로 내 코너점 탐색 및 블렌딩을 시작합니다 ---\n');

% 3-1. 경로에서 코너점 찾기
corner_indices = fun_find_path_corners(original_path, config.detection);

% 3-2. 찾은 코너점에 블렌딩 적용하기
blended_path = fun_apply_path_blending(original_path, corner_indices, config.blending);

%% 4. 결과 시각화 (Visualization)
fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');
figure; 
hold on; 
grid on; 
axis equal;
view(30, 45);

plot3(original_path(:,1), original_path(:,2), original_path(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Original Traj');
plot3(blended_path(:,1), blended_path(:,2), blended_path(:,3), 'r-', 'LineWidth', 2.5, 'DisplayName', 'Blended Traj');

title('Trajectory Blending Algorithm');
xlabel('X (mm)'); 
ylabel('Y (mm)'); 
zlabel('Z (mm)');
legend('show', 'Location', 'northeast');
rotate3d on;