% =========================================================================
% 로봇 경로 블렌딩 메인 스크립트
% =========================================================================
clear; close all; clc;

%% 1. 시뮬레이션 설정 (Configuration)
% 주요 파라미터들을 구조체(struct)로 묶어 관리하면 가독성과 확장성이 좋아집니다.
config.blending.distance = 10.0; % 블렌딩 거리 (mm)

config.detection.lookaheadPoints = 30; % 코너 탐지 시 살펴볼 점의 거리
config.detection.angleThreshold = 80;  % 코너로 인식할 최소 각도 (degrees)

%% 2. 경로 생성 (Path Generation)
% 이 부분만 교체하면 다른 어떤 경로에도 블렌딩을 적용할 수 있습니다.
fprintf('--- 사용자님께서 제공한 코드로 경로를 생성합니다 ---\n');

fprintf('테스트 경로를 생성합니다...\n');
[P, N] = generate_test_path();

%% 3. 핵심 로직 실행 (Core Logic Execution)
% 각 기능이 함수화되어 코드가 매우 간결해집니다.
fprintf('--- 경로 내 코너점 탐색 및 블렌딩을 시작합니다 ---\n');

% 3-1. 경로에서 코너점 찾기
corner_indices = fun_find_path_corners(P, config.detection);

% 3-2. 찾은 코너점에 블렌딩 적용하기
[blended_path, blended_normals] = fun_apply_path_blending_normals(P,N, corner_indices, config.blending.distance);

%% 4. 결과 시각화 (Visualization)
% fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');
% figure; 
% hold on; 
% grid on; 
% axis equal;
% view(30, 45);
% 
% plot3(P(:,1), P(:,2), P(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 경로');
% plot3(blended_path(:,1), blended_path(:,2), blended_path(:,3), 'r-', 'LineWidth', 2.5, 'DisplayName', '최종 블렌딩 경로');
% 
% title('리팩토링된 경로 블렌딩 알고리즘');
% xlabel('X (mm)'); 
% ylabel('Y (mm)'); 
% zlabel('Z (mm)');
% legend('show', 'Location', 'best');
% rotate3d on;

%% [수정된] 시각화 코드
fprintf('--- 시뮬레이션 완료. 결과를 그래프로 확인하세요 ---\n');

% --- 시각화 파라미터 설정 ---
plot_interval = 3;  % 벡터를 표시할 간격 (너무 촘촘하면 보이지 않음)
vector_length = 1;   % 표시될 벡터(화살표)의 길이 (mm)

% --- 그래프 생성 ---
figure('Name', '블렌딩 경로 및 법선 벡터 시각화', 'NumberTitle', 'off');
hold on;
grid on;
% axis equal;
% view(30, 45);

% 1. 원본 경로 및 블렌딩된 경로 플로팅
plot3(P(:,1), P(:,2), P(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', '원본 경로');
plot3(blended_path(:,1), blended_path(:,2), blended_path(:,3), 'r-', 'LineWidth', 2.5, 'DisplayName', '최종 블렌딩 경로');

% 2. ★★★ 블렌딩된 경로의 법선 벡터(툴 방향) 시각화 ★★★
% quiver3(X, Y, Z, U, V, W) -> (X,Y,Z) 위치에서 (U,V,W) 벡터를 그림
quiver3(blended_path(1:plot_interval:end, 1), ... % 벡터 시작점 X좌표
        blended_path(1:plot_interval:end, 2), ... % 벡터 시작점 Y좌표
        blended_path(1:plot_interval:end, 3), ... % 벡터 시작점 Z좌표
        -blended_normals(1:plot_interval:end, 1), ... % 벡터 U 성분 (툴 방향은 법선의 반대)
        -blended_normals(1:plot_interval:end, 2), ... % 벡터 V 성분
        -blended_normals(1:plot_interval:end, 3), ... % 벡터 W 성분
        vector_length, ... % 벡터 길이 스케일
        'b', 'LineWidth', 1, 'DisplayName', '블렌딩된 툴 방향');

title('블렌딩 된 경로에서의 보간된 법선 벡터');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
legend('show', 'Location', 'best');
rotate3d on;