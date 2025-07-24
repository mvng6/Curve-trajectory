% =========================================================================
% 로봇 폴리싱을 위한 관절 공간 경로 계획
% 최종 수정: 2025-07-24
% 작성자: LDJ
% =========================================================================

%%
clear; close all; clc;

opengl hardware;

Initialization;
Figure_setup;

%% 1단계: 데이터 로딩 및 TCP 적용을 위한 좌표 변환
disp('1단계: 데이터 로딩 및 좌표 변환 시작...');

% 데이터 파일 로드
waypoint_raw = readmatrix('252waypoint.txt');

% 경로 높이 오프셋
waypoint_raw(:,3) = waypoint_raw(:,3) - waypoint_raw(1,3);

% TCP 오프셋 설정 (플랜지 -> TCP). Z축으로 353mm.
T_flange_to_tcp = trvec2tform([0 0 0.353]);   % [m]

R = T_flange_to_tcp(1:3,1:3);       % 회전 행렬 추출
p = T_flange_to_tcp(1:3,4);         % 이동 벡터 추출
Rt = R';                            % 회전 행렬의 전치 행렬
p_inv = -Rt * p;                    % 새로운 이동 벡터 계산

T_tcp_to_flange = eye(4);
T_tcp_to_flange(1:3, 1:3) = Rt;
T_tcp_to_flange(1:3,4) = p_inv;

% 모든 웨이포인트에 대해 플랜지의 Pose 계산
num_waypoints = size(waypoint_raw,1);
flange_poses = zeros(4,4,num_waypoints);

for i = 1:num_waypoints
    trvec = waypoint_raw(i,1:3) / 1000;    % mm -> m 변환
    eul_zyz = deg2rad(waypoint_raw(i,4:6)); % deg -> rad 변환

    P_waypoint = eul2tform(eul_zyz, 'ZYZ');
    P_waypoint(1:3,4) = trvec';

    flange_poses(:,:,i) = P_waypoint * T_tcp_to_flange;
end

disp('1단계 완료.');
disp('---');

%% 2단계 로봇 모델 생성 및 역기구학(IK) 계산
disp('2단계: 역기구학 계산 시작...');

% 로봇 로드
meshFolder = 'D:\업무\1.과제\1.금형폴리싱\1.코드\3차년도\1. Matlab\8. Curve Traj\Robot\m1509';
robot = importrobot('m1509.urdf', 'DataFormat', 'column' ,'MeshPath', {meshFolder});

% IK 솔버 설정
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1];                   % 위치 정확도에 더 높은 가중치 설정함

home_position = homeConfiguration(robot);           % 모든 관절이 0인 홈포지션
initial_guess = deg2rad([90, 0, 90, 0, 90, 90]);    % 폴리싱 초기 자세를 위해 선언한 홈포지션

num_joints = numel(initial_guess);
joint_waypoints = zeros(num_joints, num_waypoints);
end_effector_name = robot.BodyNames{end};
q_prev = initial_guess';

% 각 관절의 위치 제한 [min, max] 정보를 가져옴
joint_limits = zeros(num_joints, 2);
for i = 1:num_joints    
    joint_limits(i, :) = robot.Bodies{i}.Joint.PositionLimits;
end

% 모든 웨이포인트에 대해 IK 계산
for i = 1:num_waypoints
    % clc;
    Iter_IK = [i, num_waypoints]

    target_pose = flange_poses(:,:,i);
    
    % 1. 다양한 초기 추측값(후보) 생성
    candidate_guesses = {};
    %   (1) 기준 후보
    candidate_guesses{end+1} = q_prev;
    
    %   (2) 기구학적 변형
    q_elbow_flip = q_prev;
    q_elbow_flip(4) = -q_elbow_flip(4);
    if all(q_elbow_flip >= joint_limits(:,1) & q_elbow_flip <= joint_limits(:,2))
        candidate_guesses{end+1} = q_elbow_flip;
    end

    q_wrist_flip = q_prev;
    q_wrist_flip(5) = -q_wrist_flip(5);
    q_wrist_flip(6) = q_wrist_flip(6) + pi;
    if all(q_wrist_flip >= joint_limits(:,1) & q_wrist_flip <= joint_limits(:,2))
        candidate_guesses{end+1} = q_wrist_flip;
    end

    %   (3) 랜덤 샘플링
    q_random = [];
    pertubation = (rand(num_joints, 1) - 0.5) * 2 * deg2rad(15);
    q_random = q_prev + pertubation;
    if all(q_random >= joint_limits(:,1) & q_random <= joint_limits(:,2))
        candidate_guesses{end+1} = q_random;
    end

    %   (4) 안전 후보
    q_home = home_position;
    candidate_guesses{end+1} = q_home;

    % 2. 각 후보에 대해 IK 계산 시도
    valid_solutions = [];
    for k = 1:length(candidate_guesses)
        [q_sol,sol_info] = ik(end_effector_name, target_pose, weights, candidate_guesses{k});

        if (sol_info.ExitFlag > 0)
            valid_solutions = [valid_solutions, q_sol];
        end
    end

    % 3. 최적의 해 선택
    if isempty(valid_solutions)
        warning('웨이포인트 %d에 대한 IK 해를 찾을 수 없습니다. 이전 해를 유지합니다.', i);
        joint_waypoints(:, i) = q_prev; % 해를 못 찾으면 이전 값 유지
    else
        % 각 유효 해와 이전 관절 상태(q_prev) 사이의 거리(유클리드 거리) 계산
        distances = vecnorm(valid_solutions - q_prev);

        % 거리가 가장 짧은 해를 선택
        [~, min_idx] = min(distances);
        best_sol_vec = valid_solutions(:, min_idx);

        % 결과 저장 및 다음 루프를 위해 q_prev 업데이트
        joint_waypoints(:,i) = best_sol_vec;
        q_prev = best_sol_vec;
    end

end

clc;
disp('2단계 완료.');
disp('---');

%% =================================================================
% 단계 2.5: 경로 스무딩 (표준 spline 함수 사용 - 시간 축 수정)
% =================================================================
disp('2.5단계: IK 경로 스무딩 시작 (시간 축 수정)...');

% --- [핵심 수정 1] 모든 시간 기준을 0 ~ total_time 으로 통일 ---
total_time = 60;
waypoint_times = linspace(0, total_time, num_waypoints); % 원본 시간 (0~60초)

% 1. 원본 경로점에서 일부(knot points)만 선택하여 경로를 재구성
num_knot_points = 50; 
indices = round(linspace(1, num_waypoints, num_knot_points));

knot_waypoints = joint_waypoints(:, indices);
knot_times = waypoint_times(indices); % 원본 시간 스케일(0~60초)에서 선택

% 2. 선택된 점(knot points)들을 'spline' 함수로 부드럽게 보간
ts = 0.01;
traj_times = 0:ts:total_time; % 최종 시간 또한 동일한 스케일(0~60초)

% spline 함수는 각 관절(행)에 대해 개별적으로 적용
smoothed_waypoints = zeros(num_joints, numel(traj_times));
for i = 1:num_joints
    smoothed_waypoints(i, :) = spline(knot_times, knot_waypoints(i, :), traj_times);
end

disp(['경로점을 ' num2str(num_waypoints) '개에서 ' num2str(num_knot_points) '개의 주요 지점으로 재구성 후 스무딩 완료.']);
disp('---');

%% 3단계: 5차 다항식을 이용한 관절 공간 경로 보간
disp('3단계: 경로 보간 시작...');

% total_time = 30; % 초
% waypoint_times = linspace(0, total_time, num_waypoints);
% ts = 0.01;
% traj_times = 0:ts:waypoint_times(end);
% num_samples = numel(traj_times);
% 
% [q, qd, qdd] = minjerkpolytraj(joint_waypoints, waypoint_times, num_samples);

disp('3단계: 스무딩된 경로의 속도/가속도 계산 시작...');

% 2.5단계에서 최종 위치 경로 q가 'smoothed_waypoints'로 계산되었음
q = smoothed_waypoints;

% 위치(q)를 시간에 대해 미분하여 속도(qd)와 가속도(qdd) 계산
qd = diff(q, 1, 2) / ts;
qdd = diff(qd, 1, 2) / ts;

% 미분으로 인해 행렬의 열 개수가 줄어들었으므로, 길이를 맞춰주기 위해 마지막 값을 복사하여 추가
qd(:, end+1) = qd(:, end);
qdd(:, end+1:end+2) = [qdd(:, end), qdd(:, end)];

disp('3단계 완료.');
disp('---');

%% 4단계: 시뮬레이션 및 결과 시각화
disp('4단계: 시뮬레이션 및 결과 시각화 시작...');

% 관절 각도, 속도, 가속도 그래프
figure('Name', 'Joint-Space Trajectory Profiles');
subplot(3,1,1);
plot(traj_times, rad2deg(q));
title('Joint Position'); ylabel('Position (deg)'); grid on;
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
subplot(3,1,2);
plot(traj_times, rad2deg(qd));
title('Joint Velocity'); ylabel('Velocity (deg/s)'); grid on;
subplot(3,1,3);
plot(traj_times, rad2deg(qdd));
title('Joint Acceleration'); ylabel('Accel (deg/s^2)'); grid on;
xlabel('Time (s)');

%%
q0 = initial_guess;
figure('Name', 'Robot Motion Simulation');
ax = show(robot, q0', 'PreservePlot', false);
hold on; grid on;
for i = 1:length(q)
    q0 = q(:,i);
    show(robot, q0, 'PreservePlot', false, 'Parent', ax);
    drawnow limitrate;
end

%% q, qd, qdd 데이터를 시트로 구분해서 저장
filename = 'all_joint_data.xlsx';

writematrix(q, filename, 'Sheet', 'q');
writematrix(qd, filename, 'Sheet', 'qd');
writematrix(qdd, filename, 'Sheet', 'qdd');