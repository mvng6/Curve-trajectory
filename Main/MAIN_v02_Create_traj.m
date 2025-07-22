clear;
close all;
clc;

Initialization;
Figure_setup;

%% 1) URDF 파일 불러오기
urdfFilePath = 'm1509.urdf';

robot = importrobot(urdfFilePath);
robot.Gravity = [0, 0, -9.81];

%% 2) 초기 로봇 구성 설정
ik = inverseKinematics('RigidBodyTree',robot);
weight = [0.7, 0.7, 0.7, 1, 1, 1];
endEffector = robot.BodyNames{end};
q0 = homeConfiguration(robot);

init_joints = deg2rad([90, 0, 90, 0, 90, 90]);
for i = 1:numel(q0)
    q0(i).JointPosition = init_joints(i);
end

joint_limits = [-360*pi/180, 360*pi/180;
                -360*pi/180, 360*pi/180;
                -150*pi/180, 150*pi/180;
                -360*pi/180, 360*pi/180;
                -360*pi/180, 360*pi/180;
                -360*pi/180, 360*pi/180];

%% 3) 곡면 시편 Raw 경로 데이터 불러오기
data = readmatrix('252_waypoint_faces_vectors_x.csv');

%% 4) X,Y,Z 좌표 offset
data(:,1) = data(:,1) - data(1,1) - 100;
data(:,2) = data(:,2) - data(1,2) + 340;
data(:,3) = data(:,3) - data(1,3) + 45;

%% 5) 경로의 일부 데이터만 추출
for i = 1:4
    traj_idx(i) = 2*i*10-10;
end

traj_cartesian = data(traj_idx(1):traj_idx(2),:);

% mm -> m 단위 변환
traj_cartesian(:,1:3) = traj_cartesian(:,1:3)/1000;

%% 6) Cartesian -> Quaternion 변환
traj_quaternion = zeros(size(traj_cartesian,1),7);

traj_quaternion(:,1:3) = traj_cartesian(:,1:3);
traj_quaternion(:,4:7) = eul2quat(traj_cartesian(:,4:6),'ZYZ');

%% 7) Task-Space 경로 생성
pathtform = zeros(4,4,size(traj_quaternion,1));

for idx_wp = 1:length(pathtform)
    T = se3(quat2rotm(traj_quaternion(idx_wp,4:7)), traj_quaternion(idx_wp,1:3));
    pathtform(:,:,idx_wp) = T.tform;
end

%% 8) Joint-Space 경로 생성 (Inverse Kinematics 적용)
q_prev = q0;
numPaths = length(pathtform);
pre_jointPath = zeros(numPaths, numel(q0));

for idx_joint = 1:numPaths
    clc;
    iter_joint = [idx_joint, numPaths]

    tform = pathtform(:,:,idx_joint);
    [config, ~] = ik(endEffector, tform, weight, q_prev);
    q_prev = config;

    joint_positions = [config.JointPosition];
    for j = 1:numel(joint_positions)
        joint_positions(j) = max(min(joint_positions(j), joint_limits(j,2)), joint_limits(j,1));
    end
    pre_jointPath(idx_joint,:) = joint_positions;
end

%% 9) Joint‐Space 애니메이션 시각화
% (1) Figure 및 Axes 준비
fig = figure('Name','Joint‐Space Simulation','Color','w');
ax  = axes('Parent',fig);
show(robot, q0, 'Parent',ax, 'PreservePlot', false);
view(135,20);
axis equal; grid on;
title('Joint‐Space 시뮬레이션');

% (2) 프레임 레이트 설정 (예: 초당 30프레임)
rateObj = robotics.Rate(30);

% (3) 경로 루프
numSteps = size(pre_jointPath,1);
for idx = 1:numSteps
    % 이전 q0 구조체 배열을 복사해서 사용
    qCurr = q0;
    % 각 관절 위치 업데이트
    for j = 1:numel(qCurr)
        qCurr(j).JointPosition = pre_jointPath(idx, j);
    end
    
    % 로봇 모델에 현재 관절 상태 적용
    show(robot, qCurr, ...
         'Parent',ax, ...
         'PreservePlot',false, ...
         'Frames','on', ...
         'Visuals','on');       % 'on'으로 하면 관절 좌표계도 표시됩니다.
    drawnow;
    
    % 레이트 동기화
    waitfor(rateObj);
end

disp('✅ Joint‐Space 애니메이션 완료');
