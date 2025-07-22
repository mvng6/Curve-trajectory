clear;
close all;
clc;

opengl hardware;

Initialization;
Figure_setup;

%% 1) URDF 파일 불러오기
% urdfFilePath = 'm1509.urdf';
% robot = importrobot(urdfFilePath);

meshFolder = 'D:\업무\1.과제\1.금형폴리싱\1.코드\3차년도\1. Matlab\8. Curve Traj\Robot\m1509';
robot = importrobot('m1509.urdf', 'MeshPath', {meshFolder});

robot.Gravity = [0, 0, -9.81];

%% 2) 초기 로봇 구성 설정
ik = inverseKinematics('RigidBodyTree',robot);
weight = [0.5, 0.5, 0.5, 1, 1, 1];
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

ctrl_freq = 1000;

%% 3) 곡면 시편 Raw 경로 데이터 불러오기
data = readmatrix('252waypoint.txt');

%% 4) X,Y,Z 좌표 offset
data(:,1) = data(:,1) - data(1,1) - 100;
data(:,2) = data(:,2) - data(1,2) + 340;
data(:,3) = data(:,3) - data(1,3) + 450;

%% 5) 경로의 일부 데이터만 추출
for i = 1:4
    traj_idx(i) = 2*i*10-10;
end

traj_cartesian = data(traj_idx(1):traj_idx(2),:);

% mm -> m 단위 변환
traj_cartesian(:,1:3) = traj_cartesian(:,1:3)/1000;

% deg -> rad 단위 변환
traj_cartesian(:,4:6) = deg2rad(traj_cartesian(:,4:6));

%%
diff(traj_cartesian(:,1:3))

%% 6) Cartesian -> Quaternion 변환
traj_quaternion = zeros(size(traj_cartesian,1),7);

traj_quaternion(:,1:3) = traj_cartesian(:,1:3);
traj_quaternion(:,4:7) = eul2quat(traj_cartesian(:,4:6),'ZYZ');

% 관절 각도 변화의 연속성 보장
traj_quaternion(:,4:6) = unwrap(traj_quaternion(:,4:6),[],1);

%% 7) Task-Space 경로 생성

pathtform = zeros(4,4,size(traj_quaternion,1));
for idx_task = 1:size(pathtform,3)
    cal_tform = se3(quat2rotm(traj_quaternion(idx_task,4:7)),traj_quaternion(idx_task,1:3));
    pathtform(:,:,idx_task) = cal_tform.tform;
end


%% 8) Joint-Space 경로 생성 (역기구학 계산)
q_prev = q0;

numPaths = size(pathtform,3);
jointPath = zeros(numPaths,numel(q_prev));

for idx_joint = 1:numPaths

    tform = pathtform(:,:,idx_joint);
    [config,~] = ik(endEffector, tform, weight, q_prev);

    q_prev = config;

    joint_positions = [config.JointPosition];
    for j = 1:numel(joint_positions)
        joint_positions(j) = max(min(joint_positions(j), joint_limits(j,2)), joint_limits(j,1));
    end

    jointPath(idx_joint,:) = joint_positions;
end


%% 9) Result Plot
joint_vel = diff(rad2deg(jointPath)) * ctrl_freq;
joint_acc = diff(joint_vel) * ctrl_freq;
joint_jerk = diff(joint_acc) * ctrl_freq;

figure;
for i = 1:6
    subplot(2,3,i);
    plot(joint_vel(:,i));
    title(sprintf('Joint %d',i));
end

figure;
for i = 1:6
    subplot(2,3,i);
    plot(joint_acc(:,i));
    title(sprintf('Joint %d',i));
end

figure;
for i = 1:6
    subplot(2,3,i);
    plot(joint_jerk(:,i));
    title(sprintf('Joint %d',i));
end

%% 10) Simulation

Simulation_steps = size(jointPath,1);

t_start_total = tic;
% figure;
ax = show(robot,q0,'PreservePlot',false);
hold on;
grid on;

for i = 1:Simulation_steps
    for j = 1:numel(q0)
        q0(j).JointPosition = jointPath(i,j);
    end

    show(robot,q0,'PreservePlot',false,'Parent',ax);

    drawnow limitrate;
end

toc(t_start_total);