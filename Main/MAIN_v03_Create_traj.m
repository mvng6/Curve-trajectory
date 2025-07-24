clear;
close all;
clc;

opengl hardware;

Initialization;
Figure_setup;

%% 1) URDF 파일 불러오기
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

data = flipud(data);

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

% 각 구간 사이의 길이 계산
d = sqrt(sum(diff(traj_cartesian(:,1:3)).^2,2));

s = [0; cumsum(d)];

%% 6) 원하는 이동 속도에 대한 시간 정하기
v_des   = 0.01;              % [m/s]
dt      = 0.1;            % 1kHz 제어 간격
L       = s(end);           % 경로 전체 길이
T       = L/v_des;          % 전체 이동 소요 시간
t_new   = (0:dt:T)';        % 0초에서 T 초까지 1ms(dt) 간격
s_new   = v_des * t_new;    % 시간 -> 호 길이 매핑 : t_new -> s_new

%% 7) 호 길이 기반 등속도 재샘플링

% (1) 원본 위치와 자세(쿼터니언) 분리
pts = traj_cartesian(:,1:3);
quat_orig = eul2quat(traj_cartesian(:,4:6),'ZYZ');
quat_orig = unwrap(quat_orig,[],1);

% (2) 등간격 호 길이 s_new에서 보간
x_new = interp1(s, pts(:,1), s_new, 'pchip');
y_new = interp1(s, pts(:,2), s_new, 'pchip');
z_new = interp1(s, pts(:,3), s_new, 'pchip');

% 쿼터니언 성분별 보간 후 정규화
q_new = zeros(numel(s_new),4);
for k = 1:4
    q_new(:,k) = interp1(s,quat_orig(:,k),s_new,'pchip');
end

% 정규화(normalize) - 보간 뒤 길이가 1이 되도록
q_new = q_new ./ vecnorm(q_new,2,2);

% (3) 보간한 위치 및 자세를 이용해 Task-space 변환행렬 생성
numNew = numel(s_new);
pathtform_new = zeros(4,4,numNew);

for i = 1:numNew
    R = quat2rotm(q_new(i,:));
    t = [x_new(i);y_new(i);z_new(i)];
    pathtform_new(:,:,i) = [R, t; 0 0 0 1];
end

%% 8) Joint-space 경로 생성 (역기구학)
q_prev = q0;
jointPath = zeros(numNew, numel(q0));
for i = 1:numNew
    tgt = pathtform_new(:,:,i);
    [config, ~] = ik(endEffector,tgt,weight,q_prev);

    % 현재 관절 구성(config)으로 기구학적 자코비안 행렬 계산
    J = geometricJacobian(robot, config, endEffector);

    % 조작성 지수 계산 및 저장 (det(J*J')의 제곱근)
    manipulability(i) = sqrt(det(J*J'));

    q_prev = config;
    jp = [config.JointPosition];

    % joint limit 클램핑
    for j = 1:numel(jp)
        jp(j) = max(min(jp(j), joint_limits(j,2)), joint_limits(j,1));
    end
    jointPath(i,:) = jp;
end

%% 9) Simulation
% figure;
ax = show(robot, q0, 'PreservePlot', false);
hold on; grid on;
for i = 1:numNew
    for j = 1:numel(q0)
        q0(j).JointPosition = jointPath(i,j);
    end
    show(robot, q0, 'PreservePlot', false, 'Parent', ax);
    drawnow limitrate;
end

%%
figure;
for i = 1:6
    subplot(2,3,i);
    plot(jointPath(:,i));

    title(sprintf('Joint%d',i));
end

% 조작성 지수 그래프 추가
figure;
plot(t_new, manipulability);
title('Manipulability Index');
xlabel('Time (s)');
ylabel('Manipulability');
grid on;

% (심화) 특정 관절과 조작성 지수 함께 보기
figure;
yyaxis left; % 왼쪽 y축 사용
plot(t_new, jointPath(:,5)); % 예시: Joint 5
ylabel('Joint 5 Angle (rad)');
yyaxis right; % 오른쪽 y축 사용
plot(t_new, manipulability);
ylabel('Manipulability');
title('Joint 5 Angle vs. Manipulability');
xlabel('Time (s)');
grid on;