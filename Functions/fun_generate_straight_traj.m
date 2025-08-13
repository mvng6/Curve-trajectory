function [straight_pos_traj, straight_vel_traj] = fun_generate_straight_traj(traj_start,desired_move_len_mm, direction, robot_speed_mmps, dt)
% 직선 구간에 대한 속도 프로파일 생성
%
% 입력
%   traj_start          : 직선 경로 시작 위치 좌표
%   desired_move_len_mm : 로봇 목표 이동 거리
%   direction           : 로봇 이동 방향
%   robot_speed_mmps    : 로봇 이동 속도
%   dt                  : 제어 주기
%
% 출력
%   straight_pos_traj   : 직선 구간 위치 경로 (Nx6 매트릭스) - [x,y,z,Nx,Ny,Nz]
%   straight_vel_traj   : 직선 구간 속도 경로 (Nx6 매트릭스) - [vx,vy,vz,wx,wy,wz]

    
    % 로봇 이동 방향
    unit_direction = direction / norm(direction);
    
    % 주기
    control_freq_hz = 1 / dt;
    
    % 총 이동 시간 계산
    totalTime = desired_move_len_mm / robot_speed_mmps;
    
    % 시간 벡터 생성
    traj_t = (0:dt:totalTime)';

    traj_t_accel = totalTime * 0.1;                             % 가속에 걸리는 시간 (s)
    traj_t_decel = traj_t_accel;                                % 감속에 걸리는 시간 (s)
    traj_t_const = totalTime - traj_t_accel - traj_t_decel;     % 등속도 구간 시간 (s)

    % 등속 직선 경로 생성
    speedProfile = zeros(size(traj_t));
    
    %   (A) 가속 구간 생성
    t_accel = (0:dt:traj_t_accel)';
    accel_shape = quinticpolytraj([0, 1], [0, traj_t_accel], t_accel);
    speedProfile(1:length(t_accel)) = accel_shape * robot_speed_mmps;
    
    %   (B) 등속 구간 생성
    start_const_idx = length(t_accel) + 1;
    num_const_points = round(traj_t_const * control_freq_hz);
    end_const_idx = start_const_idx + num_const_points - 1;
    speedProfile(start_const_idx:end_const_idx) = robot_speed_mmps;
    
    %   (C) 감속 구간 생성
    start_decel_idx = end_const_idx + 1;
    num_decel_points = length(traj_t) - start_decel_idx + 1;
    t_decel_corrected = linspace(0, traj_t_decel, num_decel_points);
    decel_shape = quinticpolytraj([1, 0], [0, traj_t_decel], t_decel_corrected);
    speedProfile(start_decel_idx:end) = decel_shape * robot_speed_mmps;
    
    straight_vel_traj = unit_direction .* speedProfile;

    %% 속도 프로파일에 따른 로봇 위치 좌표 및 벡터
    % 1. 위치 경로 계산
    % 각 시간 스텝까지의 누적 이동 거리(스칼라)를 계산합니다. (속도 적분)
    distanceProfile_raw = cumsum(speedProfile) * dt;
    scaling_factor = desired_move_len_mm / distanceProfile_raw(end);
    distanceProfile = distanceProfile_raw * scaling_factor;

    % 2. XYZ 위치 좌표 계산
    % 시작 위치(xyz)에 각 스텝별 변위(이동거리 * 방향)를 더합니다.
    % speedProfile과 같은 Nx1 벡터와 unit_direction 1x3 벡터를 곱해 Nx3 위치 행렬을 만듭니다.
    pos_traj_xyz = traj_start(1:3) + distanceProfile .* unit_direction(1:3);

    % 3. 로봇 경로 벡터 정보 결합
    % 경로점 개수만큼 시작 자세를 복제합니다.
    num_points = length(speedProfile);
    pos_traj_orient = repmat(traj_start(4:6), num_points, 1);
    
    % 4. 최종 위치 경로(Nx6) 생성
    straight_pos_traj = [pos_traj_xyz, pos_traj_orient];
end