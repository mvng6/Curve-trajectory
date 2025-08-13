function [curve_pos_traj, curve_vel_traj] = fun_generate_curve_traj(start_position, interp_position, interp_normal, omega, interp_dist_mm, direction, curve_move_len_mm, offset,home_flag)
% 곡면 금형의 특정 구간 추출
%
% 입력
%   start_position      : 초기 위치 좌표 및 법선 벡터 (X, Y, Z, Nx, Ny, Nz)
%   interp_position     : spline으로 보간 된 위치 좌표 행렬
%   omega               : SLERP로 보간된 법선 벡터의 각속도 행렬
%   interp_dist_mm      : 누적 이동 거리
%   direction:          : 곡면 구동 방향
%   curve_move_len_mm   : 원하는 곡면 이동 거리
%   offset              : y 위치 좌표 오프셋 [mm]
%   home_flag           : 초기 시작 위치 복귀 여부 플래그
%
% 출력
%   curve_pos_traj      : 곡선 구간 위치 경로 (Nx6 매트릭스) - [x,y,z,Nx,Ny,Nz]
%   curve_vel_traj      : 곡선 구간 속도 경로 (Nx6 매트릭스) - [vx,vy,vz,wx,wy,wz]
    
    if (home_flag == 0)
        % 로봇 이동 방향
        unit_direction = direction / norm(direction);
    
        %% 위치 경로 생성
        % (1) 시작 위치와 가장 가까운 점의 인덱스 찾기
        distances_from_start = vecnorm(interp_position(:,1:3) - start_position(:,1:3), 2, 2);
        [~, start_index] = min(distances_from_start);
    
        % (2) 시작 인덱스의 누적 거리 찾기
        start_dist_mm = interp_dist_mm(start_index);
    
        % (3) 목표 누적 거리 계산
        target_dist_mm = start_dist_mm + curve_move_len_mm;
    
        % (4) 목표 누적 거리에 가장 가까운 점의 인덱스 찾기
        [~, final_index] = min(abs(interp_dist_mm - target_dist_mm));
        
        % 경로 끝 경계 처리
        if final_index >= length(interp_dist_mm)
            final_index = length(interp_dist_mm);
        end
        
        % (5) 최종 곡선 구간의 위치/자세 경로(Nx6) 추출
        curve_pos_traj = [interp_position(start_index:final_index, :), interp_normal(start_index:final_index,:)];
        curve_pos_traj(:,2) = curve_pos_traj(:,2) + ones(length(curve_pos_traj),1)*offset;
    
        % (6) 경로 포인트 개수 확인
        num_path_points = size(curve_pos_traj, 1);
        if num_path_points < 3 % 가감속 프로파일을 만들기에 점이 너무 적으면 경고
            warning('경로 점 개수가 너무 적어 부드러운 속도 프로파일을 생성할 수 없습니다.');
            curve_vel_traj = zeros(num_path_points, 3); % 속도는 0으로 반환
            return;
        end
    
        %% 속도 프로파일 생성
        robot_speed = max(omega(start_index:final_index));
        speedProfile = zeros(num_path_points, 1);
        
        % 가감속 구간을 전체 경로점의 15%씩 할당 (S-커브는 약간 더 여유있게)
        num_accel_points = round(num_path_points * 0.15);
        num_decel_points = round(num_path_points * 0.15);
        
        % [수정] 가감속 포인트가 너무 적을 경우를 대비한 최소값 설정
        if num_accel_points < 2, num_accel_points = 2; end
        if num_decel_points < 2, num_decel_points = 2; end
        
        num_const_points = num_path_points - num_accel_points - num_decel_points;
    
        %   (A) 가속 구간 생성 (S-커브)
        % [수정] linspace 대신 quinticpolytraj 사용
        % '시간' 대신 '지점 인덱스'를 기준으로 폴리노미얼을 생성합니다.
        t_poly_accel = 1:num_accel_points;
        accel_waypoints = [0, robot_speed];
        time_waypoints_accel = [0, num_accel_points];
        accel_shape = quinticpolytraj(accel_waypoints, time_waypoints_accel, t_poly_accel);
        speedProfile(1:num_accel_points) = accel_shape;
        
        %   (B) 등속 구간 생성
        start_const_idx = num_accel_points + 1;
        end_const_idx = start_const_idx + num_const_points - 1;
        speedProfile(start_const_idx:end_const_idx) = robot_speed;
        
        %   (C) 감속 구간 생성 (S-커브)
        % [수정] linspace 대신 quinticpolytraj 사용
        start_decel_idx = end_const_idx + 1;
        t_poly_decel = 1:num_decel_points;
        decel_waypoints = [robot_speed, 0];
        time_waypoints_decel = [0, num_decel_points];
        decel_shape = quinticpolytraj(decel_waypoints, time_waypoints_decel, t_poly_decel);
        speedProfile(start_decel_idx:end) = decel_shape;
        
        curve_vel_traj = speedProfile .* unit_direction;

    elseif (home_flag == 1)
        % 로봇 이동 방향
        unit_direction = direction / norm(direction);
        
        %% 위치 경로 생성
        % (1) 시작 위치와 가장 가까운 점의 인덱스 찾기
        distances_from_start = vecnorm(interp_position(:,1:3) - start_position(:,1:3), 2, 2);
        [~, start_index] = min(distances_from_start);
        end_index = 1;

        % 입력 인덱스의 유효성 검사
        if start_index < 1 || end_index > size(interp_position, 1) || start_index <= end_index
            error('유효하지 않은 인덱스 범위입니다. start_index는 end_index보다 커야 합니다.');
        end
        
        % (2) 최종 곡선 구간의 위치/자세 경로(Nx6) 추출
        curve_pos_traj = [interp_position(start_index:-1:end_index,:), interp_normal(start_index:-1:end_index,:)];
        curve_pos_traj(:,2) = curve_pos_traj(:,2) + ones(length(curve_pos_traj),1)*offset;
       
        % (3) 경로 포인트 개수 확인
        num_path_points = size(curve_pos_traj, 1);
        if num_path_points < 3 % 가감속 프로파일을 만들기에 점이 너무 적으면 경고
            warning('경로 점 개수가 너무 적어 부드러운 속도 프로파일을 생성할 수 없습니다.');
            curve_vel_traj = zeros(num_path_points, 3); % 속도는 0으로 반환
            return;
        end

        %% 속도 프로파일 생성
        robot_speed = max(omega(start_index:-1:end_index));      
        speedProfile = zeros(num_path_points, 1);

        % 가감속 구간을 전체 경로점의 15%씩 할당 (S-커브는 약간 더 여유있게)
        num_accel_points = round(num_path_points * 0.15);
        num_decel_points = round(num_path_points * 0.15);
        
        % [수정] 가감속 포인트가 너무 적을 경우를 대비한 최소값 설정
        if num_accel_points < 2, num_accel_points = 2; end
        if num_decel_points < 2, num_decel_points = 2; end
        
        num_const_points = num_path_points - num_accel_points - num_decel_points;
    
        %   (A) 가속 구간 생성 (S-커브)
        % [수정] linspace 대신 quinticpolytraj 사용
        % '시간' 대신 '지점 인덱스'를 기준으로 폴리노미얼을 생성합니다.
        t_poly_accel = 1:num_accel_points;
        accel_waypoints = [0, robot_speed];
        time_waypoints_accel = [0, num_accel_points];
        accel_shape = quinticpolytraj(accel_waypoints, time_waypoints_accel, t_poly_accel);
        speedProfile(1:num_accel_points) = accel_shape;
        
        %   (B) 등속 구간 생성
        start_const_idx = num_accel_points + 1;
        end_const_idx = start_const_idx + num_const_points - 1;
        speedProfile(start_const_idx:end_const_idx) = robot_speed;
        
        %   (C) 감속 구간 생성 (S-커브)
        % [수정] linspace 대신 quinticpolytraj 사용
        start_decel_idx = end_const_idx + 1;
        t_poly_decel = 1:num_decel_points;
        decel_waypoints = [robot_speed, 0];
        time_waypoints_decel = [0, num_decel_points];
        decel_shape = quinticpolytraj(decel_waypoints, time_waypoints_decel, t_poly_decel);
        speedProfile(start_decel_idx:end) = decel_shape;
        
        curve_vel_traj = speedProfile .* unit_direction;

    end
    
end