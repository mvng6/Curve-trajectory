function [interp_time_s, interp_position_spline, interp_normals_slerp, interp_dist_mm] = fun_resample_interp(traj_file_name, robot_speed_mmps, dt)
% fun_resample_interp : CSV 파일로부터 경로를 읽어 리샘플링 및 보간을 적용합니다.
%
% 입력:
%       traj_file_name:         원본 금형 경로 CSV 파일 명
%       robot_speed_mmps:        원하는 로봇 이동 속도
%       dt:                     원하는 제어 주기
%
% 출력:
%       interp_time_s:          보간된 시간 데이터 
%       interp_position_spline: 보간된 Position 데이터
%       interp_normals_slerp:   보간된 법선 벡터 데이터
%       interp_dist_mm:         보간된 경로 데이터의 누적 이동 거리

    %% 데이터 불러오기 및 전처리
    % 원본 경로 데이터 로드
    data = readmatrix(traj_file_name);

    % 데이터 분리
    pre_positions_mm = data(:,1:3);         % [X, Y, Z]
    pre_normals = data(:,4:6);              % [Nx, Ny, Nz]

    % (Options) x와 y를 교체함 
    %   - 곡면 구동 시 x축을 기준으로 이동하기 위해
    positions_mm = [pre_positions_mm(:,2), pre_positions_mm(:,1), pre_positions_mm(:,3)];
    normals = [pre_normals(:,2), pre_normals(:,1), pre_normals(:,3)];
    
    % 테스트 (법선 벡터 반대로)
    normals = -normals;

    %% 경로 데이터 리샘플링
    % (1) 경로 전체 길이 및 이동 소요 시간 계산
    delta_s = vecnorm(diff(positions_mm), 2, 2);
    cumulative_dist_mm = [0; cumsum(delta_s)];
    original_time_s = cumulative_dist_mm / robot_speed_mmps;

    % (2) 입력된 제어 주기 간격의 새로운 시간/거리 축 생성
    total_duration_s = original_time_s(end);
    interp_time_s = (0:dt:total_duration_s);
    interp_dist_mm = interp_time_s * robot_speed_mmps;
    num_new_points = length(interp_time_s);

    %% 위치 및 법선 벡터 보간
    % (1) Position 보간 (Spline)
    interp_position_spline = spline(original_time_s, positions_mm', interp_time_s)';
    
    % (2) Orientation 보간 (SLERP)
    intper_normals_slerp = zeros(num_new_points,3);
    
    % SLERP 헬퍼 함수 정의 (수학 공식 기반)
    slerp = @(v0, v1, t) ...
        (sin((1-t)*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v0 + ...
        (sin(t*acos(dot(v0,v1)))/sin(acos(dot(v0,v1))))*v1;
    
    % 1kHz 시간 스텝마다 루프를 돌며 SLERP 수행
    for i = 1:num_new_points
        t_current = interp_time_s(i);
    
        % 현재 시간이 기존 경로 데이터에서 어떤 시간 구간에 속하는지 탐색
        k = find(original_time_s <= t_current, 1, 'last');
        
        % 마지막 지점을 넘어가면 마지막 법선 벡터를 그대로 사용
        if k >= length(original_time_s)
            intper_normals_slerp(i,:) = normals(end,:);
            continue;
        end
    
        % 현재 보간 구간 내에서의 진행률(t) 계산 (0~1 사이)
        t_segment = (t_current - original_time_s(k)) / (original_time_s(k+1) - original_time_s(k));
        
        v0 = normals(k,:);
        v1 = normals(k+1,:);
        
        % 두 벡터가 거의 동일한 경우 선형 보간을 수행 (NaN을 방지하기 위함)
        if abs(dot(v0, v1)) > 0.9999
            interp_normals_slerp(i,:) = (1 - t_segment)*v0 + t_segment*v1;
        else
            interp_normals_slerp(i,:) = slerp(v0, v1, t_segment);
        end
    end
    
    % 수치 오류 보정을 위한 재정규화
    interp_normals_slerp = interp_normals_slerp ./ vecnorm(interp_normals_slerp, 2, 2);
    

end