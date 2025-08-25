% fun_blend_single_corner.m (파일 전체를 교체)

function [final_path, final_normals] = fun_blend_single_corner_normals(path_points, normal_points, corner_idx, blend_dist)
    % 하나의 코너에 대해 위치(Spline)와 방향(SLERP) 블렌딩을 계산하고 적용
    
    num_spline_points = 100; % 블렌딩 구간을 나눌 점의 개수

    % --- [단계 1] 위치 블렌드 시작/끝점 찾기 (기존과 동일) ---
    dist_sum = 0;
    start_seg_idx = 0;
    for i = corner_idx:-1:2
        segment_len = norm(path_points(i,:) - path_points(i-1,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            P_spline_start = path_points(i-1,:) + ratio * (path_points(i,:) - path_points(i-1,:));
            start_seg_idx = i - 1;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if start_seg_idx == 0, error('블렌딩 거리가 코너 이전 경로 길이보다 깁니다.'); end

    dist_sum = 0;
    end_seg_idx = 0;
    for i = corner_idx:(size(path_points, 1) - 1)
        segment_len = norm(path_points(i+1,:) - path_points(i,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            P_spline_end = path_points(i,:) + ratio * (path_points(i+1,:) - path_points(i,:));
            end_seg_idx = i;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if end_seg_idx == 0, error('블렌딩 거리가 코너 이후 경로 길이보다 깁니다.'); end
    
    % --- [단계 2] 위치 스플라인 생성 (기존과 동일) ---
    T_spline_start_vec = path_points(start_seg_idx+1,:) - path_points(start_seg_idx,:);
    T_spline_start = T_spline_start_vec / norm(T_spline_start_vec);
    T_spline_end_vec = path_points(end_seg_idx+1,:) - path_points(end_seg_idx,:);
    T_spline_end = T_spline_end_vec / norm(T_spline_end_vec);
    spline_points = fun_generate_hermite_spline(P_spline_start, P_spline_end, T_spline_start, T_spline_end, num_spline_points);

    % --- [단계 3] ★★★ 방향(Orientation) 블렌딩 (SLERP) ★★★ ---
    % 3-1. 블렌딩 시작/끝 지점의 법선 벡터를 가져옴 (선형 보간으로 근사)
    N_start = normal_points(start_seg_idx,:); 
    N_end = normal_points(end_seg_idx,:);

    % 3-2. 법선 벡터를 쿼터니언으로 변환
    q_start = normalToQuaternion(N_start);
    q_end = normalToQuaternion(N_end);
    
    % 3-3. SLERP를 사용하여 두 방향 사이를 부드럽게 보간
    interp_steps = linspace(0, 1, num_spline_points);
    interp_quats = fun_slerp_manual(q_start, q_end, interp_steps);
    
    % 3-4. 보간된 쿼터니언들을 다시 법선 벡터로 변환
    spline_normals = zeros(num_spline_points, 3);
    for i = 1:num_spline_points
        % R_interp = quat2rotm(interp_quats(i));
        R_interp = quat2rotm(interp_quats(i, :));
        spline_normals(i,:) = -R_interp(:,3)'; % 툴의 Z축(-N) 방향
    end

    % --- [단계 4] 최종 경로 결합 ---
    path_before = path_points(1:start_seg_idx, :);
    normals_before = normal_points(1:start_seg_idx, :);

    path_after = path_points(end_seg_idx+1:end, :);
    normals_after = normal_points(end_seg_idx+1:end, :);
    
    final_path = [path_before; spline_points; path_after];
    final_normals = [normals_before; spline_normals; normals_after];
end

function q = normalToQuaternion(normal_vec)
    % 법선 벡터(툴의 -Z축)로부터 쿼터니언을 생성하는 헬퍼 함수
    
    z_axis_tool = -normal_vec / norm(normal_vec);
    
    % Z축만으로는 완전한 회전을 정의할 수 없으므로, X, Y축을 임의로 생성
    % 월드 X축과 평행하지 않은 벡터를 찾아 Y축을 생성
    ref_vec = [1, 0, 0];
    if abs(dot(z_axis_tool, ref_vec)) > 0.99
        ref_vec = [0, 1, 0]; % Z축이 X축과 거의 평행하면 Y축을 기준으로 삼음
    end
    
    y_axis_tool = cross(z_axis_tool, ref_vec);
    y_axis_tool = y_axis_tool / norm(y_axis_tool);
    
    x_axis_tool = cross(y_axis_tool, z_axis_tool);
    
    rotm = [x_axis_tool', y_axis_tool', z_axis_tool'];
    q = rotm2quat(rotm);
end