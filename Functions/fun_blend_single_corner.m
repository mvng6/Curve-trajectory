function final_path = fun_blend_single_corner(path_points, corner_idx, blend_dist)
    % 하나의 코너(corner_idx)에 대해 블렌딩을 계산하고 적용하는 함수
    
    % 블렌드 시작점 찾기 (코너에서 역방향)
    dist_sum = 0;
    start_seg_idx = 0;
    for i = corner_idx:-1:2
        segment_len = norm(path_points(i,:) - path_points(i-1,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            p_start = path_points(i-1,:) + ratio * (path_points(i,:) - path_points(i-1,:));
            start_seg_idx = i - 1;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if start_seg_idx == 0, error('블렌딩 거리가 코너 이전 경로 길이보다 깁니다.'); end

    % 블렌드 끝점 찾기 (코너에서 순방향)
    dist_sum = 0;
    end_seg_idx = 0;
    for i = corner_idx:(size(path_points, 1) - 1)
        segment_len = norm(path_points(i+1,:) - path_points(i,:));
        if (dist_sum + segment_len) >= blend_dist
            ratio = (blend_dist - dist_sum) / segment_len;
            p_end = path_points(i,:) + ratio * (path_points(i+1,:) - path_points(i,:));
            end_seg_idx = i;
            break;
        end
        dist_sum = dist_sum + segment_len;
    end
    if end_seg_idx == 0, error('블렌딩 거리가 코너 이후 경로 길이보다 깁니다.'); end
    
    % 경계 조건 (접선 벡터) 계산
    t_start_vec = path_points(start_seg_idx+1,:) - path_points(start_seg_idx,:);
    t_start = t_start_vec / norm(t_start_vec);
    
    t_end_vec = path_points(end_seg_idx+1,:) - path_points(end_seg_idx,:);
    t_end = t_end_vec / norm(t_end_vec);

    % 3차 에르미트 스플라인 생성
    spline_points = fun_generate_hermite_spline(p_start, p_end, t_start, t_end, 100);

    % 최종 경로 결합
    path_before = path_points(1:start_seg_idx, :);
    path_after = path_points(end_seg_idx+1:end, :);
    final_path = [path_before; spline_points; path_after];
end