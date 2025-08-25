function corner_indices = fun_find_path_corners(path_points, detection_config)
    % 경로(path_points) 내에서 코너의 인덱스를 찾아 반환하는 함수
    
    lookahead = detection_config.lookaheadPoints;
    threshold = detection_config.angleThreshold;
    corner_indices = [];
    
    fprintf('코너 탐지 중... (Lookahead: %d, Angle Threshold: %.1f)\n', lookahead, threshold);

    for i = (1 + lookahead) : (size(path_points, 1) - lookahead)
        prev_point = path_points(i - lookahead, :);
        current_point = path_points(i, :);
        next_point = path_points(i + lookahead, :);
        
        v1 = current_point - prev_point;
        v2 = next_point - current_point;
        
        if norm(v1) > 1e-6 && norm(v2) > 1e-6
            angle = acosd(dot(v1, v2) / (norm(v1) * norm(v2)));
            
            if angle > threshold
                % 동일 코너 중복 탐지 방지
                if isempty(corner_indices) || (i - corner_indices(end)) > (lookahead * 2)
                    corner_indices = [corner_indices, i];
                    fprintf('  -> 인덱스 %d 근방에서 코너 발견 (각도: %.1f도)\n', i, angle);
                end
            end
        end
    end
    
    if isempty(corner_indices)
        warning('경로에서 코너를 찾지 못했습니다.');
    else
        fprintf('%d개의 코너를 성공적으로 탐지했습니다.\n', length(corner_indices));
    end
end