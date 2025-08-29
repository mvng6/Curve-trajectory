function indices = RDP_recursive(points, epsilon, start_idx, end_idx)
    d_max = 0;
    index = 0;
    
    % 시작점과 끝점 사이의 직선에서 가장 먼 점 찾기
    for i = start_idx+1 : end_idx-1
        d = point_to_line_dist(points(i,:), points(start_idx,:), points(end_idx,:));
        if d > d_max
            d_max = d;
            index = i;
        end
    end
    
    % 최대 거리가 epsilon보다 크면, 해당 점을 기준으로 재귀 호출
    if d_max > epsilon
        rec_indices1 = RDP_recursive(points, epsilon, start_idx, index);
        rec_indices2 = RDP_recursive(points, epsilon, index, end_idx);
        indices = [rec_indices1(1:end-1); rec_indices2];
    else
        % 작으면, 시작점과 끝점만 유지
        indices = [start_idx; end_idx];
    end
end