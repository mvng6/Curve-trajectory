function dist = point_to_line_dist(p, v, w)
    % 점 p와, 점 v, w를 잇는 직선 사이의 거리 계산
    l2 = sum((v - w).^2);
    if l2 == 0
        dist = norm(p - v);
        return;
    end
    t = max(0, min(1, dot(p - v, w - v) / l2));
    projection = v + t * (w - v);
    dist = norm(p - projection);
end