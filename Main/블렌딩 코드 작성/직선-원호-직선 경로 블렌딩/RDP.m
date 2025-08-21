function indices = RDP(points, epsilon)
    % 재귀 함수를 호출하여 인덱스를 찾음
    indices = RDP_recursive(points, epsilon, 1, size(points, 1));
end