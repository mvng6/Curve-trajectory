function points = hermite_spline(P0, P1, T0, T1, num_points)
    t = linspace(0, 1, num_points);
    
    % 에르미트 기저 함수
    h00 = 2*t.^3 - 3*t.^2 + 1;
    h10 = t.^3 - 2*t.^2 + t;
    h01 = -2*t.^3 + 3*t.^2;
    h11 = t.^3 - t.^2;
    
    % 접선 벡터의 크기(속도) 조절
    scale = norm(P1 - P0);
    T0_scaled = scale * T0;
    T1_scaled = scale * T1;
    
    % =============================================================
    % %%% [최종 수정] X, Y, Z 좌표를 각각 계산하여 차원 오류 원천 방지 %%%
    % =============================================================
    % 각 좌표별로 스플라인 계산
    points_x = h00 .* P0(1) + h10 .* T0_scaled(1) + h01 .* P1(1) + h11 .* T1_scaled(1);
    points_y = h00 .* P0(2) + h10 .* T0_scaled(2) + h01 .* P1(2) + h11 .* T1_scaled(2);
    points_z = h00 .* P0(3) + h10 .* T0_scaled(3) + h01 .* P1(3) + h11 .* T1_scaled(3);
    
    % 최종 결과(Nx3)로 합치기
    points = [points_x', points_y', points_z'];
end