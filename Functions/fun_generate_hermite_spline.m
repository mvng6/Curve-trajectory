function points = fun_generate_hermite_spline(P0, P1, T0, T1, num_points)
    % 3차 에르미트 기저 함수를 이용하여 스플라인 곡선을 생성하는 함수
    
    t = linspace(0, 1, num_points)';
    h00 = 2*t.^3 - 3*t.^2 + 1;
    h10 = t.^3 - 2*t.^2 + t;
    h01 = -2*t.^3 + 3*t.^2;
    h11 = t.^3 - t.^2;
    
    scale = norm(P1 - P0); 
    T0_scaled = scale * T0; 
    T1_scaled = scale * T1;
    
    points_x = h00*P0(1) + h10*T0_scaled(1) + h01*P1(1) + h11*T1_scaled(1);
    points_y = h00*P0(2) + h10*T0_scaled(2) + h01*P1(2) + h11*T1_scaled(2);
    points_z = h00*P0(3) + h10*T0_scaled(3) + h01*P1(3) + h11*T1_scaled(3);
    
    points = [points_x, points_y, points_z];
end