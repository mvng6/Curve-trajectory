function [P, T, N] = fun_define_path_yaxis(C, R, t, V)
    % Y축 기준 경로를 정의하는 함수
    % 입력: C(중심), R(반지름), t(시간 벡터), V(속도)
    % 출력: P(위치), T(접선), N(법선)
    
    theta = (pi/2) - (V/R) * t; % 시간에 따른 각도 계산

    % 1. 위치 벡터 (P)
    x = C(1) * ones(size(theta)); % X는 중심으로 고정
    y = C(2) - R * cos(theta);
    z = R * sin(theta) + C(3);
    P = [x; y; z];

    % 2. 접선 벡터 (T, Tool의 X축)
    Tx = zeros(size(theta));
    Ty = R * sin(theta);
    Tz = R * cos(theta);
    T_vec = [Tx; Ty; Tz];
    T = T_vec ./ vecnorm(T_vec); % 단위 벡터로 정규화

    % 3. 법선 벡터 (N, Tool의 Z축)
    Nx = zeros(size(theta));
    Ny = -R * cos(theta);
    Nz = R * sin(theta);
    N_vec = [Nx; Ny; Nz];
    N = -N_vec ./ vecnorm(N_vec); % 단위 벡터로 정규화
end