function [P, N] = generate_test_path()
    % 테스트를 위한 경로(위치 P, 법선 N)를 생성하는 함수
    % 경로 구성: 직선1 -> 90도 코너 -> 직선2 -> 부드러운 곡선

    % --- 각 구간의 점 개수 설정 ---
    n_points_line1 = 50;
    n_points_line2 = 50;
    n_points_arc = 50;

    % --- [구간 1] 직선 1 ---
    x1 = linspace(0, 50, n_points_line1)';
    y1 = zeros(n_points_line1, 1);
    z1 = ones(n_points_line1, 1) * 10;
    P1 = [x1, y1, z1];
    N1 = repmat([0, 0, 1], n_points_line1, 1); % 법선 벡터는 Z축 방향

    % --- [구간 2] 직선 2 (90도 코너 이후) ---
    % 중복을 피하기 위해 시작점은 제외 (2:end)
    x2 = ones(n_points_line2 - 1, 1) * 50;
    y2 = linspace(0, 50, n_points_line2)';
    y2 = y2(2:end);
    z2 = ones(n_points_line2 - 1, 1) * 10;
    P2 = [x2, y2, z2];
    N2 = repmat([0, 0, 1], n_points_line2 - 1, 1);

    % --- [구간 3] 부드러운 곡선 ---
    arc_center_y = 50;
    arc_center_z = 0;
    arc_radius = 10;
    start_angle = pi / 2;
    end_angle = 0;
    
    theta = linspace(start_angle, end_angle, n_points_arc)';
    theta = theta(2:end); % 중복 피하기
    
    x3 = ones(n_points_arc - 1, 1) * 50;
    y3 = arc_center_y + arc_radius * cos(theta);
    z3 = arc_center_z + arc_radius * sin(theta);
    P3 = [x3, y3, z3];

    % 곡선 구간의 법선 벡터 계산
    N3_x = zeros(n_points_arc - 1, 1);
    N3_y = y3 - arc_center_y;
    N3_z = z3 - arc_center_z;
    N3_unnormalized = [N3_x, N3_y, N3_z];
    N3 = N3_unnormalized ./ vecnorm(N3_unnormalized, 2, 2); % 각 행별로 정규화

    % --- 모든 구간 결합 ---
    P = [P1; P2; P3];
    N = [N1; N2; N3];

    fprintf('MATLAB 코드로 총 %d개의 경로점을 생성했습니다.\n', size(P, 1));
end