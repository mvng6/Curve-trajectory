function [P, N, T, kappa] = fun_define_path_curve(filename)
    % 특정 금형 데이터 파일(txt or csv)을 불러와 위치(P), 법선(N),
    % 접선(T), 곡률(kappa)을 모두 계산하여 반환하는 함수.
    %
    % 입력
    %       filename    : 경로를 생성하는데 사용할 데이터 명
    %
    % 출력
    %       P           : 위치 좌표
    %       N           : 법선 벡터
    %       T           : 접선 벡터
    %       kappa       : 곡률

    % --- 1. 데이터 파일 로드 ---
    try
        raw_data = readmatrix(filename);
    catch ME
        error('파일을 읽는 데 실패했습니다: %s. 파일명과 경로를 확인해주세요.\n%s', filename, ME.message);
    end

    if size(raw_data, 2) ~= 6
        error('입력 데이터는 N x 6 형태여야 합니다 (x,y,z,nx,ny,nz).');
    end
    
    P = raw_data(:, 1:3);
    N = raw_data(:, 4:6);
    n_points = size(P, 1);
    
    fprintf('파일 "%s"에서 총 %d개의 경로 지점을 성공적으로 불러왔습니다.\n', filename, n_points);

    % --- 2. 경로 접선(Tangent) 벡터 계산 ---
    T = zeros(n_points, 3);
    for i = 2:n_points-1
        tangent_vec = P(i+1, :) - P(i-1, :);
        T(i, :) = tangent_vec / norm(tangent_vec);
    end
    T(1, :) = (P(2, :) - P(1, :)) / norm(P(2, :) - P(1, :));
    T(n_points, :) = (P(n_points, :) - P(n_points-1, :)) / norm(P(n_points, :) - P(n_points-1, :));
    
    % --- 3. 경로 곡률(Curvature) 계산 ---
    kappa = zeros(n_points, 1);
    for i = 2:n_points-1
        v_i = P(i+1, :) - P(i-1, :);
        a_i = P(i+1, :) - 2*P(i, :) + P(i-1, :);
        if norm(v_i) > 1e-6
            kappa(i) = norm(cross(v_i, a_i)) / (norm(v_i)^3);
        else
            kappa(i) = 0;
        end
    end
end