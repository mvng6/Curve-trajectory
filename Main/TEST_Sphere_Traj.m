clear; clc; close all;

Initialization;

%% ========================================================================
%  1. 경로 타입 선택 및 파라미터 설정
% =========================================================================

path_type = 'mold_from_file'; % 'hemi_xaxis', 'hemi_yaxis', 'mold_from_file' 중 선택

V = 50.0;
filename = 'my_mold_data.csv'; % <-- 파일 경로일 경우, 파일명을 입력

R = 200.0; C = [0, 370, 0]; dt = 0.01; % 수학적 경로용 파라미터

%% ========================================================================
%  2. 경로 데이터 생성 (선택된 타입에 따라 분기)
% =========================================================================
fprintf('선택된 경로 타입: %s\n', path_type);

if strcmp(path_type, 'hemi_xaxis')
    total_time = (pi * R) / V;
    t = 0:dt:total_time;
    [P, N_surface] = define_path_xaxis(C, R, t, V);
    % (접선, 곡률은 아래에서 별도 계산)

elseif strcmp(path_type, 'hemi_yaxis')
    total_time = (pi * R) / V;
    t = 0:dt:total_time;
    [P, N_surface] = define_path_yaxis(C, R, t, V);
    % (접선, 곡률은 아래에서 별도 계산)
    
elseif strcmp(path_type, 'mold_from_file')
    % ✅ 변경점: 새로운 함수를 호출하여 P, N, T, kappa를 한 번에 받아옵니다.
    [P, N_surface, T_path, kappa] = fun_define_path_curve(filename);
    
else
    error('잘못된 경로 타입입니다.');
end

% 'mold_from_file'이 아닐 경우, T_path와 kappa를 계산해야 함
if ~strcmp(path_type, 'mold_from_file')
    n_points = size(P, 2);
    P_t = P'; % 계산을 위해 n x 3 형태로 변환
    T_path = zeros(n_points, 3);
    for i = 2:n_points-1
        T_path(i, :) = (P_t(i+1, :) - P_t(i-1, :)) / norm(P_t(i+1, :) - P_t(i-1, :));
    end
    T_path(1, :) = (P_t(2, :) - P_t(1, :)) / norm(P_t(2, :) - P_t(1, :));
    T_path(n_points, :) = (P_t(n_points, :) - P_t(n_points-1, :)) / norm(P_t(n_points, :) - P_t(n_points-1, :));
    
    kappa = zeros(n_points, 1);
    for i = 2:n_points-1
        v_i = P_t(i+1, :) - P_t(i-1, :); a_i = P_t(i+1, :) - 2*P_t(i, :) + P_t(i-1, :);
        if norm(v_i) > 1e-6, kappa(i) = norm(cross(v_i, a_i)) / (norm(v_i)^3); end
    end
    P = P_t; % 원래 형태로 변환
    N_surface = N_surface';
else
    n_points = size(P,1);
end


%% ========================================================================
%  ✅ 삭제: 메인 스크립트에서 접선/곡률 계산 부분이 필요 없어졌습니다.
% =========================================================================


%% ========================================================================
%  4. 각 지점별 SpeedL 명령 생성 (수정 없음)
% =========================================================================
% (이전 답변과 동일)
speedl_commands = zeros(n_points, 6);
% ... (이하 계산 및 시각화 코드는 이전 답변과 동일합니다) ...