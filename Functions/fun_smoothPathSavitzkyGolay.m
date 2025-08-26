function smoothed_data = fun_smoothPathSavitzkyGolay(original_data, order, frame_len)
%fun_smoothPathSavitzkyGolay 3D 경로 데이터를 Savitzky-Golay 필터로 스무딩합니다.
%   - 버전 2.0: 법선 벡터(Normal Vector) 스무딩 및 정규화 기능 추가
%
%   INPUTS:
%   original_data: 스무딩할 원본 경로 데이터 (N x 6 행렬).
%   order:         필터에 사용할 다항식의 차수 (정수, 예: 2, 3).
%   frame_len:     필터 윈도우의 크기 (홀수 정수, 예: 11, 21, 31).
%
%   OUTPUTS:
%   smoothed_data: 위치와 법선 벡터가 모두 스무딩된 N x 6 행렬.

    %% 1. 입력 유효성 검사
    if mod(frame_len, 2) == 0 || frame_len <= order
        error('오류: 윈도우 크기(frame_len)는 반드시 홀수이면서 다항식 차수(order)보다 커야 합니다.');
    end
    if size(original_data, 2) ~= 6
        error('오류: 입력 데이터는 N x 6 행렬(X,Y,Z, Nx,Ny,Nz)이어야 합니다.');
    end
    
    %% 2. 데이터 분리
    P_original = original_data(:, 1:3); % 원본 위치 데이터
    N_original = original_data(:, 4:6); % 원본 법선 벡터
    
    %% 3. 필터 적용
    % 3-1. 위치 데이터(P) 스무딩
    P_smoothed = zeros(size(P_original));
    P_smoothed(:,1) = sgolayfilt(P_original(:,1), order, frame_len);
    P_smoothed(:,2) = sgolayfilt(P_original(:,2), order, frame_len);
    P_smoothed(:,3) = sgolayfilt(P_original(:,3), order, frame_len);
    
    % 3-2. 법선 벡터(N) 데이터 스무딩
    N_smoothed = zeros(size(N_original));
    N_smoothed(:,1) = sgolayfilt(N_original(:,1), order, frame_len);
    N_smoothed(:,2) = sgolayfilt(N_original(:,2), order, frame_len);
    N_smoothed(:,3) = sgolayfilt(N_original(:,3), order, frame_len);
    
    %% 4. 스무딩된 법선 벡터 정규화 (매우 중요)
    % 스무딩 후에는 단위 벡터가 아니므로, 다시 크기를 1로 만들어줍니다.
    N_smoothed_normalized = N_smoothed ./ vecnorm(N_smoothed, 2, 2);
    
    %% 5. 결과 데이터 조합
    smoothed_data = [P_smoothed, N_smoothed_normalized];

end