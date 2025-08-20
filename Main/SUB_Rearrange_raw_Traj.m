% 파이썬 코드(extract_z_vector)를 통해 STL로부터 추출한 초기 금형 형상
% 데이터(indexed_path_data_Y_axis)를 재정렬 및 전처리하는 코드

clear;
close all;
clc;

Initialization;

%% 경로 데이터 불러오기
pre_traj = readmatrix('indexed_path_data_Y_axis.csv');

%% y축을 기준으로 오름차순 정렬
[y_sort, idx_sort] = sort(pre_traj(:,3));
traj = pre_traj(idx_sort,:);

%% 중복되는 좌표 제거
% y축 위치 좌표를 이용해서 제거
new_traj = [];
for i = 1:length(traj(:,3))
    if mod(i,2) == 0
        new_traj = [new_traj; traj(i,2:7)];
    end
end

%% 시각화를 통한 확인
figure;
plot(new_traj(:,3))

%% 데이터 저장
% 1. fullfile 함수를 이용해 전체 파일 경로를 안전하게 생성합니다.
full_path = fullfile(data_cd, 'test_curve_traj.csv');

% 2. writematrix 함수로 행렬 데이터를 지정된 경로에 CSV 파일로 저장합니다.
writematrix(new_traj, full_path);

disp('test_curve_traj.csv 파일 저장이 완료되었습니다.');