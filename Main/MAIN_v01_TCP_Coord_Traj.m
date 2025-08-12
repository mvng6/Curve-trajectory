% =========================================================================
% 곡면에 대한 로봇 폴리싱을 위한 경로 계획
%   - 툴 좌표계 기준 로봇 구동을 진행 (각속도에 대한 부분만 변경하면 곡면 추종 가능)
%   - 곡면에 대한 z방향 법선 벡터를 기준으로 각속도 경로 데이터를 추출
%
% =========================================================================
% 초기 곡면 금형의 Raw 데이터(위치 좌표 & 법선벡터)를 변환 및 전처리하여 csv 파일로 저장
%
% 최종 수정: 2025-08-12
% 작성자: LDJ
% =========================================================================

%% Initialization
clear; close all; clc;

opengl hardware;

Initialization;
Figure_setup;

%% 1. 곡면 초기 경로 파일 불러오기
% 1. 파일 열기
dataTable = readtable('indexed_path_data_Y_axis.csv');
sortedTable = sortrows(dataTable, 'Y');

%% 2. 경로 진행 방향 수정
pre_data = table2array(sortedTable);

data = [pre_data(:,2),pre_data(:,3),pre_data(:,4),pre_data(:,5),pre_data(:,6),pre_data(:,7)];
pre_resultTraj = [];
for i = 1:length(data)
    if (data(i,3) ~= 0)
        pre_resultTraj = [pre_resultTraj; data(i,:)];
    end    
end

%% 3. 벡터가 겹치는 부분 제거
resultTraj = [];
for i = 1:length(pre_resultTraj)
    if (mod(i,2) == 0)
        resultTraj = [resultTraj; pre_resultTraj(i,:)];
    end    
end

%% 3. CSV 파일로 저장
% 1. 파일 이름 지정
fileName = 'transformed_data.csv';

% --- 핵심 로직 ---
% data_cd 변수가 존재하는지 확인합니다.
if ~exist('data_cd', 'var')
    error('오류: "data_cd" 변수가 작업 공간에 정의되어 있지 않습니다. 경로를 먼저 지정해주세요.');
end

% 2. data_cd에 지정된 폴더가 실제로 존재하는지 확인하고, 없으면 생성
if ~exist(data_cd, 'dir')
   mkdir(data_cd);
   fprintf('지정된 경로에 폴더가 없어 새로 생성했습니다:\n%s\n', data_cd);
end

% 3. fullfile 함수로 전체 파일 경로 생성
%    폴더 경로를 담고 있는 'data_cd' 변수와 파일 이름을 합쳐줍니다.
fullPath = fullfile(data_cd, fileName);


% --- 데이터 준비 ---
header = {'X', 'Y', 'Z', 'Nx', 'Ny', 'Nz'};
dataTable = array2table(resultTraj, 'VariableNames', header);

% 4. 생성된 전체 경로를 사용하여 테이블을 CSV 파일로 저장
writetable(dataTable, fullPath);

% 5. 작업 완료 확인 메시지 출력
fprintf('파일이 다음 경로에 성공적으로 저장되었습니다:\n%s\n', fullPath);