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
fileName = 'sorted_faces_by_y.txt';
fileID = fopen(fileName, 'r');

% 파일 열기 실패 시 오류 메시지 출력
if fileID == -1
    error('파일을 열 수 없습니다: %s', fileName);
end

% 2. textscan을 위한 포맷 지정
%   %*s : 문자열을 읽지만 무시
%   %*d : 정수를 읽지만 무시
%   %f  : 부동소수점 숫자를 읽어서 저장
formatSpec = '%*s %*s %*s %*d %*s %*s <Vector (%f, %f, %f)>, %*s %*s <Vector (%f, %f, %f)>';

% 3. textscan으로 파일 전체 데이터 읽기
dataArray = textscan(fileID, formatSpec);

% 4. 파일 닫기
fclose(fileID);

% 5. 읽어온 데이터를 n x 6 행렬로 결합
%   dataArray는 1x6 셀 배열이며, 각 셀에는 n x 1 벡터가 들어있음
pre_resultTraj = [dataArray{1} dataArray{2} dataArray{3} dataArray{4} dataArray{5} dataArray{6}];

%% 2. 경로 진행 방향 수정
%   - 현재 y축을 따라 구동하는 형태
%   - x축을 따라 구동하는 형태로 경로 수정

resultTraj = [pre_resultTraj(:,2), pre_resultTraj(:,1), pre_resultTraj(:,3), ...
              pre_resultTraj(:,5), pre_resultTraj(:,4), pre_resultTraj(:,6)];

%% 3. CSV 파일로 저장
% 1. CSV 파일의 각 열에 사용할 헤더(Header)를 정의합니다.
header = {'X', 'Y', 'Z', 'Nx', 'Ny', 'Nz'};

% 2. 숫자 행렬(Matrix)을 테이블(Table) 데이터 형식으로 변환합니다.
%    이때 'VariableNames' 옵션으로 헤더를 지정해줍니다.
dataTable = array2table(resultTraj, 'VariableNames', header);

% 3. 생성된 테이블을 CSV 파일로 저장합니다.
fileName = 'transformed_data.csv';
writetable(dataTable, fileName);

% 4. 작업 완료 확인 메시지를 출력합니다.
fprintf('"%s" 파일이 현재 MATLAB 작업 폴더에 성공적으로 저장되었습니다.\n', fileName);
