%% Initialization
clear; close all; clc;

opengl hardware;

Initialization;
Figure_setup;

%% 경로 데이터 로드
dataTable = readtable('indexed_path_data_Y_axis.csv');
sortedTable = sortrows(dataTable, 'Y');

%% 데이터 전처리
pre_data = table2array(sortedTable);

data = [pre_data(:,2),pre_data(:,3),pre_data(:,4),pre_data(:,5),pre_data(:,6),pre_data(:,7)];
pre_resultTraj = [];
for i = 1:length(data)
    if (data(i,3) ~= 0)
        pre_resultTraj = [pre_resultTraj; data(i,:)];
    end    
end

%%
resultTraj = [];
for i = 1:length(pre_resultTraj)
    if (mod(i,2) == 0)
        resultTraj = [resultTraj; pre_resultTraj(i,:)];
    end    
end


%% 각도 차이 계산 (내적)
dot_product = sum(resultTraj(1:end-1,5) .* resultTraj(2:end,6), 2);

%%
figure;
plot(dot_product)

%% 3차원 그래프 시각화
figure;
hold on;
plot3(resultTraj(:,1),resultTraj(:,2),resultTraj(:,3),'k','LineWidth',3);
quiver3(resultTraj(:,1), resultTraj(:,2), resultTraj(:,3), ...
        resultTraj(:,4), resultTraj(:,5), resultTraj(:,6), ...
        'AutoScale', 'on', 'AutoScaleFactor', 0.5, ... 
        'MaxHeadSize', 0.08, ... % << 이 값을 기본값(0.2)보다 훨씬 작게 조절하여 헤드를 날렵하게 만듭니다.
        'LineWidth', 1.5, ...                          
        'Color', [0.8500 0.3250 0.0980]);               

title('Normal Vectors with Sharpened Arrow Heads');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
% axis equal; 
% xlim([178, 182])

grid on;


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
