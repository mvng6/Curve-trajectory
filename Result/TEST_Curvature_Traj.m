clear;
close all;
clc;

% Initialization;
% Figure_setup;

%%
fname = 'Data_Speedl_F_0_P_0.00_I_0.00_D_0.00.csv';
% fname = 'Data_Speedl_tcp_set_O_convex.csv';
% fname = 'Data_Speedl_tcp_set_O_concave.csv';
data = readmatrix(fname);
Target_F = sscanf(fname, 'Data_Speedl_F_%d_P_%*f_I_%*f_D_%*f.txt', 1);

t = (data(:,1) - data(1,1))*0.001;
flange_pos = data(:,2:7);
tcp_pos = data(:,8:13);
joint = data(:,14:19);
tcp_vel = data(:,20:25);
joint_vel = data(:,26:31);

%%
figure;
subplot(2,2,1);
plot3(flange_pos(:,1),flange_pos(:,2),flange_pos(:,3));
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D Flange Pos');
axis square;

% figure;
subplot(2,2,2);
plot3(tcp_pos(:,1),tcp_pos(:,2),tcp_pos(:,3));
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D TCP Pos');
axis square;

% figure;
subplot(2,2,3);
plot(flange_pos(:,1),flange_pos(:,3));
xlabel('X (mm)');
ylabel('Z (mm)');
title('2D Flange Pos');
axis square;

% figure;
subplot(2,2,4);
plot(tcp_pos(:,1),tcp_pos(:,3));
xlabel('X (mm)');
ylabel('Z (mm)');
title('2D TCP Pos');
axis square;