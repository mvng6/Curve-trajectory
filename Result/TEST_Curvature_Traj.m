clear;
close all;
clc;

% Initialization;
Figure_setup;

%%
% fname = 'Data_Speedl_Curve_Plate_test.csv';
% fname = 'Data_Speedl_tcp_set_O_convex.csv';
fname = 'Data_Speedl_tcp_set_O_concave_test.csv';
data = readmatrix(fname);
Target_F = sscanf(fname, 'Data_Speedl_F_%d_P_%*f_I_%*f_D_%*f.txt', 1);

t = (data(:,1) - data(1,1))*0.001;
flange_pos = data(:,2:7);
tcp_pos = data(:,8:13);
joint = data(:,14:19);
des_vel = data(:,20:25);
tcp_vel = data(:,26:31);
joint_vel = data(:,32:37);

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

%%
figure;
for i = 1:6
    subplot(2,3,i);
    plot(tcp_vel(:,i));
    hold on;
    plot(des_vel(:,i));
    hold off;
    % if (i == 3)
    %     legend('Actual TCP Vel','Desired Vel','Location','northeastoutside');
    % end
    if i == 1
        ylabel('vx (mm/s)');
    elseif i == 2
        ylabel('vy (mm/s)');
    elseif i == 3
        ylabel('vz (mm/s)');
    elseif i == 4
        ylabel('wx (deg/s)');
    elseif i == 5
        ylabel('wy (deg/s)');
    elseif i == 6
        ylabel('wz (deg/s)');
    end
end