clear;
close all;
clc;

Initialization;
Figure_setup;

%%
data = readmatrix('faces_vectors_x.csv');

pre_C = data(:,1:3);
pre_N = data(:,4:6);

%%
C = [pre_C(:,2),pre_C(:,1),pre_C(:,3)];
N = [pre_N(:,2),pre_N(:,1),pre_N(:,3)];

%%
figure;
scatter3(C(:,1), C(:,2), C(:,3), 36, 'filled');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Curve Plate Trajectory');
grid on; axis equal;

hold on;
quiver3(C(:,1), C(:,2), C(:,3), N(:,1), N(:,2), N(:,3), 0.5);
legend('Center Points','Normals');
hold off;

