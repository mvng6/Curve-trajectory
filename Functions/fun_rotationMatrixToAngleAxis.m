% rotationMatrixToAngleAxis.m

function [angle, axis] = fun_rotationMatrixToAngleAxis(R)
    % 회전 행렬(Rotation Matrix)을 축-각도(Axis-Angle) 표현으로 변환하는 함수
    
    angle = acos((trace(R) - 1) / 2);
    
    % angle이 0에 가까운 경우 (회전이 거의 없는 경우)
    if abs(angle) < 1e-6
        axis = [0 0 0];
        angle = 0;
        return;
    end
    
    % 회전 축 계산
    axis_x = R(3,2) - R(2,3);
    axis_y = R(1,3) - R(3,1);
    axis_z = R(2,1) - R(1,2);
    axis_unnormalized = [axis_x, axis_y, axis_z];
    axis = axis_unnormalized / (2 * sin(angle));
end