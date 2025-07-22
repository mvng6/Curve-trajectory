function R = fun_rotZYmZ(phi, theta, psi)
    % Z → Y → -Z: 마지막 각도에 -부호만 바꿈
    R = fun_rotZYZ(phi, theta, -psi);
end
