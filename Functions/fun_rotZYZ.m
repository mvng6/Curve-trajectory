function R = fun_rotZYZ(phi, theta, psi)
    % 입력은 라디안
    Rz1 = [cos(phi), -sin(phi), 0;
           sin(phi),  cos(phi), 0;
           0,         0,        1];
       
    Ry  = [cos(theta), 0, sin(theta);
           0,          1, 0;
          -sin(theta), 0, cos(theta)];
      
    Rz2 = [cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
           0,         0,        1];

    R = Rz1 * Ry * Rz2;
end
