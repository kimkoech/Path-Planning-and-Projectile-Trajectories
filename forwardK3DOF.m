%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: 3DOF Arm forward kinematics


%}

% Given the current configuration "q", return position of end effector 
function p = forwardK3DOF(q)
    [L1, L2, L3] = getLinkLengths();
    
    %Find homogenous matrices
    H_0_1 = DH(L1,0,0,q(1));
    H_1_2 = DH(L2,0,0,q(2));
    H_2_3 = DH(L3,0,0,q(3));
    
    T = H_0_1 * H_1_2 * H_2_3;  
    
    
    p = T(1:3,4);

end