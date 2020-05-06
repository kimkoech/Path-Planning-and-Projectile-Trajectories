%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Get origins of each link


%}


%Given the current configuration "q", return a 3x3 matrix of current frame
% origins: [o1, o2, o3] (each column is an origin).
function origins = getCurrentOrigins(q)
    [L1, L2, L3] = getLinkLengths();
    
    %homogenous matrices
    H_0_1 = DH(L1,0,0,q(1));
    H_1_2 = DH(L2,0,0,q(2));
    H_2_3 = DH(L3,0,0,q(3));
    
    H_0_2 = H_0_1 * H_1_2;
    H_0_3 = H_0_2 * H_2_3;
    
    % get origins
    o1 = H_0_1(1:3,4);
    o2 = H_0_2(1:3,4);
    o3 = H_0_3(1:3,4);
    
    origins = [o1, o2, o3];
end
