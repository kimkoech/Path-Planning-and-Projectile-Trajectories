
%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Calculate linear velocity jacobian for each joint


%}


% Given the current configuration "q", return Jacobians corresponding to
% each link.
function jacobians = getCurrentJacobians(q)
    [L1, L2, L3] = getLinkLengths();
    
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    % Calculate values for the Jacobians J1, J2, and J3 (should be 3x3 matrices)
    
    J1 = [-L1*sin(q1), 0, 0;...
           L1*cos(q1), 0, 0;...
           1          , 0, 0];
       
    J2 = [-L1+sin(q1)-L2*sin(q1+q2), -L2*sin(q1+q2), 0;...
           L1*cos(q1)+L2*cos(q1+q2), L2*cos(q1+q2) , 0;...
           1                         , 1              , 0];
    
    J3 = [-L1+sin(q1)-L2*sin(q1+q2)-L3*sin(q1+q2+q3), -L2*sin(q1+q2)-L3*sin(q1+q2+q3), -L3*sin(q1+q2+q3);...
           L1*cos(q1)+L2*cos(q1+q2)+L3*cos(q1+q2+q3), L2*cos(q1+q2)+L3*cos(q1+q2+q3) , L3*cos(q1+q2+q3) ;...
           1                                           , 1                                , 1                 ];
    
    
    
    jacobians = {J1, J2, J3};
end