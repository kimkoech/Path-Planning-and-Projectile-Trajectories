%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: 3DOF Arm inverse Kinematics


%}

% Given a 3x1 vector "p" representing the end-effector position (x, y) and
% orientation (theta) (measured counterclockwise from horizontal), return
% current configuration "q".
function q = inverseK3DOF(p)
    % TODO: solve the inverse kinematics
  
    
    [L1, L2, L3] = getLinkLengths();
    % end effector position and orientation
    x = p(1);
    y = p(2);
    phi = p(3);
    
    wx = x - L3 * cos(phi);
    wy = y - L3 * sin(phi);
    
    D = (wx^2 + wy^2 - L1^2 - L2^2)/(2 * L1 * L2);
    
    % find q2
    q2 = atan2(real(sqrt(1-D^2)), D);
  
    % find q1
    q1 = atan2(wy,wx) - atan2(L2*sin(q2),L1+L2*cos(q2));

    % find q3
    q3 = phi - q1 - q2;

    % return one solution to IK
    q = [
        q1;
        q2;
        q3
        ];
    
    
end