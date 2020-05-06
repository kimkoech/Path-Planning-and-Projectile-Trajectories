
%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Homogenous matrix from Denavit–Hartenberg(DH) parameters 


%}
% Function to generate DH matrix
% takes degrees
function A = DH(a, alpha, d, theta)
    A = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1
        ];
end 