%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Projectile kinematics


%}

% Function that takes end effector position and end effector velocity
% to compute a trajectory

% Given values
%target = [0;0.1];
%Range = target(1);
%y_target = target(2);

function projectileTraj = getProjectileTrajectory(V0x,V0y,x0,y0)

g = -9.8; % gravitational accel in m/s^2 

% Rember to redesign for position of target ie y0-y_target
% calculate the time of flight
t1 = (-V0y + sqrt(V0y^2 - (4*0.5*g*y0))) / (2*0.5*g);
t2 = (-V0y - sqrt(V0y^2 - (4*0.5*g*y0))) / (2*0.5*g);

% use the positive time
t = max(t1,t2);

% make array from t0 to tmax
t_array = linspace(0,t,40);

% Kinematic equations

% X direction
% Velocity V(t) = V_0x + at ==> V_x [Constant and Given]
% Displacement S = 0.5at^2 + V_0xt + S_o ==> x = V0x * t + x0
x = (V0x .* t_array) - x0;

% Y direction
% Velocity Vy(t) = V0y + gt
% Displacement y(t) = 0.5gt^2 + V0yt + y0
y = 0.5*g*(t_array.^2) + V0y.*t_array + y0;

projectileTraj = [-x;y];

end
