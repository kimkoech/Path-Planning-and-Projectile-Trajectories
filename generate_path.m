%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: path generator for projectile launcher


%}


clear all; close all; clc;


% no obastacles in the path
obs = [];

path_option = '2';
% solve for the trajectory
% starting and ending position and orientation
if path_option == '1'
    ps = [0.3;0;0];
    pf = [-0.3;0;pi];
    xi = [1000, 1, 1];
    dir = [1 0 0; 0 1 0; 0 0 1];
    Q = path3link(ps, pf, obs,xi);
    %target position
    target = [-0.7,0];
elseif path_option == '2'
    ps = [0.3;0;0];
    pf = [-0.1;0;pi];
    xi = [1, 100, 100];
    dir = [-1 0 0; 0 1 0; 0 0 1];
    Q = path3link(ps, pf, obs,xi);
    %target position
    target = [-0.8,0];
else
    steps = 200;
    dir = [-1 0 0; 0 1 0; 0 0 1];
    Q = [linspace(0,3.142,steps);
        linspace(0,3.142,steps);
        linspace(0,3.142,steps)];
    ps = forwardK3DOF(Q(:,1));
    pf = forwardK3DOF(Q(:,end));
    %target position
    target = [-0.25,0];
end

    

% config space velocity given that servos run at 0.18sec/60 degree
V_servo = [0.18/60;0.18/60;0.18/60]; %sec/degree
V_config = deg2rad(1./V_servo); % convert from sec/deg to rad/s (SI)

% GRADIENT DESCENT PATH PLANNING
% for each configuration calculate the jacobian and the workpace velovity
% Also find the corresponding end effector position and orientation
[r,c] = size(Q);
V_workspace = zeros(3,c);
P = zeros(3,c);
for i=1:c
    % WORKSPACE V = Jacobian x V_config
    Jv = getCurrentJacobians(Q(:,i));
    Jv_EE = Jv{3};
    % assuming servos run at consant velocity defined by V_config
    V_workspace(:,i) = dir * (Jv_EE * V_config);
    
    % get end effector position
    origins = getCurrentOrigins(Q(:,i));
    P(:,i) = origins(:,3);
end

% plot trajectories
figure()
hold on;
projectiles = {};
for i = 1:c
    V0x = V_workspace(1,i);
    V0y = V_workspace(2,i);
    x0 = P(1,i);
    y0 = P(2,i);
    projectile = getProjectileTrajectory(V0x,V0y,x0,y0);
    projectiles{i} = projectile;
    plot(projectile(1,:), projectile(2,:));
end

% plot target trajectory
x_target = target(1);
y_target = target(2);
[desiredTraj,desired_x0,desired_y0,desired_V0x,desired_V0y, max_range] = ...
    findTrajectory(V_workspace,projectiles,x_target,y_target);


%plot start
plot(desired_x0, desired_y0,'b*', 'MarkerSize', 20);
%plot trajectory
plot(desiredTraj(1,:), desiredTraj(2,:), 'LineWidth', 1.5);
%plot target
plot(x_target, y_target,'r*', 'MarkerSize', 20);
hold off;

% plot target trajectory separately
figure()
hold on;
plot(desired_x0, desired_y0,'b*', 'MarkerSize', 20); %plot start
plot(desiredTraj(1,:), desiredTraj(2,:), 'LineWidth', 1.5); %plot trajectory
plot(x_target, y_target,'r*', 'MarkerSize', 20); %plot target
hold off;


% visualize the path
visualizePath(Q, ps, pf, obs, projectiles);

% visualize the path
visualizePath(Q, ps, pf, obs, {});

% visualize the path
visualizePath(Q, ps, pf, obs, {desiredTraj});
disp(max_range)
