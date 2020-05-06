%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Select the appropriate trajectory

%}

function [traj,x0,y0,V0x,V0y,max_range] = findTrajectory(V_workspace, projectiles,x_target, y_target)

% Check the list of projectiles
min = norm(projectiles{1}(:,1) - [x_target, y_target]);
index = 1;
[Nr,Nc] = size(projectiles{1});
max_range = projectiles{1}(1,1)
for i=1:length(projectiles)
    
    % find target traj
    diff = norm(projectiles{i}(:,Nc) - [x_target, y_target]);
    if diff < min
        min = diff;
        index = i;
    end
    
    %find max range
    range = projectiles{i}(1,end);
    if range<max_range
        max_range = range;
    end
    
    
end

% extract the desired the projectile trajectory
traj = projectiles{index};
x0 = traj(1,1);
y0 = traj(2,1);
V0x = V_workspace(1,index);
V0y = V_workspace(2,index);
disp('max range')
disp(max_range)


end

