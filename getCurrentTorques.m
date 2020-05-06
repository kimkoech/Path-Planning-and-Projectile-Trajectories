%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Calculate current torques for each joint


%}


% Given the current configuration q, the final configuration qf, and a
% [3xM] matrix of obstacles (where each column gives (x, y, radius), return
% a 3x1 vector representing the jointspace torques on the manipulator.
function torques = getCurrentTorques(q, qf, obs,xi)

    % get current attractive forces
    Fatt = getCurrentAttractiveForces(q, qf,xi); % 3 x 3

    % get current repulsive forces
    Frep = getCurrentRepulsiveForces(q, obs); % 3 x 3
    
    % get current Jacobians
    jacobians = getCurrentJacobians(q); % {[3x3] [3x3] [3x3]}
    
    % TODO: set the cumulative jointspace "torques" vector
    torques = zeros(3, 1);
    
    for i=1:length(torques)
        % sum of products [3x3]' * [3x1] + [3x3]' * [3x1]
        torques = torques + (jacobians{i}' * Fatt(:,i) + ...
            jacobians{i}' * Frep(:,i));
    end
    
end