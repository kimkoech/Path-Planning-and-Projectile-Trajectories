%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Calculate repulsive forces on each link 


%}


% Given the current configuration q and a [3xM] matrix of obstacles (where 
% each column gives (x, y, radius), return a 3x3 matrix in which each column
% is a force vector acting on the ith frame: [F1, F2, F3].
function forces = getCurrentRepulsiveForces(q, obs)
 
    nu = [1, 1, 1];
    region_of_influence = 2;

    % get current origins
    origins = getCurrentOrigins(q);

    % Replsive "forces" matrix (the ith column corresponds to the forces on the ith link)
    
    a = origins;

    % preallocate
    forces = zeros(3, 3);
    
    % for each joint
    for i = 1:length(a)

        % for each obstacle
        for j=1:length(obs)
            
            xyz_obs = [obs(1:2,j);0]; 
            radius = obs(3,j);
            p_o = radius + region_of_influence; % region of influence
            v_obs_i = -xyz_obs+a(:,i);
  
            
            %rho = norm(v_obs_i - (radius * (v_obs_i/norm(v_obs_i))));
            rho = norm(a(:,i) - xyz_obs);
            
            %  point on obstacle closest to oi
            b = xyz_obs + (radius * (v_obs_i/norm(v_obs_i)));
            
            % compute the force
            forces(:,i) = forces(:,i) + (nu(i) *...
                ((1/rho) - (1/p_o)) * ...
                (1/rho^2) * ...
                ((a(:,i) - b)/norm(a(:,i)-b)));

        end

    end
    
    
end