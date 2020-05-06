%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: Calculate attractive forces on each link


%}


% Given the current configuration q and end configuration qf, return a 3x3
% matrix in which each column is a force vector acting on the ith frame: [F1, F2, F3].
function forces = getCurrentAttractiveForces(q, qf, xi)
    % TODO: set d to a meaningful value
    % TODO: set xi to a meaningful vector
    d = 1;
    % smooth parth
    %xi = [1000, 1, 1]
    %xi = [1, 100, 100];
    %xi = [1, 10, 1000];
    
    % get current origins and final origins
    origins = getCurrentOrigins(q);
    final_origins = getCurrentOrigins(qf);

    % TODO: set the attractive "forces" matrix (the ith column corresponds to the forces on the ith link)
    forces = zeros(3, 3);
    
    % compute difference
    diff = origins - final_origins;
    
    % compute attractive forces
    for i=1:length(origins)
        
        norm_diff = norm(diff(:,i));
        if norm_diff <= d
            forces(:,i) = -0.5 *  xi(i) * diff(:,i);
        else
            forces(:,i) = -d * xi(i) * diff(:,i)/ norm_diff;
        end
             
    end
    
    
    
end