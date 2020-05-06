
%{
Project: 3DOF Arm launching a projectile
Author: Billy Koech

Module: path generator for projectile launcher


%}

function Q = path3link(ps, pf, obs,xi)
    % apply inverse kinematics to get configurations
    qs = inverseK3DOF(ps);
    qf = inverseK3DOF(pf);
    

    % TODO: set parameters to something meaningful
    epsilon = 3.14165;%1;%3.1418;%1.5709;
    step_size = .01;
    %step_size = .05;
    max_iterations = 500;
    
    % prepare array for trajectory
    Q = [qs];
    
    % set the current configuration to the start configuration
    q_current = qs;
    p_current = forwardK3DOF(qs);
    
    % start iteration counter
    iter = 0;
    
    % iterate with gradient descent until convergence
    while ((norm(p_current - pf) > epsilon) && (iter < max_iterations))
        
        %  get torques at the current configuration
        torques = getCurrentTorques(q_current, qf, obs,xi);
        % disp(norm(p_current - pf));
        % take a step down the gradient
        if norm(p_current - pf) > epsilon
            q_current = q_current + step_size .* torques/norm(torques);
            p_current = forwardK3DOF(q_current);
        else
            return
        end
        
        % add the new configuration to the path
        Q = [Q q_current];
        
        % update iteration counter
        iter = iter + 1;
    end
    disp('done!')
    disp(qf)
    
    % (note: in a while loop, it's always safer to check for maximum
    % iterations to avoid accidentally starting an endless program, even
    % though this isn't strictly part of the algorithm.)
end