

%{
Project: 3DOF Arm launching a projectile
Original Author: ES 159/259 Staff, Spring 2020 
Editor: Billy Koech

Module: visualize the path of a 3 link manipulator


%}

% Function for visualization (no need to modify, unless you'd like to!)
function visualizePath(Q, ps, pf, obs, projectiles)
    % apply inverse kinematics to get configurations
    qs = inverseK3DOF(ps);
    qf = inverseK3DOF(pf);

    % get desired start and final end-effector points
    os = getCurrentOrigins(qs);
    of = getCurrentOrigins(qf);

    % get trajectories of each origin
    o1_path = zeros(size(Q));
    o2_path = zeros(size(Q));
    o3_path = zeros(size(Q));
    for i = 1:size(Q, 2)
        os = getCurrentOrigins(Q(:, i));
        o1_path(:, i) = os(:, 1);
        o2_path(:, i) = os(:, 2);
        o3_path(:, i) = os(:, 3);
    end

    % plot joint values
    figure()
    for i = 1:3
        subplot(1, 3, i)
        plot(Q(i,:))
        title(sprintf('Joint %d', i))
    end
    
    % plot animation of the manipulator
    figure()
    hold on
    plot(os(1, 3), os(2, 3), 'r*', 'MarkerSize', 10)
    plot(of(1, 3), of(2, 3), 'r*', 'MarkerSize', 10)
    plot(o1_path(1,:), o1_path(2,:))
    plot(o2_path(1,:), o2_path(2,:))
    plot(o3_path(1,:), o3_path(2,:))
    
    %%%
    % plot projectiles
    [r_cell, c_cell] = size(projectiles);
    for i=1:c_cell
        proj_traj = projectiles{i};
        plot(proj_traj(1,:), proj_traj(2,:));
    end
    %%%
    for i = 1:size(obs, 2)
        th = 0 : pi/50 : 2*pi;
        x = obs(3, i) * cos(th) + obs(1, i);
        y = obs(3, i) * sin(th) + obs(2, i);
        plot(x, y, 'Color', [184, 161, 230]/256, 'LineWidth', 1.5);
        h = fill(x, y, [184, 161, 230]/256);
    end
    
    link1 = plot([0 o1_path(1, 1)], [0 o1_path(2, 1)], 'k*-', 'LineWidth', 1.5);
    link2 = plot([o1_path(1, 1), o2_path(1, 1)], [o1_path(2, 1), o2_path(2, 1)], 'k*-', 'LineWidth', 1.5);
    link3 = plot([o2_path(1, 1), o3_path(1, 1)], [o2_path(2, 1), o3_path(2, 1)], 'k*-', 'LineWidth', 1.5);
    
    for i = 1:size(Q, 2)
        link1.XData = [0 o1_path(1, i)];
        link1.YData = [0 o1_path(2, i)];
        link2.XData = [o1_path(1, i) o2_path(1, i)];
        link2.YData = [o1_path(2, i) o2_path(2, i)];
        link3.XData = [o2_path(1, i) o3_path(1, i)];
        link3.YData = [o2_path(2, i) o3_path(2, i)];
        
        drawnow
        pause(0.05)
    end
end
