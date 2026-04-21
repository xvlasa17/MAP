function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

public_vars.counter = read_only_vars.counter;
% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars.pf_enabled = 0;
    public_vars.kf_enabled = 0;
    public_vars.planning_required = 0;
end
if (read_only_vars.counter == 20)
%    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
%    public_vars.kf.R = diag([0.01, 0.01, 0.05]);
    public_vars.pf_enabled = 0;
    public_vars.kf_enabled = 1;
end
if (read_only_vars.counter == 30)
%public_vars.kf.R = diag([0.001, 0.001, 0.00005]);
end

% 9. Update particle filter
if (read_only_vars.counter >= 20 && public_vars.pf_enabled)
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end
% 10. Update Kalman filter
if (read_only_vars.counter >= 20 && public_vars.kf_enabled)
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end
% 11. Estimate current robot position
if (read_only_vars.counter >= 20)
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
end
% 12. Path planning
if (mod(read_only_vars.counter,100) == 20)
    public_vars.planning_required = 1;
end
    public_vars = plan_path(read_only_vars, public_vars);
% 13. Plan next motion command
if (read_only_vars.counter <= 20)
    public_vars = cali_motion(read_only_vars, public_vars);
else
    public_vars = plan_motion(read_only_vars, public_vars);
end

% 14. Uncertainty
%public_vars = uncertainty(read_only_vars, public_vars);
end

