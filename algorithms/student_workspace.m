function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
public_vars.counter = read_only_vars.counter;
change = 0;

if (read_only_vars.counter == 1)
    public_vars.pf_enabled = 0;
    public_vars.kf_enabled = 0;
end

pf_enabled_old = public_vars.pf_enabled;
kf_enabled_old = public_vars.kf_enabled;

if(isfinite(read_only_vars.gnss_position))
    public_vars.pf_enabled = 0;
    public_vars.kf_enabled = 1;
else
    public_vars.pf_enabled = 1;
    public_vars.kf_enabled = 0;
end

if(public_vars.pf_enabled == 1 && pf_enabled_old == 0 && read_only_vars.counter > 20)
    public_vars = set_init_particle_filter(read_only_vars,public_vars);
    change=1;
end

if(public_vars.kf_enabled == 1 && kf_enabled_old == 0 && read_only_vars.counter > 20)
    public_vars = set_init_kalman_filter(read_only_vars,public_vars);
    change=1;
end

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    if(public_vars.pf_enabled)
        public_vars.init_particle_required = 1;
    else
        public_vars.init_particle_required = 0;
    end
    public_vars.planning_required = 0;
    public_vars.motion_state = 1;
end
if (public_vars.init_particle_required && public_vars.pf_enabled)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.init_particle_required = 0;
end
if (read_only_vars.counter == 20 && public_vars.kf_enabled)
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

% 9. Update particle filter
if (public_vars.pf_enabled)
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end
% 10. Update Kalman filter
if (read_only_vars.counter >= 20 && public_vars.kf_enabled)
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end
% 11. Estimate current robot position
old_estimated_pose = public_vars.estimated_pose;
if ((read_only_vars.counter >= 20 || public_vars.pf_enabled) && change == 0)
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
end

public_vars.estimated_pose

% 12. Path planning
if ((read_only_vars.counter == 20 || (read_only_vars.counter == 1 && public_vars.pf_enabled)))
    public_vars.planning_required = 1;
end
if ((read_only_vars.counter > 20 || (read_only_vars.counter > 1 && public_vars.pf_enabled)))
    if (norm(old_estimated_pose(1:2) - public_vars.estimated_pose(1:2)) > 0.5 )
        public_vars.planning_required = 1;
    end
end
public_vars = plan_path(read_only_vars, public_vars);
% 13. Plan next motion command
if (read_only_vars.counter <= 20 && public_vars.kf_enabled)
    public_vars = cali_motion(read_only_vars, public_vars);
else
    public_vars = plan_motion(read_only_vars, public_vars);
end

% 14. Uncertainty
%public_vars = uncertainty(read_only_vars, public_vars);

%Reach goal?
if (read_only_vars.counter > 20)
if (norm(read_only_vars.map.goal-public_vars.estimated_pose(1:2)) < read_only_vars.map.goal_tolerance/2)
    if public_vars.pf_enabled
        public_vars.init_particle_required = 1;
    end
end
end

[public_vars.motion_state public_vars.motion_vector public_vars.kf_enabled public_vars.pf_enabled public_vars.estimated_pose]

end

