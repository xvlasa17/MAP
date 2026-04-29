function [public_vars] = set_init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Initializes particles at random poses within map limits

N = read_only_vars.max_particles;

% Generovani nahodnych hodnot
x     = public_vars.estimated_pose(1) + rand(N,1) * (0.5) - 0.25;
y     = public_vars.estimated_pose(2) + rand(N,1) * (0.5) - 0.25;
theta = mod(public_vars.estimated_pose(3) + rand(N,1) * (0.2) - 0.1+pi,2*pi)-pi;

public_vars.particles = [x, y, theta];

end

