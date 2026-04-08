function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here
ScaleX = read_only_vars.map.limits(1,3) - read_only_vars.map.limits(1,1);
ScaleY = read_only_vars.map.limits(1,4) - read_only_vars.map.limits(1,2);
Particles = rand(read_only_vars.max_particles,3);
public_vars.particles = [Particles(:,1)*ScaleX+read_only_vars.map.limits(1,1),Particles(:,2)*ScaleY+read_only_vars.map.limits(1,2),(Particles(:,3)*pi*2-pi)];
end

