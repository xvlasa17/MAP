function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

%estimated_pose = sum(public_vars.particles,1)/size(public_vars.particles,1);

estimated_pose = [0,0,0];
N = size(public_vars.particles, 1);
if(public_vars.pf_enabled)
    estimated_pose = mean(public_vars.particles(1:round(N/10),:));
end
if(public_vars.kf_enabled)
    estimated_pose = public_vars.mu';
end

end

