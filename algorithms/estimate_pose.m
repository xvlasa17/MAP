function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

%estimated_pose = sum(public_vars.particles,1)/size(public_vars.particles,1);

%estimated_pose = public_vars.particles(1,:);
N = size(public_vars.particles, 1);
estimated_pose = mean(public_vars.particles(1:round(N/10),:));

end

