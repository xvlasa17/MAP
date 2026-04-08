function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

%estimated_pose = sum(public_vars.particles,1)/size(public_vars.particles,1);
estimated_pose = median(public_vars.particles);
%estimated_pose = nan(1,3);

end

