function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here
dt   = read_only_vars.sampling_period;
L    = read_only_vars.agent_drive.interwheel_dist;
vR = motion_vector(1);
vL = motion_vector(2);
% vypocet rychlosti
v     = (vR + vL) / 2;
omega = (vR - vL) / L;
% pridani sumu/noise
SIGMA_V     = 0.15;
SIGMA_OMEGA = 0.15;
v     = v     + randn * SIGMA_V;
omega = omega + randn * SIGMA_OMEGA;
% aktualizace pozice
theta     = old_pose(3) + omega * dt;
new_pose  = [old_pose(1) + v * cos(theta) * dt, old_pose(2) + v * sin(theta) * dt,  theta];
end


