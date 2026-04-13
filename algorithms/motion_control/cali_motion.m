function [public_vars] = cali_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
d=read_only_vars.agent_drive.interwheel_dist;

fi = -0.25;
vi = 0.9;
if(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 0.5)
    vi=-0.25;
end
public_vars.motion_vector(1) = (vi+fi*d*2);
public_vars.motion_vector(2) = (vi-fi*d*2);
end