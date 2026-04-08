function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
%pose = public_vars.estimated_pose;
pose = read_only_vars.mocap_pose;
target = get_target(pose, public_vars.path);

% II. Compute motion vector
d=read_only_vars.agent_drive.interwheel_dist;

vector = target - [pose(1) pose(2)];
fi = wrapToPi(atan2(vector(2),vector(1))-pose(3));
vi = max([read_only_vars.agent_drive.max_vel-abs(fi*d*2),0]);

% if(read_only_vars.lidar_distances(1) < 0.5)
%     vi=-0.5;
% end

public_vars.motion_vector(1) = (vi+fi*d*2);
public_vars.motion_vector(2) = (vi-fi*d*2);
end