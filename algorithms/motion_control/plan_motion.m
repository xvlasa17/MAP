function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

public_vars.estimated_pose = read_only_vars.mocap_pose;

target = get_target(public_vars.estimated_pose, public_vars.path);

% II. Compute motion vector
d=read_only_vars.agent_drive.interwheel_dist;
x=public_vars.estimated_pose(1);
y=public_vars.estimated_pose(2);
f=public_vars.estimated_pose(3);

vector = target - [x y];
vi = norm(vector);
fi = wrapToPi(atan2(vector(2),vector(1))-f);

a = (2*vi+fi*d);
b = (2*vi-fi*d);

k=max(a,b)-read_only_vars.agent_drive.max_vel;
a = (2*vi-k+fi*d);
b = (2*vi-k-fi*d);

public_vars.motion_vector(1) = a;
public_vars.motion_vector(2) = b;
end