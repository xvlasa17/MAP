function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
pose = public_vars.estimated_pose;
%pose = read_only_vars.mocap_pose;
target = get_target(pose, public_vars.path);

% II. Compute motion vector
d=read_only_vars.agent_drive.interwheel_dist;

vector = target - [pose(1) pose(2)];
fi = wrapToPi(atan2(vector(2),vector(1))-pose(3));
vi = max([read_only_vars.agent_drive.max_vel-abs(fi*d*2),0]);
if(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 2.0)
    vi = max([read_only_vars.agent_drive.max_vel/2-abs(fi*d*2),0]);
end
if(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 1.0)
    vi = max([read_only_vars.agent_drive.max_vel/4-abs(fi*d*2),0]);
end
if(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 0.5)
    if(read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2))<read_only_vars.lidar_distances(1,1))
        fi=-pi/8;
    else
        fi=pi/8;
    end
    vi=0;
end
public_vars.motion_vector(1) = (vi+fi*d*2);
public_vars.motion_vector(2) = (vi-fi*d*2);
end