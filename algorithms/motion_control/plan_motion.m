function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
pose = public_vars.estimated_pose;
%pose = read_only_vars.mocap_pose;
target = get_target(pose, public_vars.path);

% II. Compute motion vector
d=read_only_vars.agent_drive.interwheel_dist;
public_vars.motion_state = 1;
vector = target - [pose(1) pose(2)];
fi = wrapToPi(atan2(vector(2),vector(1))-pose(3));
vi = max([read_only_vars.agent_drive.max_vel-abs(fi*d*2),0]);
if(any(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 0.5))
    vi = max([read_only_vars.agent_drive.max_vel/2-abs(fi*d*2),0]);
    public_vars.motion_state = 2;
end
if(all(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.5))
    public_vars.motion_memory=0.4;
end
if(any(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < public_vars.motion_memory))
    public_vars.motion_memory=0.65;
    public_vars.motion_state = 4;
    fi=pi/8*sign(read_only_vars.lidar_distances(1,1)-read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)));
    vi=0.25;
    if(all(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 0.5))
        fi=pi/4*sign(public_vars.motion_vector(1)-public_vars.motion_vector(2));
        vi=0.0;
        public_vars.motion_state = 5;
    end
end
public_vars.motion_state
public_vars.motion_vector(1) = (vi+fi*d*2);
public_vars.motion_vector(2) = (vi-fi*d*2);
end