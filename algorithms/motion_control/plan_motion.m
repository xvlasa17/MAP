function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
pose = public_vars.estimated_pose;
%pose = read_only_vars.mocap_pose;
target = get_target(pose, public_vars.path);

% II. Compute motion vector
d=read_only_vars.agent_drive.interwheel_dist;
%public_vars.motion_state = 1;
if(size(public_vars.path,1) == 1)
    vector = [pose(1) pose(2)] + 0.2*[cos(pose(3)) sin(pose(3))];
else
    vector = target - [pose(1) pose(2)];
end
change = 0;

% Stavovy automat ovladani robota podle senzoru
% Zmena stavu podle lidaru
switch public_vars.motion_state
    case 1 % Normalni stav
        if(any(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) < 0.75))
            public_vars.motion_state = 2;
            change = 0;
        end
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.30) < 2)
            public_vars.motion_state = 3;
            change = 1;
        end
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.50) < 3)
            public_vars.motion_state = 4;
            change = 1;
        end
    case 2 % Zpomaleni
        if(all(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.75))
            public_vars.motion_state = 1;
            change = 0;
        end
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.30) < 2)
            public_vars.motion_state = 3;
            change = 1;
        end
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.50) < 3)
            public_vars.motion_state = 4;
            change = 1;
        end
    case 3 % Pomale otaceni podel steny
        if(all(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 1.0))
            public_vars.motion_state = 2;
            change = 1;
        end
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 0.50) < 3)
            public_vars.motion_state = 4;
            change = 0;
        end
    case 4 % Otaceni pred prekazkou
        if(sum(read_only_vars.lidar_distances(1,[1,2,size(read_only_vars.lidar_distances,2)]) > 1.0) >= 2)
            public_vars.motion_state = 3;
            change = 0;
        end
end

switch public_vars.motion_state
    case 1 % Normalni stav
        fi = mod(atan2(vector(2),vector(1))-pose(3)+pi,2*pi)-pi;
        vi = max([read_only_vars.agent_drive.max_vel-abs(fi*d*2),0]);
    case 2 % Zpomaleni
        fi = mod(atan2(vector(2),vector(1))-pose(3)+pi,2*pi)-pi;
        vi = max([read_only_vars.agent_drive.max_vel*0.85-abs(fi*d*2),0]);
        if(change)
            vi = max([read_only_vars.agent_drive.max_vel*0.65-abs(fi*d*2),0]); 
        end
    case 3 % Pomale otaceni podel steny
        if(change)
        if all([isfinite(read_only_vars.lidar_distances(1,2)),isfinite(read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)))])
                fi=pi/16*sign(read_only_vars.lidar_distances(1,2)-read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)));
        else
            if (~isfinite(read_only_vars.lidar_distances(1,2)))
                fi=pi/16;
            else
                fi=-pi/16;
            end
        end
        else
            fi=pi/4*sign(public_vars.motion_vector(1)-public_vars.motion_vector(2));
            if (fi == 0)
                fi=pi/16;
            end
        end
        vi=0.65;
    case 4 % Otaceni pred prekazkou
        if(change)
        if all([isfinite(read_only_vars.lidar_distances(1,2)),isfinite(read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)))])
                fi=pi/8*sign(read_only_vars.lidar_distances(1,2)-read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)));
        else
            if (~isfinite(read_only_vars.lidar_distances(1,2)))
                fi=pi/8;
            else
                fi=-pi/8;
            end
        end
        else
            fi=pi/4*sign(public_vars.motion_vector(1)-public_vars.motion_vector(2));
            if (fi == 0)
                fi=pi/8;
            end
        end
        vi=0.0;
        if((sum(read_only_vars.lidar_distances(1,[2,size(read_only_vars.lidar_distances,2)]) < 1.0) >= 2 || sum(read_only_vars.lidar_distances(1,[3,size(read_only_vars.lidar_distances,2)-1]) < 1.0) >= 2 )&& read_only_vars.lidar_distances(1,1) > 0.5)
            vi = 0.75;
            fi=0.1*sign(read_only_vars.lidar_distances(1,2)-read_only_vars.lidar_distances(1,size(read_only_vars.lidar_distances,2)));
            if fi > 0.1
                fi = 0;
            end
        end
end

public_vars.motion_vector(1) = (vi+fi*d*2);
public_vars.motion_vector(2) = (vi-fi*d*2);

end