function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here

for i=size(path,1):-1:0
    if (i == 0)
        break
    end
    d(i) = norm([estimated_pose(1)-path(i,1),estimated_pose(2)-path(i,2)]);
    if(d(i)<0.4)
        break
    end
end
if (i == 0)
    [a,i]=min(d);
end
    target = [path(i,1), path(i,2)];
end

