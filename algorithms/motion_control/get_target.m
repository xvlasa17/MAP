function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here

for i=size(path,1):-1:1
    if((estimated_pose(1)-path(i,1))^2+(estimated_pose(2)-path(i,2))^2<0.2)
        break
    end
end
target = [path(i,1), path(i,2)];
end

