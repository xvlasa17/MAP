function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));
for i=1:length(lidar_config)
    pozice=[pose(1),pose(2)];
    measurement(1,i)=norm(min(ray_cast(pozice,map.walls,lidar_config(1,i)+pose(3))-pozice));
end
end

