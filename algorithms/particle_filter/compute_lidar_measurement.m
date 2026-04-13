function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_LIDAR_MEASUREMENT Simulates lidar readings for a particle pose

measurement = zeros(1, length(lidar_config));

for i = 1:length(lidar_config)
    % absolutní směr
    direction = pose(3) + lidar_config(i);
    % ziskani bodu pruseciku paprsku se stenou mapy
    intersections = ray_cast([pose(1), pose(2)], map.walls, direction);
    if isempty(intersections)
        measurement(i) = Inf;
    else
        % vypocet vzdalenosti
        dists = sqrt((intersections(:,1) - pose(1)).^2 + (intersections(:,2) - pose(2)).^2);
        measurement(i) = min(dists);
    end
end

end

