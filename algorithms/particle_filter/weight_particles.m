function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
N = size(particle_measurements, 1);
mu=0.5;
weights = zeros(N,1);
for i=1:N
    P=1;
    for j=1:size(particle_measurements, 2)
        if (isfinite(lidar_distances(1,j)) && isfinite(particle_measurements(i,j)))
            P=P*exp((((lidar_distances(1,j)-particle_measurements(i,j))/mu)^2)/(-2));
        elseif (lidar_distances(1,j)~=particle_measurements(i,j))
            P=P*1e-10;
        end
    end
    weights(i,1)=P;
end
weights(isnan(weights)) = 0.00;
weights = weights/sum(weights);
%max(weights);
end