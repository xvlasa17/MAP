function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
N = size(particle_measurements, 1);
mu=7.5;
weights = zeros(N,1);
for i=1:N
    sum=1;
    for j=1:size(particle_measurements, 2)
        sum=sum*exp((((lidar_distances(1,j)-particle_measurements(i,j))/mu)^2)/(-2));
    end
    weights(i,1)=sum;
end
weights(isnan(weights)) = 0.00;
weightsA = weights/max(weights);
N = size(particle_measurements, 1);
weights = zeros(N,1);
for i=1:N
    sum=0;
    for j=1:size(particle_measurements, 2)
        sum=sum+(lidar_distances(1,j)-particle_measurements(i,j))^2;
    end
    weights(i,1)=1/sqrt(sum)+weights(i,1);
end
weights(isnan(weights)) = 0.000001;
weightsB = weights/max(weights);
weights = [weightsB,weightsA];
end