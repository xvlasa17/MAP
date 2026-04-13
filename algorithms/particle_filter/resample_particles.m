function [new_particles] = resample_particles(particles, weights, limits)
%RESAMPLE_PARTICLES Summary of this function goes here
N = size(particles, 1);
new_particles = zeros(size(particles));

%Filter
index = floor(rand(1)*N)+1;
for i=1:N
    beta = rand(1)*2*max(weights(:,1));
    while weights(index,1) < beta
        beta = beta - weights(index,1);
        index = index + 1;
        if index > N
            index = 1;
        end
    end
    new_particles(i,:)=particles(index,:);
end

if(max(weights)*N < 10)
    fraction = 0;
else
    fraction = max(weights)*N*0.001;
    if (fraction > 0.75)
        fraction = 0.75;
    end
end
%fraction
n_inject = round(fraction * N);

xMin = limits(1);
yMin = limits(2);
xMax = limits(3);
yMax = limits(4);

%Serazeni podle vahy
A=round(N/10);
[B,I] = sort(weights);

for k = 1:n_inject
    x     = xMin + rand(1) * (xMax - xMin);
    y     = yMin + rand(1) * (yMax - yMin);
    theta = -pi  + rand(1) * 2*pi;
    new_particles(I(k+A), :) = [x, y, theta];
end

%Kopirovani nejsilnejsich hodnot
new_particles(I(1:A),:) = particles(I((N-A+1):N),:);
%weights(I(1:A),:) = weights(I((N-A+1):N),:);
end

