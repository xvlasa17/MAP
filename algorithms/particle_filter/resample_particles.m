function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here
N = size(particles, 1);
new_particles = zeros(size(particles));
%typ=round(rand(1)+1.5);
typ=1;
index = floor(rand(1)*N)+1;
for i=1:N
    beta = rand(1)*2*max(weights(:,typ));
    while weights(index,typ) < beta
        beta = beta - weights(index,typ);
        index = index + 1;
        if index > N
            index = 1;
        end
    end
    new_particles(i,:)=particles(index,:);
end

% r = rand(1)/N;
% c = weightsA(1,1);
% i = 1;
% 
% for n = 1:N/2
%     u = r + (n-1)/N;
%     while u > c
%         i = i + 1;
%         c = c + weightsA(i,1);
%     end
%     new_particles(n,:) = particles(i,:);
% end
% particles = new_particles;
% weightsB = weights ./ Sum(2);
% 
% r = rand(1)/N;
% c = weightsB(N/2,2);
% i = N/2;
% 
% for n = N/2:N
%     u = r + (n-1)/N;
%     while u > c
%         i = i + 1;
%         if i > N
%             i=1;
%         end
%         c = c + weightsB(i,2);
%     end
%     new_particles(n,:) = particles(i,:);
% end

end

