function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;

alpha = 0.1;
beta = 0.3;

while true
    delta_sum = [0, 0];
    for i = 2:length(old_path)-1
        delta = alpha*(old_path(i,:) - new_path(i,:)) + beta*(new_path(i-1,:) + new_path(i+1,:) -2*new_path(i,:));
        new_path(i,:) = new_path(i,:) + delta;
        delta_sum = delta_sum + abs(delta);
    end
    if norm(delta_sum) < 0.1
        return;
    end
end

end

