function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here
R=0.1;
a=0.25;
motion_vector_noise=motion_vector.*((rand(1,2)).*a+1.0-a/2);
wi = R*(motion_vector_noise(1) - motion_vector_noise(2))/(read_only_vars.agent_drive.interwheel_dist);
vi = R*(motion_vector_noise(1) + motion_vector_noise(2))/2;
w = wi + old_pose(3);
new_pose = [old_pose(1,1:2),0] + [cos(w)*vi,sin(w)*vi,w];% + [rand(1,2),0]*a;

end

