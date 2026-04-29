function [public_vars] = set_init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1 0 0; 
                    0 1 0];
public_vars.kf.R = diag([0.001, 0.001, 0.00005]);
%public_vars.kf.R = diag([0.25, 0.25, 0.05]);
public_vars.kf.Q = diag([0.35 0.35]);

%public_vars.mu = [mean(read_only_vars.gnss_history) 0];
%public_vars.sigma = diag([std(read_only_vars.gnss_history).^2 2*pi]);

public_vars.mu = public_vars.estimated_pose;
public_vars.sigma = diag([0.01 0.01 0.01]);

end

