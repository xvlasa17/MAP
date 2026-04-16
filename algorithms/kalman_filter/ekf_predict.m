function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, read_only_vars)
%EKF_PREDICT Summary of this function goes here

v = (u(1) + u(2))/2;  
w = (u(1) - u(2))/read_only_vars.agent_drive.interwheel_dist;  
dt = read_only_vars.sampling_period;

x = mu(1);
y = mu(2);
theta = mu(3);

% new_mu 
x_pred = x + v*cos(theta)*dt;
y_pred = y + v*sin(theta)*dt;
theta_pred = theta + w*dt;
    
new_mu = [x_pred; y_pred; theta_pred];

% new_sigma
G = [ 1, 0, -v*sin(theta)*dt;
      0, 1,  v*cos(theta)*dt;
      0, 0,  1             ];

new_sigma = G * sigma * G' + kf.R;
end

