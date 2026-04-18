function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

x1=1:0.2:6.5;
y1=x1;
a=(135:-2:0);
x2=4.14*sqrt(2)*cos(a*pi/180)+10.14;
y2=4*sqrt(2)*sin(a*pi/180)+2;
path = [x1 x2;y1 y2]';

% y1=0:0.2:4;
% x1=0*y1+2;
% x2=2:0.1:10;
% y2=sin(x2*pi/4-2*pi/4)+4;
% y3=4:0.3:8;
% x3=y3*0+10;
% a=0:5:90;
% x4=2*cos(a*pi/180)+8;
% y4=2*sin(a*pi/180)+8;
% x5=0:0.1:8;
% y5=x5*0+10;
% x5=-x5+8;
% 
% path = [x1 x2 x3 x4 x5;y1 y2 y3 y4 y5]';

% y1=0:0.2:10;
% x1=0*y1+1;
% path=[x1;y1]';
end

