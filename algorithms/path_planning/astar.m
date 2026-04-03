function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

x1=1:0.1:2;
y1=3*x1-2;
x2=2:0.1:10;
y2=sin(x2*pi/4-2*pi/4)+4;
y3=4:0.1:8;
x3=y3*0+10;
a=0:5:90;
x4=2*cos(a*pi/180)+8;
y4=2*sin(a*pi/180)+8;
x5=0:0.1:6;
y5=sin(x5*pi)*0.5+10;
x5=-x5+8;


path = [x1 x2 x3 x4 x5;y1 y2 y3 y4 y5]';

end

