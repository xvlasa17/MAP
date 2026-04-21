function [public_vars] = init_dynamic(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

%% Init
Action = public_vars.action;
Goal = read_only_vars.discrete_map.goal;
Map = read_only_vars.discrete_map.map';
Value = ones(size(Map))*1e10;
Policy = zeros(size(Map));

%% ConvolutionMap
A=[2 2 2 2 2;
   2 3 3 3 2;
   2 3 4 3 2;
   2 3 3 3 2;
   2 2 2 2 2;]./60;
ConvolutionMap = conv2(Map,A);
B=[ 0   0   1   2   1   0   0;
    0   3   13  22  13  3   0;
    1   13  59  97  59  13  1;
    2   22  97  159 97  22  2;
    1   13  59  97  59  13  1;
    0   3   13  22  13  3   0; 
    0   0   1   2   1   0   0;]./1003;
b=5;
ConvolutionMap = conv2(ConvolutionMap,B);
offset = 0.15; 
ConvolutionMap(ConvolutionMap>offset)=offset;

figure(2);
imagesc(ConvolutionMap);


%% Dynamic path planning

Value(Goal(1),Goal(2)) = 0;

change = 1;
while change == 1
    change = 0;
    for x=1:size(Map,1)
        for y=1:size(Map,2)
            for a=1:size(Action,1)
                x2 = x + Action([a,mod(a,8)+1,mod(a+6,8)+1,mod(a+1,8)+1,mod(a+5,8)+1],1);
                y2 = y + Action([a,mod(a,8)+1,mod(a+6,8)+1,mod(a+1,8)+1,mod(a+5,8)+1],2);
                if all(x2>=1) && all(y2>=1) && all(x2<=size(Map,1)) && all(y2<=size(Map,2))
                    if all(all(Map(x2,y2) ~= 1))
                        v2=(Value(x2(1),y2(1))+   Action(a,3)*(1/(offset-ConvolutionMap(x2(1)+b,y2(1)+b)))*0.35 + ...
                                            Action(mod(a,8)+1,3)*(1/(offset-ConvolutionMap(x2(2)+b,y2(2)+b)))*0.225 + ...
                                            Action(mod(a+6,8)+1,3)*(1/(offset-ConvolutionMap(x2(3)+b,y2(3)+b)))*0.225 + ...
                                            Action(mod(a+1,8)+1,3)*(1/(offset-ConvolutionMap(x2(4)+b,y2(4)+b)))*0.1 + ...
                                            Action(mod(a+5,8)+1,3)*(1/(offset-ConvolutionMap(x2(5)+b,y2(5)+b)))*0.1);
                        if v2 < Value(x,y)
                            change = 1;
                            Value(x,y) = v2;
                            Policy(x,y) = a;
                        end
                    end
                end
            end
        end
    end
end

figure(3);
imagesc(Policy);
public_vars.policy = Policy;
end

