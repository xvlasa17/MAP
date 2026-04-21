function [public_vars] = init_dynamic(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

%% Init
Action = public_vars.action;
Goal = read_only_vars.discrete_map.goal;
Map = read_only_vars.discrete_map.map';
Value = ones(size(Map))*1e10;
Policy = zeros(size(Map));

%% ConvolutionMap
A=[0.75 0.75 0.75;
   0.75   1  0.75;
   0.75 0.75 0.75;]./7;
ConvolutionMap = conv2(Map,A);
B=[ 0   0   1   2   1   0   0;
    0   3   13  22  13  3   0;
    1   13  59  97  59  13  1;
    2   22  97  159 97  22  2;
    1   13  59  97  59  13  1;
    0   3   13  22  13  3   0; 
    0   0   1   2   1   0   0;]./1003;
b=4;
ConvolutionMap = conv2(ConvolutionMap,B);
ConvolutionMap(ConvolutionMap>0.25)=0.25;

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
                x2 = x + Action(a,1);
                y2 = y + Action(a,2);
                if x2>=1 && y2>=1 && x2<=size(Map,1) && y2<=size(Map,2) && Map(x2,y2) ~= 1
                    v2=Value(x2,y2)+Action(a,3)*(1/(0.25-ConvolutionMap(x2+b,y2+b)));
                    if v2 < Value(x,y)
                        change = 1;
                        Value(x,y) = v2;
                        Policy(x,y) = a;
                    end
%                else
%                    Policy(x,y) = mod(a+3,8)+1 ;
                end
            end
        end
    end
end

figure(3);
imagesc(Policy);
public_vars.policy = Policy;
end

