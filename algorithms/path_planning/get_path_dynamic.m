function [Path] = get_path_dynamic(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

%% Init
Policy = public_vars.policy;
Action = public_vars.action;
Goal = read_only_vars.discrete_map.goal;
Map = read_only_vars.discrete_map.map';
limitSizeX = read_only_vars.discrete_map.limits(3) - read_only_vars.discrete_map.limits(1);
limitSizeY = read_only_vars.discrete_map.limits(4) - read_only_vars.discrete_map.limits(2);
ScaleStep = [limitSizeX limitSizeY] ./ [size(Map)];
Start = round(public_vars.estimated_pose(1,1:2) ./ ScaleStep);

%% get path from Dynamic path planning

Point = Start;
i=1;
Path(i,:) = Point.*ScaleStep;
while any(Goal ~= Point)
    if (Policy(Point(1),Point(2)) == 0)
         public_vars.planning_required = 1;
         break;
    end
    Point=Point+Action(Policy(Point(1),Point(2)),1:2);
    i=i+1;
    Path(i,:) = Point.*ScaleStep-ScaleStep./2;
end

end

