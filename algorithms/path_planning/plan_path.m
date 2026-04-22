function [public_vars] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

public_vars.action =   [ 1, 0, 1       ;
                         1, 1, sqrt(2) ;
                         0, 1, 1       ;
                        -1, 1, sqrt(2) ;
                        -1, 0, 1       ;
                        -1,-1, sqrt(2) ;
                         0,-1, 1       ;
                         1,-1, sqrt(2) ];
%public_vars.planning_required=1;

if (read_only_vars.counter == 1)
    public_vars = init_dynamic(read_only_vars, public_vars);
end

if (public_vars.planning_required == 1)
    public_vars.planning_required=0;
    public_vars.path = get_path_dynamic(read_only_vars,public_vars);
    public_vars.path = smooth_path(public_vars.path);
end
end

