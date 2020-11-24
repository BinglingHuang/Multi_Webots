function [g_record, route] = GradientBasedPlanner_lm (att, rep, start_coords, end_coords, max_its)
  %function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate
f = att + rep;
lmin = local_minimum(f, end_coords);
[num_lm column] = size(lmin);
[gx, gy] = gradient (-f);
route=start_coords;
current=start_coords;
ncoords=[];
g_record = [];
local_minimum_flag = 0;

%%% All of your code should be between the two lines of stars.
% *******************************************************************
for i=1:max_its
%     for j = 1:num_lm
%         norm(lmin(j,:) - current)
%     end
g=[gx(round(current(2)),round(current(1))),gy(round(current(2)),round(current(1)))];  % use the round function to calculate current position and get gx, gy
g_record = [g_record; g];
if norm(g) > 0.1 && local_minimum_flag == 0
    ncoords=current+g/norm(g);   % normalized g to ensure distance btw successive nodes equals to 1
else
    local_minimum_flag = 1;
    for j = 1:num_lm
        dis(j) = norm(lmin(j,:) - current);
    end
    [mini, minimal_index] = min(dis);
    Xtp = lmin(minimal_index, :);
    if Xtp - current == 0
        g_virob = 0;
    else
        g_virob = 50 * (current - Xtp)/norm(Xtp - current)^3;
    end
    if rand(1) > 0.5
        g = rand(1,2);
    else
        g = g + g_virob;
    end
    ncoords = current + g/norm(g);
    norm(Xtp - current);
    if norm(g_virob) < 0.2
        local_minimum_flag = 0;
    end
end
current = ncoords;
route=[route;current]; % add new coordinate node into route, it will be a floating number

X=[current;end_coords];
d=pdist(X,'euclidean');
if d<3
    %disp('end');
    break
end

end
% *******************************************************************
end