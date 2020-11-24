function route = GradientBasedPlanner_SAlm (f, att, obs, start_coords, end_coords, max_its)
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
dist = bwdist(obs);
dist(dist<10)=0;
lmin = local_minimum(f, end_coords)
% lmin = [21 63; 29 123; 65 68];
[num_lm column] = size(lmin);
[gx, gy] = gradient (-f);
visited_point = [];
visited_point_history = []; 
near_point = [];
route=start_coords;
current=start_coords;
ncoords=[];
local_minimum_flag = 0;
T0 = 1000;
T = T0;
Tmin = 1e-5;
% dist(dist<10)=0;
%%% All of your code should be between the two lines of stars.
% *******************************************************************
for i=1:max_its
g=[gx(round(current(2)),round(current(1))),gy(round(current(2)),round(current(1)))];  % use the round function to calculate current position and get gx, gy
for j = 1:num_lm
    dis(j) = norm(lmin(j,:) - current);
end
[minimal, minimal_index] = min(dis);
if minimal > 3 && local_minimum_flag == 0
    ncoords=round(current+g/norm(g));   % normalized g to ensure distance btw successive nodes equals to 1
    current = ncoords;
    route=[route;current];
else
%%  calculate which local minimum it trapped in
    if local_minimum_flag == 0
        Xtp = lmin(minimal_index, :);
    end
    local_minimum_flag = 1;
%% find near point surround the current position
    near_point = current + [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
    length = size(near_point);
    near_point_new = [];
    for index = 1:length(1)
        if dist(round(near_point(index,2)), round(near_point(index,1))) <= 10
            continue;
        end
        if ~isempty(visited_point) && ismember(near_point(index,:), visited_point,"rows")
            continue;
        end
        near_point_new = [near_point_new; near_point(index,:)];
    end
%%   selet the best points
    length = size(near_point_new);
    pmin = 1000;
    index_min = -1;
    for index = 1:length(1)
         potential = att(round(near_point_new(index,2)), round(near_point_new(index,1)));
         if potential < pmin
             pmin = potential;
             index_min = index;
         end
    end
    if index_min == -1
        "can't find the path"
        break;
    else
        visited_point = [visited_point; near_point_new(index_min,:)];
%         visited_point_history = [visited_point_history; near_point_new(index_min,:)];
    end
    new_ncoords = near_point_new(index_min,:);
%%  check if out of the local minimum
    X = [new_ncoords; end_coords];
    if f(round(new_ncoords(2)), round(new_ncoords(1))) <= f(round(Xtp(2)), round(Xtp(1))) || pdist(X,'euclidean') < 3
        ncoords = visited_point;
        "out of the local minimum"
        visited_point = [];
        local_minimum_flag = 0;
        T = T0;
    end
    current = new_ncoords;
end

%%  update the route path
if local_minimum_flag == 0
    route=[route;ncoords]; % add new coordinate node into route, it will be a floating number
%     ncoords = improved_lm(route, obs);
end
X=[current; end_coords];
d=pdist(X,'euclidean');
if d<3
    "successfully found the path"
    
    break
end

end
end