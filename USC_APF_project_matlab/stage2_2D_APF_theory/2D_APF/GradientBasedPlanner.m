function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
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

[gx, gy] = gradient (-f);
route=start_coords;
current=start_coords;
ncoords=[];


%%% All of your code should be between the two lines of stars.
% *******************************************************************
for i=1:max_its

g=[gx(round(current(2)),round(current(1))),gy(round(current(2)),round(current(1)))];  % use the round function to calculate current position and get gx, gy
if norm(g) > 0.03
    ncoords=current+g/norm(g);   % normalized g to ensure distance btw successive nodes equals to 1
else
%     g = rand(1,2);
    ncoords = current + g/norm(g);
end
current=ncoords; 
route=[route;current]; % add new coordinate node into route, it will be a floating number

X=[current;end_coords];
d=pdist(X,'euclidean');
if d<2
    %disp('end');
    break
end

end
% *******************************************************************
end