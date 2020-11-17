% map = box_map(path(1,:), W, length, width);
% map = flipud(map);
% map1 = occupancyMap(map);
% show(map1)
start_coords = [30, 30, 0];
end_coords = [8, 8, 0];
% route = wavefrontplanner(cost, C_space, start_coords, end_coords);
route = Gradient_descent(f, C_space, start_coords, end_coords);