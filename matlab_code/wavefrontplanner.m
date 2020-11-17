function path = wavefrontplanner(cost, C_space, start_coords, end_coords)
    path = start_coords;
    current = start_coords;
    adjacent = [ 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    angle_list = -90:5:89;
    max_its = 1e4;
    for i = 1:max_its
        current_index = [current(1:2), angletoindex(current(3))];
        mininal_v = 1e10;
        for k = 1:length(adjacent)
            neighbor = current_index + adjacent(k,:);
            if C_space(neighbor(1) ,neighbor(2), neighbor(3))
                continue
            end
            if cost(neighbor(1) ,neighbor(2), neighbor(3)) < mininal_v
                min_neighb = neighbor;
                mininal_v = cost(neighbor(1) ,neighbor(2), neighbor(3));
            end
        end
        angle = ang_range(angle_list(min_neighb(3)));
        current = [min_neighb(1:2), angle];
        path = [path; current];
        if current == end_coords
            "successfully found the path"
            break;
        elseif i == max_its
            "fail to find the path"
        end
    end
end