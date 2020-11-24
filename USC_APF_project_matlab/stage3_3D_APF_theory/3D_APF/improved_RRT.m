function improved_path = improved_RRT(visited_point, C_space)
    S_end = visited_point(end, :);
    S = visited_point(1,:);
    Num_point = size(visited_point);
    for k = 2:Num_point(1)
        if ~detected_obstacle(S(end,:), visited_point(k,:), C_space) 
            continue
        else
            S = [S; visited_point(k - 1,:)];
        end
    end
    S = [S; S_end];
%     S = visited_point;
    improved_path = interpolation(S);
end