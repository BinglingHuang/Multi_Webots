function improved_path = improved_lm(visited_point, obstacle)
    S_end = visited_point(end, :);
    S = visited_point(1,:);
    Num_point = size(visited_point);
    for k = 2:Num_point(1)
        if ~detected_obstacle(S(end,:), visited_point(k,:), obstacle)
            continue
        else
            S = [S; visited_point(k,:)];
        end
    end
    S = [S; S_end];
    improved_path = interpolation(S);
end


