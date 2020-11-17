function index = angletoindex(angle)
    Num_degree = 36;
    angle_list = -90:(180/Num_degree):89;
    if angle >= 90
        angle = angle - 180;
    end
    [~, index] = min(abs(angle_list - angle));
end