function detected_flag = detected_obstacle(start_p, end_p, obstacle)
    detected_flag = 0;
    step_size = round(norm(end_p-start_p));
    for i = 1:step_size
        position = (1 - i/step_size) * start_p + (i/step_size) * end_p;
        if obstacle(round(position(2)), round(position(1)))
            detected_flag = 1;
            break
        end
    end
end