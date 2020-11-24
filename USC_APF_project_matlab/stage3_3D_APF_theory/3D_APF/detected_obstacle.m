function detected_flag = detected_obstacle(start_p, end_p, C_space)
    detected_flag = 0;
    step_size = round(metric(end_p, start_p));
    theta1 = start_p(3);
    theta2 = end_p(3);
    delta_theta = abs(theta1 - theta2);
    for i = 1:step_size
        position = (1 - i/step_size) * start_p(1:2) + (i/step_size) * end_p(1:2);
        if delta_theta <= 90
            theta = (1 - i/step_size) * theta1 + (i/step_size) * theta2;
        else
            theta1 = theta1 * (theta1 <= 90) + (theta1 - 180) * (theta1 > 90);
            theta2 = theta2 * (theta2 <= 90) + (theta2 - 180) * (theta2 > 90);
            theta = (1 - i/step_size) * theta1 + (i/step_size) * theta2;
            theta = ang_range(theta);
        end
        if C_space(round(position(1)), round(position(2)), angletoindex(theta))
            detected_flag = 1;
            break
        end
    end
end