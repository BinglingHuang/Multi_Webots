function improved_path = interpolation(path)
    improved_path = path(1, :);
    size_path = size(path);
    for i = 1:(size_path(1) - 1)
        step_size = round(metric(path(i, :), path(i+1, :)));
        theta1 = path(i,3);
        theta2 = path(i+1,3);
        delta_theta = abs(theta1 - theta2);
        for j = 1: step_size
            position = (1 - j/step_size) * path(i, 1:2) + (j/step_size) * path(i + 1, 1:2);
            if delta_theta <= 90
                theta = (1 - j/step_size) * theta1 + (j/step_size) * theta2;
            else
                theta1 = theta1 * (theta1 <= 90) + (theta1 - 180) * (theta1 > 90);
                theta2 = theta2 * (theta2 <= 90) + (theta2 - 180) * (theta2 > 90);
                theta = (1 - j/step_size) * theta1 + (j/step_size) * theta2;
                theta = ang_range(theta);
            end
            improved_path = [improved_path; round(position) round(theta)];
        end
    end
end