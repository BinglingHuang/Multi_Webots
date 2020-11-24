function route = Gradient_descent(f, C_space, start_coords, end_coords)
%% before running this code, run test_1 first.
    [gx, gy, gz] = gradient(-f);
    route=start_coords;
    current=start_coords;
    angle_list = -90:5:89;
    max_its = 1e4;
    for i = 1:max_its
        theta = angletoindex(current(3));
        x_gradient = gx(current(1),current(2),theta);
        y_gradient = gy(current(1),current(2),theta);
        z_gradient = 2 * gz(current(1),current(2),theta);
        g = [x_gradient, y_gradient, z_gradient]
        if C_space(current(1),current(2),theta) == 0
            g = 2 * rand(1,3) - 1;
        end
        ncoords = round([current(1),current(2),theta] + g/(norm(g) + 1e-4));
        angle = ang_range(angle_list(ncoords(3)));
        current = [ncoords(1), ncoords(2), angle];
        route = [route;current]
        d = metric(current, end_coords);
        if d < 3
            "successfully found the path"
            break
        elseif i == max_its
            "fail to find the path"
        end
    end

end