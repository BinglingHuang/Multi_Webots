function d = metric(Q1, Q2)
    wxy = 1;
    wth = 0.2;
    Q1_p = Q1(:, 1:2);
    Q1_theta = Q1(:, 3);
    Q2_p = Q2(:, 1:2);
    Q2_theta = Q2(:, 3);
    p = sqrt(sum((Q1_p - Q2_p).^2, 2));
    del = abs(Q1_theta - Q2_theta);
    theta = del.* (del <= 90) + (180 - del) .* (del > 90);
    d = wxy * p + wth * theta;
end