function angle = ang_range(theta)
    if theta >= 0 && theta <= 180
        angle = theta;
        return
    elseif theta > 180
        angle = ang_range(theta - 180);
    elseif theta < 0
        angle = ang_range(theta + 180);
    end
end
