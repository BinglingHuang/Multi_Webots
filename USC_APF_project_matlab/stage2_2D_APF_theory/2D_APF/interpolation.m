function improved_path = interpolation(S)
    shape = size(S);
    improved_path = S(1, :);
    for i = 1:shape(1)-1
        step_size = round(norm(S(i+1,:) - S(i,:)));
        for j = 1:step_size
            position = (1 - j/step_size) * S(i, :) + (j/step_size) * S(i+1, :);
            improved_path = [improved_path; round(position)];
        end
    end
end