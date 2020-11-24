function lmin = local_minimum(f, goal)
    LM1 = islocalmin(f,1);
    LM2 = islocalmin(f,2);
    LM = LM1.*LM2;
    [row, col] = find(LM);
    y = row;
    x = col;
    lmin = [x,y];
    vec = lmin - goal;
    [m n] = size(vec);
    for i = 1 : m
        dis(i) = norm(vec(i,:));
    end
    [minimum, index] = min(dis);
    lmin(index, :) = [];
end
