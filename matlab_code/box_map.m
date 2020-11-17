function box_map(bp, W, box_length, box_width)
    x = bp(1);
    y = bp(2);
    rad = bp(3)*pi/180;
    map = zeros(size(W));
    map_after_r = zeros(size(W));
    par2 = (box_width - 1)/2;
    par1 = (box_length - 1)/2;
    map(x-par2:x+par2,y-par1:y+par1) = 1;
    R = [cos(rad) -1 * sin(rad); sin(rad) cos(rad)];
    [row, column] = find(map);
    Index = [row,column];
    fix_point = bp(1:2);
    for i = 1:length(Index)
        delta = Index(i,:)' - fix_point';
        P_new = R*delta + fix_point';
        P_new = round(P_new);
        map_after_r(P_new(1), P_new(2)) = 1;
    end
    answer = map_after_r | W;
    answer = flipud(answer);
    map1 = occupancyMap(answer);
    show(map1)
    pause(0.5);
end