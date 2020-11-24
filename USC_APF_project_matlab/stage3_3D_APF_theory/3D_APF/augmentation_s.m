function au_s = augmentation_s(S, goal, d1)
    au_s = S;
    adjacent = [ 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    current = goal;
    while au_s(current(1),current(2),current(3)) ~= 1
        au_s(current(1),current(2),current(3)) = 1;
        max = 1;
        for i = 1:length(adjacent)
            neighbor = current + adjacent(i,:);
            potential = d1(neighbor(1),neighbor(2),neighbor(3));
            if potential > max
                min_neighbor = neighbor;
                max = potential;
            end
        end
        current = min_neighbor;
    end
end