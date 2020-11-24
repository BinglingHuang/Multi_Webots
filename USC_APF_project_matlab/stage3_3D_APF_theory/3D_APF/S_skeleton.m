function [S, d1] = S_skeleton(C_space)
    % C_space 0 free 1 collision
    adjacent = [ 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    %% for every x belongs to x-free, set d1(x) to M (infinity number)
    M = 1e4;
    O = containers.Map;
    C_size = size(C_space);
    S = zeros(C_size);
    d1 = zeros(C_size);
    [m, n, k] = ind2sub(size(C_space), find(C_space == 0));
    for i = 1: length(m)
        d1(m(i), n(i), k(i)) = M;
    end
    %% Scan C_space and identify every point x such that BM(x) = 1 
    % and one of its neighbors is in C_space(x) x == 0
    L0 = []; % frame boundary of the C_sapce
    [m, n, k] = ind2sub(size(C_space), find(C_space == 1));
    for i = 1: length(m)
        p = [m(i) n(i) k(i)];
        neighbor = p + adjacent;
        for j = 1:length(neighbor)
            q = neighbor(j,:);
            if min(q) < 1
                continue
            end
            if q(1) > C_size(1)
              continue
            end
            if q(2) > C_size(2)
              continue
            end
            if q(3) > C_size(3)
              continue
            end
            if C_space(q(1),q(2),q(3)) == 0
                L0 = [L0;p];
                O(num2str(p)) = p;
                d1(p(1),p(2),p(3)) = 0;
                break;
            end
        end
    end
    %% wave_front_algorithm
    while size(L0,1) ~= 0
        Ox = O(num2str(L0(1,:)));
        for k = 1:size(adjacent, 1)
            adj = L0(1,:) + adjacent(k,:);
            if min(adj) < 1
                continue
            end
            if adj(1) > C_size(1)
              continue
            end
            if adj(2) > C_size(2)
              continue
            end
            if adj(3) > C_size(3)
              continue
            end
            if C_space(adj(1), adj(2), adj(3)) == 1
              continue
            end
            if d1(adj(1), adj(2), adj(3)) ~= M
                Oy = O(num2str(adj));
                dis = sum(abs(Oy-Ox));
                if dis >= 4
                    S(adj(1),adj(2),adj(3)) = 1;
                end
              continue
            end
            d1(adj(1), adj(2), adj(3)) = d1(L0(1,1), L0(1,2), L0(1,3)) + 1;
            L0(size(L0,1)+1,:) = adj;
            O(num2str(adj)) = Ox;
        end
        L0 = L0(2:end,:);
    end
end



