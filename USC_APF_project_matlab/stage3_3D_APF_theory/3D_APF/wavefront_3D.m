function cost = wavefront_3D(goal, logitMap)
% Calculate a wavefront path plan, given the current position, goal position, 
% and current logit(map) occupancy map.

% Threshold the map to get an occupancy grid.
occ = (exp(logitMap)./(exp(logitMap)+1)) > 0.6;
[xaxis, yaxis, zaxis] = size(logitMap);

% Create a cost map
cost = zeros(size(occ));
goali = goal(1);
goalj = goal(2);
goalk = goal(3);

open = [goali goalj goalk];
cost(goali, goalj, goalk) = 1;
adjacent = [ 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];  % immediate adjacents
%adjacent = [ 1 -1; 1 0; 1 1; 0 -1; 0 0; 0 1; -1 -1; -1 0; -1 1 ];
while size(open,1) ~= 0
  % Iterate through cells adjacent to the cell at the top of the open queue:
  for k=1:size(adjacent,1)
    % Calculate index for current adjacent cell:
    adj = open(1,:)+adjacent(k,:);
    % Make sure adjacent cell is in the map
    if min(adj) < 1
      continue
    end
    if adj(1) > xaxis
      continue
    end
    if adj(2) > yaxis
      continue
    end
    if adj(3) > zaxis
      continue
    end
    % Make sure the adjacent cell is not an obstacle 
    if occ(adj(1), adj(2), adj(3)) == 1
      continue
    end
    % Make sure the adjacent cell is not closed:
    if cost(adj(1), adj(2), adj(3)) ~= 0
      continue
    end
    % Set the cost and add the adjacent to the open set
    cost(adj(1), adj(2), adj(3)) = cost(open(1,1), open(1,2), open(1,3)) + 1;
    open(size(open,1)+1,:) = adj;
  end

  % Pop the top open cell from the queue
  open = open(2:end,:);
end