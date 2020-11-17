import numpy as np

def check_open_list(open_list, newPos):
    par = np.array(open_list)
    par = par[:, 0:2]
    c = np.sum(abs(par - newPos), axis=1)
    Near_index = np.where(c == 0)
    ans = Near_index[0]
    return ans

def Astar_path_planner(field, lmin, current, distance_array, end_coords):
    [lminNum, dimen] = np.shape(lmin)
    x = current[0]
    y = current[1]
    open_list = [[x, y, 0, field[x][y], 0+field[x][y], -1]]
    closed = np.ones(np.shape(field), dtype=np.int)
    closedlist = []
    max_its = 10000
    pathFound = False
    neighbor = np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
    distance_array_local = distance_array
    distance_array_local[distance_array_local < 10] = 0
    for i in range(max_its):
        if open_list != []:
            I = np.argmin(open_list, axis=0)
            n = open_list[I[4]]
            open_list.pop(I[4])
            lm_dist = [np.linalg.norm(lmin[j] - n[0:2]) for j in range(lminNum)]
            lm_I = np.argmin(lm_dist)
            Xtp = lmin[lm_I]
            d = np.linalg.norm(n[0:2] - end_coords)
            if d < 3 or field[round(n[0]), round(n[1])] <= field[round(Xtp[0]), round(Xtp[1])] - 10:
                pathFound = True
                break
            near_point = n[0:2] + neighbor
            for j in range(len(neighbor)):
                newPos = near_point[j]
                if distance_array_local[round(newPos[0]), round(newPos[1])] <= 1:
                    continue
                if closed[newPos[0]][newPos[1]] != 0:
                    historicCost = n[2] + 1
                    heuristicCost = field[newPos[0]][newPos[1]]
                    totalCost = historicCost + heuristicCost
                    add = True
                    if open_list != []:
                        Near_index = check_open_list(open_list, newPos)
                        if Near_index.size > 0:
                            I = Near_index[0]
                            if open_list[I][4] < totalCost:
                                add = False
                            else:
                                open_list.pop(I)
                    if add:
                        new_item = [newPos[0], newPos[1], historicCost, heuristicCost, totalCost, len(closedlist) - 1]
                        open_list.append(new_item)
            closed[n[0]][n[1]] = 0
            closedlist.append(n)
    if not pathFound:
        print("can't escape the local minimal")
        return None
    else:
        print("escape the local minimal")
    path = [n[0:2]]
    prev = n[5]
    while prev > 0:
        next_point = closedlist[prev][0:2]
        path.append(next_point)
        prev = closedlist[prev][5]
        print("hahahha")
    route = []
    for i in range(len(path)):
        route.append(path[-(1 + i)])
    print("finish the Astar algorithm")
    return route

