import numpy as np


def wavefrontAlgothm(target, visited, logitMap):
    list = [target]
    while(len(list) != 0):
        logitMap = wavefrontAlgothm_helper(visited, logitMap, list)
        # print(logitMap)
        list.remove(list[0])
    return logitMap


def wavefrontAlgothm_helper(visited, logitMap, list):
        adj = np.array([[0,1],[0,-1], [1,0],[-1,0]])
        for a in adj:
            new = list[0]+a
            if canvisited(new, visited, logitMap):
                # print(list)
                list.append(np.array([new[0], new[1]]))
                visited[new[0]][new[1]] = True
                logitMap[new[0]][new[1]] = logitMap[list[0][0]][list[0][1]]+1
        return logitMap


def canvisited(target, visited, logitMap):
    if inRange(target, logitMap)==False or visited[target[0]][target[1]]:
        return False
    else:
        return True


def inRange(target, logitMap):
    if target[0] >= logitMap.shape[0] or target[1] >= logitMap.shape[1] or target[0] < 0 or target[1] < 0:
        return False
    else:
        return True


def wavePath(start, target, logitMap, list):
    if start[0] == target[0] and start[1] == target[1]:
        return list
    adj = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])
    min = float(10**10)
    min_index = start
    for a in adj:
        new = start + a
        if inRange(new, logitMap):
            if min > logitMap[new[0]][new[1]]:
                min = logitMap[new[0]][new[1]]
                min_index = new
    list.append(min_index)
    print("min_index: ", min_index)
    print("target: ", target)
    return wavePath(min_index, target, logitMap, list)


logitMap = np.zeros([4, 4])
logitMap_pre = np.copy(logitMap)
visited = np.zeros([4, 4], dtype=bool)
target = np.array([3, 3])
start = np.array([1, 1])
visited[target[0]][target[1]] = True
logitMap[target[0]][target[1]] = 1
logitMap = wavefrontAlgothm(target, visited, logitMap)
print(logitMap)
list = []
list = wavePath(start, target, logitMap, list)
print("list: ", list)
