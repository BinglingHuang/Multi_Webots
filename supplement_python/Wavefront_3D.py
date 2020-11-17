
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def wavefrontAlgothm(goal, logitMap):
    zaxis = logitMap.shape[0]
    yaxis = logitMap.shape[1]
    xaxis = logitMap.shape[2]
    cost = np.zeros((zaxis, xaxis, yaxis))
    goali = goal[0]
    goalj = goal[1]
    goalk = goal[2]
    open = np.array([0, 0, 0])
    open = np.row_stack((open, [goali, goalj, goalk]))
    cost[goalk][goali][goalj] = 1
    adjacent = np.array([[1,0,0],[-1,0,0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0,0,-1]])
    times = 0
    while(open.shape[0] !=1):
        for k in range (adjacent.shape[0]):
            adj = open[1] + adjacent[k]
            if min(adj) < 0:
                continue
            if adj[0] >= xaxis:
                continue
            if adj[1] >= yaxis:
                continue
            if adj[2] >= zaxis:
                continue
            if logitMap[adj[2]][adj[0]][adj[1]] == 1:
                continue
            if cost[adj[2]][adj[0]][adj[1]] !=0:
                continue
            cost[adj[2]][adj[0]][adj[1]] = cost[open[1][2]][open[1][0]][open[1][1]] + 1
            open = np.row_stack((open, adj))
        open = np.delete(open, 1, axis=0)
        times = times +1

# print the 3D picture for cost matrix
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # x = np.arange(0, cost.shape[1],1)
    # y = np.arange(0, cost.shape[2], 1)
    # xdata, ydata = np.meshgrid(x,y)
    # zdata = cost[0, :, :]
    # ax.plot_surface(xdata,ydata,zdata, cmap='Blues')
    # plt.show()
    return cost






