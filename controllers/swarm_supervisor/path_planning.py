import numpy as np
from camera_Image import *
from scipy.signal import argrelextrema
import random

start_coords = [30, 30]
end_coords = [120, 40]
max_its = 2000


def find2D_array_localminimum(y, end_coords):
    b = list(argrelextrema(y, np.less, axis=1))
    c = list(argrelextrema(y, np.less, axis=0))
    d = []
    e = []
    g = []
    for i in range(len(b[0])):
        d.append([b[0][i], b[1][i]])
    for i in range(len(c[0])):
        e.append([c[0][i], c[1][i]])
    f = np.array([x for x in d if x in e])
    for x in f - end_coords:
        g.append(np.linalg.norm(x))
    f = np.delete(f, np.argmin(g), 0)
    return f

def ismember(A, B):
    return np.array([(a == B).all() for a in A]).any()

def interpolation(S):
    shape = np.shape(S)
    path_improved = S[1]
    for i in range(shape[0]-1):
        step_size = int(round(np.linalg.norm(S[i+1]-S[i])))
        for j in range(step_size):
            position = (1 - j/step_size)*np.array(S[i]) + (j/step_size) * np.array(S[i+1])
            position = list(np.around(position).astype(int))
            path_improved.append(position)
    return path_improved

def detected_obstacle(start_p, end_p, distance_array):
    detected_flag = 0
    start_p = np.array(start_p)
    end_p = np.array(end_p)
    step_size = int(round(np.linalg.norm(end_p - start_p)))
    for i in range(step_size):
        position = (1 - i/step_size)*start_p + (i/step_size) * end_p
        if distance_array(int(round(position[0])), int(round(position[1]))) <=1:
            detected_flag =1
            break
    return detected_flag

def improved_path(visited_point, distance_array):
    S = []
    S_end = visited_point[-1]
    S.append(visited_point[0])
    Num_point = np.shape(visited_point)
    for k in range(1, Num_point[0]):
        if not detected_obstacle(S[-1], visited_point[k], distance_array):
            continue
        else:
            S.append(visited_point[k])
    S.append(S_end)
    path_improved = interpolation(S)
    return path_improved








def gradient_based_planner(field, start_coords, end_coords, max_its, distance_array):
    route = []
    current = np.array(start_coords)
    route.append(list(current))
    [gx, gy] = np.gradient(-1*field)
    local_minimum_flag = 0
    lmin = find2D_array_localminimum(field, end_coords)
    print(lmin)
    [lminNum, dimen] = np.shape(lmin)
    t0 = 1000
    t = t0
    tmin = 1e-5
    flag = 1
    visited_point = np.empty(shape=[0, 2], dtype = 'int64')
    for i in range(max_its):
        print(i, " times")
        x = int(round(current[0]))
        y = int(round(current[1]))
        # print([x, y])
        g = [gx[x, y], gy[x, y]]
        lm_dist = [np.linalg.norm(lmin[j] - current) for j in range(lminNum)]
        # if flag == 1:
        #     g1 = [gx[29,120], gy[29,121]]
        #     print("g: ",g1)
        #     print("norm_g: ", np.linalg.norm(g1))
        #     flag = 0
        if (np.linalg.norm(g) > 0.1 or min(lm_dist) > 3) and local_minimum_flag == 0:
            ncoords = current + g / np.linalg.norm(g)
            ncoords = np.around(ncoords).astype(int)
            current = ncoords
            route.append(list(ncoords))
        else:
            local_minimum_flag = 1
            Index = np.argmin(lm_dist)
            Xtp = lmin[Index]
            near_point = current + np.array([[1,0],[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]])
            length = near_point.shape
            new_near_point = np.empty(shape = [0, 2], dtype = 'int64')
            for index in range(length[0]):
                if distance_array[int(round(near_point[index][0])), int(round(near_point[index][1]))] <=1:
                    continue
                if visited_point.any() and ismember(visited_point, near_point):
                    print("a")
                    continue
                new_near_point = np.append(new_near_point, np.array([near_point[index]]), axis = 0)

            length = new_near_point.shape
            pmin = 1000
            index_min = -1
            for index in range(length[0]):
                potential = field[int(round(new_near_point[index][0])), int(round(new_near_point[index][1]))]
                if potential < pmin:
                    pmin = potential
                    index_min = index
            if index_min == -1:
                print("can't find the path")
                break
            else:
                visited_point = np.append(visited_point, np.array([new_near_point[index_min]]), axis = 0)
            new_ncoords = new_near_point[index_min]
            delta = field[int(round(new_ncoords[0])), int(round(new_ncoords[1]))]\
                - field[int(round(current[0])), int(round(current[1]))]
            if delta >= 0:
                p = np.exp(-1 * delta / t)
                if np.random.rand(1) < p:
                    t = t * 0.99
                else:
                    visited_point = np.delete(visited_point, -1, 0)
                    print(visited_point)
                    print("drop the new coordinates")
                    continue
            else:
                t = t * 0.99
            if field[int(round(new_ncoords[0])), int(round(new_ncoords[1]))] < field[int(round(Xtp[0])), int(round(Xtp[1]))] \
                    or t < tmin:
                print("escape the minimum")
                local_minimum_flag = 0
                ncoords = improved_path(visited_point, distance_array)
                # print(visited_point)
                visited_point = np.empty(shape=[0, 2], dtype = 'int64')
                for j in range(len(ncoords)):
                    route.append(list(ncoords[j]))
                t = t0
            current = new_ncoords
        # if local_minimum_flag == 0:
        #     route.append(list(current))  ## route is a list now
        d = np.linalg.norm(current - end_coords)
        if d < 3:
            print("successfully found the path")
            break
    return route

if __name__ == '__main__':
    path = 'C:/Users/Administrator/PycharmProjects/APF_path planning/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
    field, distance_array = field_creation(path, end_coords)
    lmin = find2D_array_localminimum(field, end_coords)
    # print(lmin)
    route = gradient_based_planner(field, start_coords, end_coords, max_its, distance_array)
    # print(route)
    plot_transfer_to_baw_image(path)
    # plot(route)




