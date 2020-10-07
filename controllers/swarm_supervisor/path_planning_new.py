import numpy as np
from camera_Image import *
from scipy.signal import argrelextrema
import cv2
import random
import scipy.ndimage.filters as filters
import scipy.ndimage.morphology as morphology

start_coords = np.array([60, 40])
end_coords = np.array([120, 20])
max_its = 3000
point_size = 1
point_color_start = (1,1,1)
point_color_end = (0, 0, 255)
thickness = 4
SIZE = 2


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
    # f = np.delete(f, np.argmin(g), 0)
    return f


# def find2D_array_localminimum(arr, end_coords):
#     neighborhood = morphology.generate_binary_structure(len(arr.shape), 2)
#     # print("neighborhood: ", neighborhood)
#     local_min = (filters.minimum_filter(arr, footprint=neighborhood)==arr)
#     # print("local_min: ", local_min)
#     detected_minima = local_min
#     local_minima_location = np.where(detected_minima)
#     size = np.shape(local_minima_location)
#     print(size)
#     list = []
#     for i in range(size[1]):
#         list.append([local_minima_location[0][i], local_minima_location[1][i]])
#     list = np.array(list)
#     return list




def ismember(A, B):
    return np.array([(a == B).all() for a in A]).any()

def interpolation(S):
    path_improved = []
    shape = np.shape(S)
    # path_improved.append(S[0])
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
        if distance_array[int(round(position[0])), int(round(position[1]))] <= 1:
            detected_flag = 1
            break
    return detected_flag

def improved_path(visited_point, distance_array):
    S = []
    S_end = visited_point[-1]
    S.append(visited_point[0])
    Num_point = np.shape(visited_point)
    action_point = []
    for k in range(1, Num_point[0]):
        if not detected_obstacle(S[-1], visited_point[k], distance_array):
            continue
        else:
            S.append(visited_point[k])
    S.append(S_end)
    path_improved = interpolation(S)
    return path_improved

def gradient_based_planner(field, att, start_coords, end_coords, max_its, distance_array):
    start_coords = np.array([int(round(start_coords[0])), int(round(start_coords[1]))])
    end_coords = np.array([int(round(end_coords[0])), int(round(end_coords[1]))])
    route = []
    current = start_coords
    route.append(list(current))
    [gx, gy] = np.gradient(-1*field)
    local_minimum_flag = 0
    lmin = find2D_array_localminimum(field, end_coords)
    [lminNum, dimen] = np.shape(lmin)
    # for i in range(lminNum):
    visted_point_total = []
    print(lmin)
    t0 = 1000
    t = t0
    flag = 1
    tmin = 1e-5
    visited_point = np.empty(shape=[0, 2], dtype='int64')
    # visited_point_history = np.empty(shape=[0, 2], dtype='int64')
    distance_array_local = distance_array
    distance_array_local[distance_array_local < 17] = 0
    step_length = 0
    for i in range(max_its):
        x = int(round(current[0]))
        y = int(round(current[1]))
        g = [gx[x, y], gy[x, y]]
        # if flag == 1:
        #     print(g[27,117])
        #     print(g[29,117])
        #     print(g[30,117])
        #     flag = 0
        lm_dist = [np.linalg.norm(lmin[j] - current) for j in range(lminNum)]
        # if (np.linalg.norm(g) > 0.1 or min(lm_dist) > 3) and local_minimum_flag == 0:
        if min(lm_dist) > 3 and local_minimum_flag == 0:
            # print("gradient: ", current)
            ncoords = current + g / np.linalg.norm(g)
            ncoords = np.around(ncoords).astype(int)
            current = ncoords
            route.append(list(ncoords))
            # visited_point_history = np.append(visited_point_history, np.array([current]), axis=0)
        else:
            if local_minimum_flag==0:
                Index = np.argmin(lm_dist)
                Xtp = lmin[Index]
            local_minimum_flag = 1
            # Index = np.argmin(lm_dist)
            # Xtp = lmin[Index]
            near_point = current + np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
            length = near_point.shape
            new_near_point = np.empty(shape=[0, 2], dtype='int64')
            for index in range(length[0]):
                if distance_array_local[int(round(near_point[index][0])), int(round(near_point[index][1]))] <= 1:
                    continue
                if visited_point.any() and ismember(visited_point, near_point[index]):
                    continue
                new_near_point = np.append(new_near_point, np.array([near_point[index]]), axis=0)
            length = new_near_point.shape
            pmin = 1000
            index_min = -1
            for index in range(length[0]):
                potential = att[int(round(new_near_point[index][0])), int(round(new_near_point[index][1]))]
                if potential < pmin:
                    pmin = potential
                    index_min = index
            if index_min == -1:
                print("can't find the path")
                break
            else:
                visited_point = np.append(visited_point, np.array([new_near_point[index_min]]), axis=0)
                # visited_point_history = np.append(visited_point_history, np.array([new_near_point[index_min]]), axis=0)
            new_ncoords = new_near_point[index_min]
            delta = field[int(round(new_ncoords[0])), int(round(new_ncoords[1]))]\
                - field[int(round(current[0])), int(round(current[1]))]
            if delta >= 0:
                p = np.exp(-1 * delta / t)
                random_num = np.random.rand(1)
                # print(t)
                if random_num < p:
                    t = t * 0.98
                else:
                    visited_point = np.delete(visited_point, -1, 0)
                    # print(visited_point)
                    # print("p: ", p)
                    # print("random_num: ", random_num)
                    print("in the ", i, "interation , drop the new coordinates")
                    continue
            else:
                t = t * 0.98
            if field[int(round(new_ncoords[0])), int(round(new_ncoords[1]))] <= field[int(round(Xtp[0])), int(round(Xtp[1]))] \
                    or t < tmin or (new_ncoords == end_coords).all():
                print("escape the minimum")
                local_minimum_flag = 0
                ncoords = visited_point
                visited_point = np.empty(shape=[0, 2], dtype='int64')
                for j in range(len(ncoords)):
                    route.append(list(ncoords[j]))  ## route is a list now
                t = t0
            current = new_ncoords
            # print(new_ncoords)
        d = np.linalg.norm(current - end_coords)
        if d < 1:
            local_minimum_flag = 0
            print("d = ", d)
            print("current = ", current)
            print("end_coords", end_coords)
            print("successfully found the path")
            break
    return route

def route_webots(field, att, start_coords, end_coords, max_its, distance_array):
    route = gradient_based_planner(field, att, start_coords, end_coords, max_its, distance_array)
    route = np.array(route)
    route = improved_path(route, distance_array)
    route = np.array(route)
    path = 'C:/Users/Administrator/PycharmProjects/APF_MSAA/APF_MSAA_PYTHON/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
    image_environment = cv2.imread(path)
    for point in route:
        cv2.circle(image_environment, tuple([point[1]*4, point[0]*4]), point_size, point_color_start, thickness)
    # cv2.circle(image_environment, tuple([end_coords[1]*4, end_coords[0]*4]), point_size, point_color_end, thickness*3)
    # cv2.circle(image_environment, tuple([start_coords[1] * 4, start_coords[0] * 4]), point_size, point_color_start, thickness * 3)
    # cv2.imshow('Environment_path_webots', image_environment)
    cv2.imwrite("Environment_path_webots.png", image_environment)
    ReprMeter = (distance_array.shape[0] - 1) / SIZE
    route = route[:, [1,0]]
    webots_route = route / ReprMeter - SIZE / 2
    end_coords = end_coords / ReprMeter - SIZE /2
    end_coords = end_coords[[1,0]]
    route_len = webots_route.shape[0]
    size_route = round((route_len-1) / 3)
    part_route = []
    for i in range(1, 3):
        route_eachIndex = int(i*size_route)
        print(route_eachIndex)
        part_route.append(webots_route[route_eachIndex])
    part_route.append(end_coords)
    # print("webots_route: ", webots_route)
    # print("part_route: ", part_route)
    return part_route





if __name__ == '__main__':
    path = 'C:/Users/Administrator/PycharmProjects/APF_MSAA/APF_MSAA_PYTHON/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
    image_environment = cv2.imread(path)
    image = transfer_to_baw_image(path)
    field, att, rep, distance_array = field_creation(path, end_coords)
    lmin = find2D_array_localminimum(field, end_coords)
    route = gradient_based_planner(field, att, start_coords, end_coords, max_its, distance_array)
    print("route: ", route)
    route = improved_path(np.array(route), distance_array)
    routeWebots = route_webots(field, att, start_coords, end_coords, max_its, distance_array)
    print("webots: ", routeWebots)
    # print(route)
    for point in route:
        cv2.circle(image_environment, tuple([point[1]*4, point[0]*4]), point_size, point_color_start, thickness)
    cv2.circle(image_environment, tuple([end_coords[1]*4, end_coords[0]*4]), point_size, point_color_end, thickness*3)
    cv2.circle(image_environment, tuple([start_coords[1] * 4, start_coords[0] * 4]), point_size, point_color_start, thickness * 3)
    cv2.imshow('Environment_path', image_environment)
    cv2.imwrite("Environment_path.png", image_environment)
    # cv2.imshow('Black white image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





