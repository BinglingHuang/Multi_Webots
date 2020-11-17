import numpy as np
from camera_Image import *
from scipy.signal import argrelextrema
import cv2
import random
import scipy.ndimage.filters as filters
import scipy.ndimage.morphology as morphology
import conv
from A_star_path_planning import *
import sys
import math

start_coords = np.array([30, 60])
end_coords = np.array([120, 20])
max_its = 3000
point_size = 1
point_color_start = (1, 1, 1)
point_color_end = (0, 0, 255)
point_color_path = (0, 255, 255)
thickness = 4
SIZE = 2
step = int(180/36)
a = range(-90, 90, step)

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
    return f

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
        if distance_array[int(round(position[0])), int(round(position[1]))] <= 10:
            detected_flag = 1
            break
    return detected_flag


def improved_path(visited_point, distance_array):
    if len(visited_point) == 1:
        return visited_point
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
    visited_point = np.empty(shape=[0, 2], dtype='int64')
    distance_array_local = distance_array
    distance_array_local[distance_array_local < 10] = 0
    for i in range(max_its):
        x = int(round(current[0]))
        y = int(round(current[1]))
        g = [gx[x, y], gy[x, y]]
        lm_dist = [np.linalg.norm(lmin[j] - current) for j in range(lminNum)]
        if min(lm_dist) > 3:
            a = 1
            ncoords = current + g / np.linalg.norm(g)
            ncoords = np.around(ncoords).astype(int)
            current = ncoords
            route.append(list(ncoords))
            cv2.circle(image_environment, tuple([ncoords[1] * 4, ncoords[0] * 4]), point_size, point_color_start, thickness)
        else:
            a = 0
            ncoords = Astar_path_planner(field, lmin, current, distance_array, end_coords)
            if ncoords is None:
                print("failed to find the path")
                sys.exit(0)
            print("original path = ", ncoords)
            ncoords = improved_path(np.array(ncoords), distance_array)
            for j in range(len(ncoords)):
                route.append(list(ncoords[j]))  ## route is a list now
            print("improve path is =", ncoords)
            current = ncoords[-1]
            # print("hahaha")
        d = np.linalg.norm(current - end_coords)
        if d < 3:
            print("successfully found the path")
            break
    return route

def routeInPicture(route):
    route = np.array(route)
    route_len = route.shape[0]
    size_route = round((route_len - 1) / 3)
    Pictureroute = []
    for i in range(1, 3):
        route_eachIndex = int(i * size_route)
        Pictureroute.append(route[route_eachIndex])
    Pictureroute.append(end_coords)
    return Pictureroute

def pic_to_conv(route, distance_array, C_space):
    route = np.array(route)
    distance_array_shape = distance_array.shape
    C_space_shape = C_space.shape
    route = route / 4
    route = np.round(route)
    return route


if __name__ == '__main__':
    path = 'D:/convolutional3DOF_field/Python/Python/Environment1.png'
    image_environment = cv2.imread(path)
    image = transfer_to_baw_image(path)
    field, att, rep, distance_array = field_creation(path, end_coords)
    lmin = find2D_array_localminimum(field, end_coords)
    route = gradient_based_planner(field, att, start_coords, end_coords, max_its, distance_array)
    # route = improved_path(np.array(route), distance_array)
    # routeWebots = route_webots(field, att, start_coords, end_coords, max_its, distance_array)
    # for point in route:
    #     cv2.circle(image_environment, tuple([point[1]*4, point[0]*4]), point_size, point_color_path, thickness)
    # cv2.circle(image_environment, tuple([end_coords[1]*4, end_coords[0]*4]), point_size, point_color_end, thickness*3)
    # cv2.circle(image_environment, tuple([start_coords[1] * 4, start_coords[0] * 4]), point_size, point_color_start, thickness * 3)
    # cv2.imshow('Environment_path', image_environment)
    # cv2.imwrite("Environment_path.png", image_environment)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    C_space = conv.cspaceCreate()
    ThreeD_end_coords = np.array([int(end_coords[0]/4), int(end_coords[1]/4), 18])
    field = conv.ThreeD_field(ThreeD_end_coords) # /C_space: 25 scale route: /4 scale
    ThreeD_route = []
    print(field[:, 20, 17])
    for i in range(len(route)):
        x = int(route[i][0] / 4)
        y = int(route[i][1] / 4)
        min_p_i = np.argmin(field[:, x, y])
        ThreeD_route.append([route[i][0], route[i][1], min_p_i])

    for point in ThreeD_route:
        x = point[1]
        y = point[0]
        theta = a[point[2]] * math.pi / 180
        line_s = [int(x + 3 * math.cos(theta)), int(y + 3 * math.sin(theta))]
        line_e = [int(x - 3 * math.cos(theta)), int(y - 3 * math.sin(theta))]
        cv2.line(image_environment, tuple([line_s[0]*4, line_s[1]*4]), tuple([line_e[0]*4, line_e[1]*4]), (134, 2, 34), 1)
        cv2.circle(image_environment, tuple([point[1] * 4, point[0] * 4]), point_size, point_color_path, thickness)
    cv2.circle(image_environment, tuple([end_coords[1] * 4, end_coords[0] * 4]), point_size, point_color_end,
               thickness * 3)
    cv2.circle(image_environment, tuple([start_coords[1] * 4, start_coords[0] * 4]), point_size, point_color_start,
               thickness * 3)
    cv2.imshow('Environment_path', image_environment)
    cv2.imwrite("Environment_path.png", image_environment)
    cv2.waitKey(0)
    cv2.destroyAllWindows()






