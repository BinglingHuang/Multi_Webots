"""construct the field"""
import sys
import cv2

# from field_theory_controller.field_theory_calculation import distance

sys.path.append("C:\\Users\\Administrator\\PycharmProjects\\APF_MSAA\\APF_MSAA_PYTHON\\new_obstacle_environment_version2\\controllers\\swarm_supervisor")
from field_theory_calculation import *
from scipy import integrate
import camera_Image
import swarm_supervisor_calculation
import numpy as np
import math
from scipy import ndimage

###################----------------------field_theory_scalor---------------------##########################
e_puck_D = 0.07
obstacle_D = 0.4
k = 0.8
obstacle_field_r = 0.4
ratio_K1 = 1
SIZE = 2

###################----------------------social_field_calculation---------------------##########################
def social_field(customData, e_puck_position, K4, ReprMeter, map_array, rep):
## customData dic{key:[x,y,z,yaw]} e_puck_position list [x,z,yaw] calculate the social field
    vector = []
    vector.append([0,0])
    k = K4
    obs_social_field_threshold = 5
    mu = 1
    #############----------------social field between e-puck and other e-pucks -----------###############
    for key in customData.keys():
        if key[0] == 'e':    #judge if the key is epuck
            other_e_puck_position = [customData[key][0], customData[key][2]]
            r = distance(other_e_puck_position, e_puck_position) - e_puck_D + 0.03
            x = (e_puck_position[0] - other_e_puck_position[0]) / (k * 1e3 * pow(r, 3))
            y = (e_puck_position[1] - other_e_puck_position[1]) / (k * 1e3 * pow(r, 3))
            vector.append([x, y])

    #############----------------social field between e-puck and obstacles-----------###############
    e_puck_position = (e_puck_position + SIZE / 2) * ReprMeter
    distance_array = ndimage.distance_transform_edt(map_array)
    e_puck_position_x = int(round(e_puck_position[1]))
    e_puck_position_y = int(round(e_puck_position[0]))
    distance_Betw_e_puck_obstacle = distance_array[e_puck_position_x][e_puck_position_y]
    distance_gradient_X, distance_gradient_Y =  np.gradient(-1 * rep)
    # print(distance_Betw_e_puck_obstacle)
    x = 1/30 * distance_gradient_Y[e_puck_position_x][e_puck_position_y]
    y = 1/30 * distance_gradient_X[e_puck_position_x][e_puck_position_y]
    # print("x_gradient: ", distance_gradient_X[e_puck_position_x][e_puck_position_y])
    # print("y_gradient: ", distance_gradient_Y[e_puck_position_x][e_puck_position_y])
    # print("x: ", x)
    # print("y: ", y)
    vector.append([x,y])
    # print("vector_social: ", vector)
    vector_sum = np.sum(vector, axis = 0)
    return vector_sum



def converge_line_calculation(Box_position, Box_D, Goal_position, ReprMeter, map_array):
    rep = camera_Image.repulsive(map_array, 400, 1.5, Goal_position)
    vertexs = np.array(swarm_supervisor_calculation.boxVertexs(Box_position, Box_D))
    # print(vertexs)
    boxVertex = (vertexs + SIZE / 2) * ReprMeter
    # print(boxVertex)
    boxVertex = boxVertex[:, [1, 0]]
    Goal_position = (Goal_position + SIZE / 2) * ReprMeter
    Goal_position = Goal_position[[1, 0]]
    # print(boxVertex)
    X_value = np.arange(0, map_array.shape[0], 1)
    Y_value = np.arange(0, map_array.shape[1], 1)
    Y, X = np.meshgrid(X_value, Y_value)
    att = camera_Image.attractive(map_array, Goal_position, 1 / 100)
    # print(b)
    sum = rep + att
    line1Value = swarm_supervisor_calculation.line_pointssum(boxVertex[0], boxVertex[1], sum)
    line2Value = swarm_supervisor_calculation.line_pointssum(boxVertex[1], boxVertex[2], sum)
    line3Value = swarm_supervisor_calculation.line_pointssum(boxVertex[2], boxVertex[3], sum)
    line4Value = swarm_supervisor_calculation.line_pointssum(boxVertex[3], boxVertex[0], sum)
    lineValue_list = [line1Value, line2Value, line3Value, line4Value]
    # print(lineValue_list)
    b = sorted(lineValue_list, reverse=True)
    converge_list = list(map(lambda x: x and x > b[2], lineValue_list))
    # print(converge_list)
    return converge_list


###################----------------------task_field_curve_integration_method---------------------##########################
def sumx_curve_int_x(x, k, point, e_puck_position):
    e_puck_position = e_puck_position[:2]
    y = k * (x - point[0]) + point[1]
    d = distance([x,y], e_puck_position)
    f1 = (x - e_puck_position[0]) * pow(1 + pow(k, 2), 1 / 2) / pow(d, 3)
    return f1


def sumy_curve_int_x(x, k, point, e_puck_position):
    e_puck_position = e_puck_position[:2]
    y = k * (x - point[0]) + point[1]
    d = distance([x,y], e_puck_position)
    f1 = (y - e_puck_position[1]) * pow(1 + pow(k, 2), 1 / 2) / pow(d, 3)
    return f1


def sumx_curve_int_y(y, k, point, e_puck_position):
    e_puck_position = e_puck_position[:2]
    x = 1 / k * (y - point[1]) + point[0]
    d = distance([x,y], e_puck_position)
    f1 = (x - e_puck_position[0]) * pow(1 + pow(1 / k, 2), 1 / 2) / pow(d, 3)
    return f1

def sumy_curve_int_y(y, k, point, e_puck_position):
    e_puck_position = e_puck_position[:2]
    x = 1 / k * (y - point[1]) + point[0]
    d = distance([x, y], e_puck_position)
    f1 = (y - e_puck_position[1]) * pow(1 + pow(1 / k, 2), 1 / 2) / pow(d, 3)
    return f1

def magnetic_field(e_puck_position, box_position, BOX_D, converge_list, K2, K3):
    line_point = []     #four points of the box_edge
    line_k = []         #k of the edge
    line_num = 4        #number of the box_edge
    r = []              #distance from the epuck to the center of the box_edge
    strength = []       #distance portion
    vector = []         #magnetic strength vector
    # b_p = list(map(lambda x:x*100,box_position))  #box_position (x,z,theta)
    # e_p = list(map(lambda x:x*100,e_puck_position))  #e_puck_position (x,z,theta)
    # d = BOX_D*100
    b_p = box_position
    e_p = e_puck_position
    d = BOX_D
    pi = math.pi
    eps = 1e-10
    for i in range(line_num):   #calculation point of the box
        line_point.append([b_p[0] + d * math.sqrt(2) / 2 * np.cos(b_p[2] + (i - 1) * pi / 2 + pi / 4),
        b_p[1] - d * math.sqrt(2) / 2 * np.sin(b_p[2] + (i - 1) * pi / 2 + pi / 4)])
    # print(line_point)
    for i in range(line_num):
        if i == 3:
            a = 0
        else:
            a = i+1
        line_k.append((line_point[a][1] - line_point[i][1])/(line_point[a][0] - line_point[i][0] + eps) + eps)  #calculation k of the box
        if converge_list[i] == True:
            if abs(line_k[i]) < 1e1:
                sumx,err = integrate.quad(sumx_curve_int_x, line_point[i][0], line_point[a][0], args = (line_k[i], line_point[i], e_p))
                # sumx = 2 * np.sign(line_point[a][0] - line_point[i][0]) * sumx
                sumx = K2 * np.sign(line_point[a][0] - line_point[i][0]) * sumx
                sumy,err = integrate.quad(sumy_curve_int_x, line_point[i][0], line_point[a][0], args = (line_k[i], line_point[i], e_p))
                # sumy = 2 * np.sign(line_point[a][0] - line_point[i][0]) * sumy
                sumy = K2 * np.sign(line_point[a][0] - line_point[i][0]) * sumy
            if abs(line_k[i]) >= 1e1:
                sumx,err = integrate.quad(sumx_curve_int_y, line_point[i][1], line_point[a][1], args = (line_k[i], line_point[i], e_p))
                # sumx = 2 * np.sign(line_point[a][1] - line_point[i][1]) * sumx
                sumx = K2 * np.sign(line_point[a][1] - line_point[i][1]) * sumx
                sumy,err = integrate.quad(sumy_curve_int_y, line_point[i][1], line_point[a][1], args = (line_k[i], line_point[i], e_p))
                # sumy = 2 * np.sign(line_point[a][1] - line_point[i][1]) * sumy
                sumy = K2 * np.sign(line_point[a][1] - line_point[i][1]) * sumy
        elif converge_list[i] == False:
            if abs(line_k[i]) < 1e1:
                sumx,err = integrate.quad(sumx_curve_int_x, line_point[i][0], line_point[a][0], args = (line_k[i], line_point[i], e_p))
                # sumx = -0.8 * np.sign(line_point[a][0] - line_point[i][0]) * sumx
                sumx = -1 * K3 * np.sign(line_point[a][0] - line_point[i][0]) * sumx
                sumy,err = integrate.quad(sumy_curve_int_x, line_point[i][0], line_point[a][0], args = (line_k[i], line_point[i], e_p))
                # sumy = -0.8 * np.sign(line_point[a][0] - line_point[i][0]) * sumy
                sumy = -1 * K3 * np.sign(line_point[a][0] - line_point[i][0]) * sumy
                # print('3',[sumx,sumy])
            if abs(line_k[i]) >= 1e1:
                sumx,err = integrate.quad(sumx_curve_int_y, line_point[i][1], line_point[a][1], args = (line_k[i], line_point[i], e_p))
                # sumx = -0.8 * np.sign(line_point[a][1] - line_point[i][1]) * sumx
                sumx = -1 * K3 * np.sign(line_point[a][1] - line_point[i][1]) * sumx
                sumy,err = integrate.quad(sumy_curve_int_y, line_point[i][1], line_point[a][1], args = (line_k[i], line_point[i], e_p))
                # sumy = -0.8 * np.sign(line_point[a][1] - line_point[i][1]) * sumy
                sumy = -1 * K3 * np.sign(line_point[a][1] - line_point[i][1]) * sumy
                # print('4',[sumx,sumy])
        vector.append([sumx, sumy])
    vector_sum = np.sum(vector, axis = 0)
    return vector_sum
