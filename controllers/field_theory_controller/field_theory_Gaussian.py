import numpy as np
import math
from calculation import *
import statistics as st
import scipy.stats as scs

# def Get_distance(A_x, A_y, B_x, B_y):
#     d = abs(A_x - B_x) ** 2 + abs(A_y - B_y) ** 2
#     return math.sqrt(d)

def Gaussian_field(e_puck_position, box_position, BOX_D, converge_list):
    # box_position = (x,z,theta)
    # e_puck_position = (x,z,theta)
    line_center = []  # center of the box_edge
    mean_angle = []  # angle between the box_edge and epuck
    line_num = 4  # number of the box_edge
    r = []  # distance from the epuck to the center of the box_edge
    angle_num = 8
    ###set angle space
    N = []

    for i in range(line_num):

        # Line center list
        line_center.append(
            [
                box_position[0] + BOX_D / 2 * np.cos(i * math.pi / 2 + box_position[2]),
                box_position[1] - BOX_D / 2 * np.sin(i * math.pi / 2 + box_position[2]),
            ]
        )
        # Mean angle list
        mean_angle.append(
            math.atan2(line_center[i][1] - e_puck_position[1], line_center[i][0] - e_puck_position[0])
        )
        r.append(distance(line_center[i], e_puck_position))


    Gaussian_portion = [
        1 / (r[0]) ** 2,
        1 / (r[1]) ** 2,
        1 / (r[2]) ** 2,
        1 / (r[3]) ** 2,
    ]
    sum_portion = sum(Gaussian_portion)
    _Gaussian_portion = []
    for i in range(line_num):
        _Gaussian_portion.append(Gaussian_portion[i] % sum_portion)

    angle_normalized = []


###################################################################################
    for i in range(line_num):
        angle = []

        for j in range(angle_num):
            angle.append(-math.pi + j * math.pi / 4)
            if angle[j] < -math.pi + mean_angle[i]:
                angle[j] += 2 * math.pi
            if angle[j] > math.pi + mean_angle[i]:
                angle[j] -= 2 * math.pi

        angle_normalizedprime = scs.norm.pdf(angle, mean_angle[i], 1)
        angle_normalized = angle_normalizedprime.tolist()

        # for j in range(angle_num):

        #     angle_normalized[j] = (angle[j] - st.mean(angle)) / st.stdev(angle)

        #     angle_normalized[j] += mean_angle[i]

        if converge_list[i] == 1:
            N.append([_Gaussian_portion[i] * x for x in angle_normalized])

        if converge_list[i] == 0:
            N.append([-_Gaussian_portion[i] * x for x in angle_normalized])

#####################################################################################
    S = []
    temp = 0.0
    for j in range(len(N[0])):
        for i in range(len(N)):
            temp += N[i][j]
        S.append(temp)
        temp = 0.0

    M = max(S)
    indexprime = []
    for i, j in enumerate(S):
        if j == M:
            indexprime.append(i)
    index = indexprime[0]

    angle_selected = -math.pi + (index - 1) * math.pi / 4
    vector = [np.cos(angle_selected), np.sin(angle_selected)]
    return vector
