import numpy as np
import math
from calculation import *

def magnetic_field(e_puck_position,box_position,BOX_D,converge_list):
    line_center = []    #center of the box_edge
    mean_angle = [] #angle between the box_edge and epuck
    line_num = 4    #number of the box_edge
    r = []  #distance from the epuck to the center of the box_edge
    epuck_p = [e_puck_position[0],e_puck_position[2]]
    strength = []   #distance portion
    vector = [] #magnetic strength vector
    for i in range(line_num):
        line_center.append([box_position[0]+BOX_D / 2 * np.cos(i * math.pi / 2 + box_position[3]),
        box_position[2] - BOX_D / 2 * np.sin(i * math.pi / 2 + box_position[3])])
        mean_angle.append(math.atan2(line_center[i][1]-epuck_p[1],line_center[i][0]-epuck_p[0]))
        r.append(distance(line_center[i],epuck_p))
    for i in range(np.size(r)):   strength.append(1/pow(r[i],2))
    sum_strength = sum(strength)
    strength = list(map(lambda x :x/sum_strength,strength))
    for i in range(np.size(r)):
        if converge_list[i] == True:
            vector.append([2*strength[i]*np.cos(mean_angle[i]),2*strength[i]*np.sin(mean_angle[i])])
        else:
            vector.append([-1*strength[i]*np.cos(mean_angle[i]),-1*strength[i]*np.sin(mean_angle[i])])
    vector_sum = np.sum(vector,axis = 0)
    torward_angle = -1*math.atan2(vector_sum[1],vector_sum[0])
    torward_angle = torward_angle - math.pi/2
    return vector_sum
