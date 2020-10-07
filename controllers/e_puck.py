import sys
import numpy as np
from calculation import *

def dis_calcul_ep(dictionary, E_puck_position):
    E_puck_position = E_puck_position[:2]
    obstacle = {}
    for key in dictionary.keys():
        other_e_puck_position = [dictionary[key][0], dictionary[key][2]]
        dis = distance(other_e_puck_position, E_puck_position)
        if dis < 0.003:
            obstacle[key] = dis
    return obstacle

def meetObstacle(psValues):
    right_obstacle = psValues[0] > 100.0 or psValues[1] > 100.0 or psValues[2] > 100.0
    left_obstacle = psValues[5] > 100.0 or psValues[6] > 100.0 or psValues[7] > 100.0
    leftSpeed = 0.5 * Max_speed
    rightSpeed = 0.5 * Max_speed
    if left_obstacle == True and right_obstacle == False:
        leftSpeed += 0.5 * Max_speed
        rightSpeed -= 0.5 * Max_speed
    elif right_obstacle == True and left_obstacle == False:
        leftSpeed -= 0.5 * Max_speed
        rightSpeed += 0.5 * Max_speed
    elif left_obstacle == True and right_obstacle == True:
        if psValues[3] > 100.0 and psValues[4] > 100.0:
            leftSpeed = 0
            rightSpeed = 0
        elif psValues[4] > 100.0:
            leftSpeed = (-leftSpeed) - 0.5 * Max_speed
            rightSpeed = -rightSpeed + 0.5 * Max_speed
        elif psValues[3] > 100.0:
            leftSpeed = (-leftSpeed) + 0.5 * Max_speed
            rightSpeed = (-rightSpeed) - 0.5 * Max_speed
        else:
            leftSpeed = -0.5 * Max_speed
            rightSpeed = -0.5 * Max_speed

        return leftSpeed,rightSpeed


## main function:
# obstacle_list = dis_calcul_ep(dict , point1)
# if len(obstacle_list) != 0:
#     for i in range(8):
#         psValues.append(ps[i].getValue())
#     meetObstacle(psValues)
# else:
#     field_theory.......
