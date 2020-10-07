import numpy as np
import math
import struct
def distance(position_A, position_B):
    distance = np.sqrt(pow((position_A[0] - position_B[0]), 2) + pow((position_A[1] - position_B[1]), 2))
    return distance


def distance_target(box_position, BOX_D, target_position):
    A_position = [box_position[0] + BOX_D / 2 * np.cos(box_position[3]),#center position
                  box_position[2] - BOX_D / 2 * np.sin(box_position[3])]
    B_position = [box_position[0] + BOX_D / 2 * np.cos(math.pi / 2 + box_position[3]),
                  box_position[2] - BOX_D / 2 * np.sin(math.pi / 2 + box_position[3])]
    C_position = [box_position[0] + BOX_D / 2 * np.cos(math.pi + box_position[3]),
                  box_position[2] - BOX_D / 2 * np.sin(math.pi + box_position[3])]
    D_position = [box_position[0] + BOX_D / 2 * np.cos(3 * math.pi / 2 + box_position[3]),
                  box_position[2] - BOX_D / 2 * np.sin(3 * math.pi / 2 + box_position[3])]
    box_p = [box_position[0],box_position[2]]
    A_target = distance(A_position, target_position)
    B_target = distance(B_position, target_position)
    C_target = distance(C_position, target_position)
    D_target = distance(D_position, target_position)
    Cen_target = distance(box_p, target_position)
    return [A_target, B_target, C_target, D_target, Cen_target]


def getDis(pointA, pointB, E_puck_position):
    pointX = E_puck_position[0]
    pointY = E_puck_position[2]
    lineX1 = pointA[0]
    lineX2 = pointB[0]
    lineY1 = pointA[1]
    lineY2 = pointB[1]
    a = lineY2 - lineY1
    b = lineX1 - lineX2
    c = lineX2 * lineY1 - lineX1 * lineY2
    dis = (math.fabs(a * pointX + b * pointY + c)) / (math.pow(a * a + b * b, 0.5))
    return dis

def whichside(pointA, pointB, e_puck_position, box_position):
    # (y-y1)/(y2-y1) - (x-x1)/(x2-x1)=0
    a = (e_puck_position[2]-pointA[1])*(pointB[1]-pointA[1])-(e_puck_position[0]-pointA[0])*(pointB[0]-pointA[0])
    b = (box_position[2]-pointA[1])*(pointB[1]-pointA[1])-(box_position[0]-pointA[0])*(pointB[0]-pointA[0])
    c = a*b
    return c
