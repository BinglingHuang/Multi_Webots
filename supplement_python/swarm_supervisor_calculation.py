import numpy as np
import math
import struct

def distance_position(position_A, position_B):
    distance_twoPosition = np.sqrt(pow((position_A[0] - position_B[0]), 2) + pow((position_A[1] - position_B[1]), 2))
    return distance_twoPosition


def distance_target(box_position, BOX_D, target_position):
    A_position = [box_position[0] + BOX_D / 2 * np.cos(box_position[3]),
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

def boxVertexs(box_position, BOX_D):
    line_point = []
    for i in range(4):   #calculation point of the box
        line_point.append([box_position[0] + BOX_D * math.sqrt(2) / 2 * np.cos(box_position[2] + (i - 1) * math.pi / 2 + math.pi / 4),
                           box_position[1] - BOX_D * math.sqrt(2) / 2 * np.sin(box_position[2] + (i - 1) * math.pi / 2 + math.pi / 4)])
    return line_point

def linesFunctions(boxVertex):
    A_point = boxVertex[0]
    B_point = boxVertex[1]
    C_point = boxVertex[2]
    D_point = boxVertex[3]
    line1 = (x- A_point[0])/(B_point[0]-A_point[0])*(B_point[1]-A_point[1])+A_point[1]  #AB line
    line2 = (x- B_point[0])/(C_point[0]-B_point[0])*(C_point[1]-B_point[1])+B_point[1]  #BC line
    line3 = (x- C_point[0])/(D_point[0]-C_point[0])*(D_point[1]-C_point[1])+C_point[1]  #CD line
    line4 = (x- D_point[0])/(A_point[0]-D_point[0])*(A_point[1]-D_point[1])+D_point[1]  #DA line
    return line1, line2, line3, line4

def line_pointssum(A_point, B_point, map_array):
    A_X = A_point[0]
    A_Y = A_point[1]
    B_X = B_point[0]
    B_Y = B_point[1]
    # print("A ", A_point)
    # print("B ", B_point)
    nums = 10
    X_dis = (B_X-A_X)/(nums-1)
    Y_dis = (B_Y-A_Y)/(nums-1)
    AX_Integer = int(round(A_X))
    Ay_Integer = int(round(A_Y))
    BX_Integer = int(round(B_X))
    By_Integer = int(round(B_Y))
    sumValue = 0
    # sumValue = map_array[AX_Integer][Ay_Integer] + map_array[BX_Integer][By_Integer]
    for i in range(0, nums):
        x = int(round(A_X + i * X_dis))
        y = int(round(A_Y + i * Y_dis))
        sumValue = sumValue + map_array[x][y]
    return sumValue



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

def bytesToFloat(h1,h2,h3,h4):
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!f",ba)[0]

def floatToBytes(f):
    bs = struct.pack("f",f)
    return (bs[3],bs[2],bs[1],bs[0])


def webots_to_map_coordination_transfrom(coords_we, size, ReprMeter):
    coords_we = np.array(coords_we)
    coords_we = (coords_we + size/2) * ReprMeter
    m = np.shape(coords_we)
    if len(m) == 1:
        coords_map = coords_we[[1, 0]]
    else:
        coords_map = coords_we[:, [1, 0]]
    return coords_map


# def map_