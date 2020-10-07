#
from ismember import ismember
import cv2
# from scipy import ndimage
# import numpy as np
# import math
# from calculation import *
# # a = np.array(([0,0,1,0,0],
# #               [1,0,1,1,1],
# #               [0,1,1,0,1],
# #               [1,1,1,1,0],
# #               [0,1,1,0,1]))
# # # 0 is black
# # print(b)
# import cv2
# import numpy as np
# RESOLUTION = 640
# SIZE = 2
# ReprMeter = 160/2
# Box_position = [-0.6128389187331906, 0.0999215713821463, -0.6081486313921486, -0.00032564802520107023]
# vertexs = boxVertexs(Box_position, SIZE)
# A_point = (int(round((vertexs[0][0]+SIZE/2)*ReprMeter)), int(round((vertexs[0][1]+SIZE/2)*ReprMeter)))
# C_point = (int(round((vertexs[3][0]+SIZE/2)*ReprMeter)), int(round((vertexs[3][1]+SIZE/2)*ReprMeter)))
# print(vertexs[0][1]+SIZE/2)
# print(A_point[1])
#
#
#
# img_path = "Environment.png"
# img = cv2.imread(img_path)
# # cv2.imshow("img", img)
# img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
# # cv2.imshow("img_gray", img_gray)
# cv2.imwrite("img.png", img_gray)
# (thresh, blackAndWhiteImage) = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)
# image = blackAndWhiteImage
# # cv2.imshow("Black white image", blackAndWhiteImage)
# cv2.imwrite("Black_white_image.png", blackAndWhiteImage)
# BlackWhite_Path = "Black_white_image.png"
# map = cv2.imread(BlackWhite_Path)
# cv2.rectangle(map, (80,0), (80,0),(0,0,255),2)
# cv2.imwrite("2.jpg", map)
# cv2.imshow('2.jpg', map)
# cv2.waitKey()
#

# import numpy as np
# from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# from matplotlib import cm
# fig = plt.figure()
# ax3 = plt.axes(projection='3d')
# a = np.eye(4)
# x = np.arange(0, 4, 1)
# y = np.arange(0, 4, 1)
# xx1, yy1 = np.meshgrid(x, y )
# goal = [3, 3]
# b = (xx1 - goal[0])**2 + (yy1 - goal[0])**2
# ax3.plot_surface(xx1,yy1,b,rstride = 1, cstride =1,cmap=cm.cool)
# plt.show()
import numpy as np
def ismember(A, B):
    return np.array([(a == B).all() for a in A]).any()





# a = np.array([1.4,2.7])
# # c = np.array(3,4)
# b = np.around(a).astype(int)
# print(b)
# m = a.shape[0]
# print(m)
# n = a.shape[1]
# print(n)

# a = np.array([[1,2],[2,3],[3,4]])
# b = np.array([[0,0]])
# print(a[0])
# b = np.append(b, np.array([a[0]]), axis = 0)
# print(b)
#
# x = np.empty(shape=[0, 2], dtype = 'int64')
# print(x.any())
# a = np.array([[1,2],[3,4]])
# print(a.any())
#
# b = 4536563\
#     -5646
# print(b)
#
# a = []
# a.append([3,4])
# print(a)
# n = [[[4,5],[5,4],[5,6]]]
#
# print(type(a))
# print(x)
# x = np.append(x, np.array([a[0]]), axis = 0)
# print(x)
# x = np.append(x, np.array([a[1]]), axis = 0)
# print(x)


# # print(a[-1])
# visited_point = np.array([[1,2],[3,4],[5,6],[3,6]])
# # visited_point = np.empty(shape=[0, 2], dtype = 'int64')
# b = np.array([[2,3],[3,4]])
# print(b[1]==visited_point[1])
# # if visited_point.any() and ismember(visited_point, b[1]):
# #     print("contuine")
# # else:
# #     print("keep going")
# s = []
# s.append(list(visited_point[0]))
# s.append(list(visited_point[1]))
# print(s)
# print(list(s[0]))
# print(type(s))
# print(type(s[0]))

# point_color = (0, 0, 255)
# point_color_local = (1,1,1)
# point_size = 1
# thickness = 4
# a = [61,40]
# b = [61,24]
# c = [105,45]
# d = [124,29]
# e = [61, 21]
# f = [120, 20]
# g = [123,29]
# image_environment = cv2.imread("Environment.png")
# cv2.circle(image_environment, tuple([a[1]*4, a[0]*4]), point_size, point_color, thickness*3)
# cv2.circle(image_environment, tuple([b[1] * 4, b[0] * 4]), point_size, point_color, thickness * 3)
# cv2.circle(image_environment, tuple([c[1]*4, c[0]*4]), point_size, point_color, thickness*3)
# cv2.circle(image_environment, tuple([d[1] * 4, d[0] * 4]), point_size, point_color, thickness * 3)
# cv2.circle(image_environment, tuple([e[1]*4, e[0]*4]), point_size, point_color_local, thickness*3)
# cv2.circle(image_environment, tuple([f[1] * 4, f[0] * 4]), point_size, point_color_local, thickness * 3)
# cv2.circle(image_environment, tuple([g[1]*4, g[0]*4]), point_size, point_color_local, thickness*3)
# cv2.imshow('Environment_path', image_environment)
# cv2.imwrite("Environment_path_1.png", image_environment)
# cv2.waitKey(0)
# visited_point = np.array([[0,3,4],[4,0,3],[3,5,6]])
# print(visited_point)
# x,y = np.where(visited_point == 0)
# print(x.shape[0])
# a = 0
# b = 1
# print(a!=b)
a = np.array([1,2])
a = a[[1,0]]
print(a)

# for i in range(10):
#     print(i," ")
# route = np.array([[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9]])
#
# # route = route[:, [1, 0]]
# print(route)
# route_len = route.shape[0]
# size_route = round((route_len-1) / 3)
# print(size_route)
# for i in range(1, 3):
#     print("i: ", i)
#     route_eachIndex = i * size_route
#     print("route_each: ", route_eachIndex)
#     print(route[route_eachIndex])
# print(route[route_len-1])
#
# route = np.array([[1,2,4],[2,3,3],[3,4,6],[4,5,8],[5,6,2],[6,7,0],[7,8,4],[8,9,3]])
# a = np.array([1,2,3])
# a = a[[1, 0]]
# print(a)
# route = route[:, [1, 0]]
# print(route)

# a = [[0,0,0]] * 10
# print(a)

from collections import Counter
# a = np.array([[1,2],[3,4],[1,2]])
# print(Counter(a))
#
# for i in range(10):  ###右移
#     A.insert(0, A.pop())
#
# a = [[0, 0, 0]] * 10
# a.insert(0, [1, 2, 3])
# a.pop()
# print(a)

# import random
# b = random.uniform(-1,1)
# print(b)
# a = 6.18 * random.uniform(-1,1)
# print(a)

a = np.[2,4,5]
b = [0,0,0]
print(a.all() == b)
np.array(position_list[9]).any() != [0]:



