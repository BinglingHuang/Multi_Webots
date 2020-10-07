# from calculation import *
import numpy as np
import math
SIZE = 2
# ReprMeter = 160/SIZE
# Box_position = [0.39999997784318014, 0.0999215071962221, 0.40000003889943514, -0.0003256490511217354]
# vertexs = np.array(boxVertexs(Box_position, SIZE))
# A_point = [(vertexs[0][0] + SIZE / 2) * ReprMeter, (vertexs[0][1] + SIZE / 2) * ReprMeter]
# B_point = [(vertexs[1][0] + SIZE / 2) * ReprMeter, (vertexs[1][1] + SIZE / 2) * ReprMeter]
# C_point = [(vertexs[2][0] + SIZE / 2) * ReprMeter, (vertexs[2][1] + SIZE / 2) * ReprMeter]
# D_point = [(vertexs[3][0] + SIZE / 2) * ReprMeter, (vertexs[3][1] + SIZE / 2) * ReprMeter]

# line1Value = line1_pointssum(A_point, B_point, X,Y)
# print(A_point)
# print(B_point)
# print(C_point)
# print(D_point)
# boxVertex = (vertexs + SIZE/2) * ReprMeter
# print(boxVertex)
# # line1, line2, line3, line4 = linesFunctions(boxVertex)
# x = np.arange(0, 5, 1)
# y = np.arange(0, 5, 1)
# xx1, yy1 = np.meshgrid(x, y )
# c = xx1*2-4*yy1
# print(c)
# X3 = 3
# # cons = (xx1>2) & (yy1>3)
# # value = c[cons]
# e = (c>0)
# value = np.where(e, c, 0)
# print(c)
# print(value)
# print(value)
# value = np.where(e)
# x1 = value[0]
# x2 = value[1]
# x1 = np.delete(x1,1)
#
# print(x1)
# print(x2)
# sum = 0
# for i in range(x1.size):
#     sum = sum + c[x1[i]][x2[i]]
# print(sum)
# print(c[x1[1]][x2[1]])
# x_1, y_1 = np.meshgrid(x1,x2)
# print(x_1)
# print(y_1)
# print(value2[0])
# print(value2[1])
# A_point = [4,4]
# B_point = [6,4]
# print(max(A_point[0], B_point[0]))
# print(min(A_point[0], B_point[0]))
# print(math.sqrt(2))
# print(10**-2)
# X_max = int(round(1))
# print(X_max)
# print(type(X_max))
# x = abs(-1)
# print(x)
#
# b = np.array([[2,3],
#      [4,5],
#      [6,7]])
# # b.reverse()
# # b[0][1], b[0][0] = b[0][0], b[0][1]
# b = b[:,[1,0]]
# print(b)
# print(10**-7)

# def fun(x):
#     y = (Y1*(X2-X1)+(x-X1)*(Y2-Y1))/(X2-X1)
#     return y
#
ax = 10
ay = 10
bx = 40
by = 10
N = 4
A = (bx-ax)/N
B = (by-ay)/N
for i in range(0,N+1):
    print('x%d = %5.3f'%(i,(ax+i*A)), 'y%d = %5.3f'%(i, (ay+i*A)))

# # a = 1.34
# # print(type(round(a)))
# boxVertex = np.array([1,2])
# boxVertex = boxVertex[[1, 0]]
# print(boxVertex)
a = np.arange(0,5,1)
b = np.arange(0,5,1)
c = np.meshgrid(a, b)
print(c)
