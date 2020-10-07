import numpy as np
x = 2
y = 3

A = np.array([
        [0, 0, 0, 0, 2],
        [0, 0, 0, 0, 1]
      ])
# f = x**2 + y**2
gX, gY = np.gradient(1*A)
print(gX)
print(gY)
# gx1 = gX[0][0]
# gy1 = gY[0][0]
# print(gx1)
#
# a = [3, 4]
# b = np.linalg.norm(a)
# print(b)

