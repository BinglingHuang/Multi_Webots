import numpy as np
from scipy import ndimage
# a = 123
# b  = isinstance(a, int)
# print(b)
# X_value = np.arange(0, output.shape[0], 1)
# Y_value = np.arange(0, output.shape[1], 1)
# X, Y = np.meshgrid(X_value, Y_value)


dx = np.arange(-2, 3, 1)
dy = np.arange(-2, 3, 1)
X, Y = np.meshgrid(dx,dy)
array = []
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        if(X[i][j] == 2 or Y[i][j] == 2 or X[i][j] == -2 or Y[i][j] == -2):
            array.append([X[i][j], Y[i][j]])
print(array)
def distance_orientationR(point_coord, b, original_point, index, E_distance):
    Xmax = b.shape[0]
    Ymax = b.shape[1]
    dx = np.array([-1, 0, 1, 1, 1, 0, -1, -1])
    dy = np.array([1, 1, 1, 0, -1, -1, -1, 0, ])
    for i in range(8):
        newX = point_coord[0] + dx[i]
        newY = point_coord[1] + dy[i]
        print("newX: ", newX)
        print("newY: ", newY)
        if(newX >= 0) and (newX < Xmax) and (newY >= 0) and (newY < Ymax):
            if (b[newX][newY] == 0):
                E_distance_new = np.sqrt((original_point[0]-newX)**2 + (original_point[1]-newY)**2)
                print("E_distanace: ", E_distance_new)
                if E_distance == E_distance_new:
                    index.append([newX, newY])
                    return index
            # point_coord = np.array([newX, newY])
            # return distance_orientationR(point_coord, b, original_point, index, E_distance)
        else:
            print("out_of_index: ", [newX, newY])
            # return


if __name__ == '__main__':

    A = np.array([
        [0, 0, 1, 1, 1],
        [1, 1, 1, 1, 1],
        [0, 1, 1, 1, 0],
        [1, 0, 1, 0, 1],
        [0, 1, 1, 1, 1],
      ])
    b = ndimage.distance_transform_edt(A)
    x = 1
    y = 0
    print(b)
    print(b[x][y])
    a = False
    tmp = int(b[x][y])
    if(tmp == b[x][y]):
        R = tmp
        a = True
    # else:
    #
    # print(a)
    original_point = np.array([x,y])
    point_coord = np.array([x,y])
    index = []
    E_distance = b[x][y]
    index = distance_orientationR(point_coord, b, original_point, index, E_distance)
    print(index)
    # Xmax = b.shape[0]
    # Ymax = b.shape[1]

f = x**2 + y**2
g = np.gradient(f)
print(g)
