import numpy as np
import scipy.ndimage.filters as filters
import scipy.ndimage.morphology as morphology

def detect_local_minima(arr):
    neighborhood = morphology.generate_binary_structure(len(arr.shape), 2)
    print("neighborhood: ", neighborhood)
    local_min = (filters.minimum_filter(arr, footprint=neighborhood)==arr)
    print("local_min: ", local_min)
    detected_minima = local_min
    local_minima_location = np.where(detected_minima)
    size = np.shape(local_minima_location)
    list = []
    for i in range(size[1]):
        list.append([local_minima_location[0][i], local_minima_location[1][i]])
    return list



# def local_minima(array2d):
#     local_minima = [ index
#                      for index in np.ndindex(array2d.shape)
#                      if index[0] > 0
#                      if index[1] > 0
#                      if index[0] < array2d.shape[0] - 1
#                      if index[1] < array2d.shape[1] - 1
#                      if array2d[index] < array2d[index[0] - 1, index[1] - 1]
#                      if array2d[index] < array2d[index[0] - 1, index[1]]
#                      if array2d[index] < array2d[index[0] - 1, index[1] + 1]
#                      if array2d[index] < array2d[index[0], index[1] - 1]
#                      if array2d[index] < array2d[index[0], index[1] + 1]
#                      if array2d[index] < array2d[index[0] + 1, index[1] - 1]
#                      if array2d[index] < array2d[index[0] + 1, index[1]]
#                      if array2d[index] < array2d[index[0] + 1, index[1] + 1]
#                    ]
#     return local_minima


if __name__ == '__main__':
    arr = np.array([[1,2,3,4], [2,4,6,2], [5,2,9,7],[5,2,8,4]])
    print(arr.shape)
    print(arr)
    local_minima_location = detect_local_minima(arr)
    narray = np.array(local_minima_location)
    print(narray)
    print(type(narray))
    a = np.random.rand(1)
    print(a)
    arr = np.delete(arr, -1, 0)
    print(arr)
    # print(arr[local_minima_location])

#
# [ 61  21]
#  [ 62  20]
#  [120  26]
#  [121  27]
#  [122  28]
#  [123  29]]


