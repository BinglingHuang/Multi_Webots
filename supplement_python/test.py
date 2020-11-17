import numpy as np
import camera_Image
# def check_open_list(open_list, newPos):
#     # print(open_list)
#     par = np.array(open_list)
#     # print(par)
#     par = par[:, 0:2]
#     c = np.sum(abs(par - newPos), axis=1)
#     Near_index = np.where(c == 0)
#     return Near_index[0]
#
# a = np.array([6])
# if a.size > 0:
#     print(a)
# step = int(180/36)
# a = range(-90, 90, step)
# print(a[18])
# path = 'D:/convolutional3DOF_field/Python/Python/Environment1.png'
# blackAndWhite = camera_Image.transfer_to_baw_image(path)
# output = camera_Image.resize(blackAndWhite, 25)
# output = camera_Image.resize(output, 25)
# # shape = [output.shape[0], output.shape[1]]
# # W = np.array([[0] * shape[0]] * shape[1])
# output[output == 0] = 1
# output[output > 1] = 0
# # -------------------------------------free space and make fourier transform-------------
# W = output

for i in range(-4,5):
    print(i)
