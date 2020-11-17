import camera_Image
import numpy as np
from scipy.fftpack import fft, ifft
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
from mpl_toolkits import mplot3d
import Wavefront_3D
from scipy import ndimage



#---------------------------------limit the range of coordinate points------------------
def periodic(P_new, N):
    if N[0] > P_new[0] >= 0 and N[1] > P_new[1] >= 0:
        return P_new
    if P_new[0] < 0:
        P_new[0] = P_new[0] + N[0]
    if P_new[0] >= N[0]:
        P_new[0] = P_new[0] - N[0]
    if P_new[1] < 0:
        P_new[1] = P_new[1] + N[1]
    if P_new[1] >= N[1]:
        P_new[1] = P_new[1] - N[1]
    return periodic(P_new, N)


box_length = 7
box_width = 3
Goal = np.array([7, 7, 18]) # [-pi/2, pi/2] 36 steps



def cspaceCreate():
    #-------------------------------------get the black and white picutre---------------------#
    path = 'D:/convolutional3DOF_field/Python/Python/Environment1.png'
    blackAndWhite = camera_Image.transfer_to_baw_image(path)
    output = camera_Image.resize(blackAndWhite, 25)
    output = camera_Image.resize(output, 25)
    # shape = [output.shape[0], output.shape[1]]
    # W = np.array([[0] * shape[0]] * shape[1])
    output[output == 0] = 1
    output[output > 1] = 0
    #-------------------------------------free space and make fourier transform-------------
    W = output
    W_FT = np.fft.fft2(W)
    N = W_FT.shape
    Num_degree = 36
    fix_point = np.array([0, 0])
    fix_point_T = fix_point.reshape(fix_point.shape[0], 1)
    #--------------------------------get the initial box position and make the position point in a specific limit-----------------
    A_zero_zero = np.zeros(N)
    origin_box = []
    for i in range(-1, 2):
        for j in range(-3, 4):
            index_init = np.array([i, j])
            origin_box.append(list(index_init))
            origin_init = periodic(index_init, N)
            A_zero_zero[origin_init[0]][origin_init[1]] = 1
    origin_box = np.array(origin_box)
    C_space = np.zeros([Num_degree, N[0], N[1]])
    step = int(180/Num_degree)
    times = 0
    #----------------------------------C-space convolution-----------------------------------
    for theta in range(-90, 90, step):
        A_zero_zero_theta = np.zeros(N)
        A_theta = np.zeros(N)
        [X, Y] = np.where(A_zero_zero == 1)
        X = np.array(X)
        Y = np.array(Y)
        index = np.vstack((X, Y))
        alpha = theta * math.pi /180
        R = [[math.cos(alpha), -math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]]
        Num = np.shape(X)[0]
        for i in range(Num):
            delta = origin_box[i,:] - fix_point
            delta = np.array(delta)
            delta = delta.reshape(delta.shape[0], 1)
            P_new = np.dot(R, delta) + fix_point_T
            P_new = np.round(P_new)
            P_new = P_new.astype(int)
            P_new = periodic(P_new, N)
            A_zero_zero_theta[P_new[0], P_new[1]] = 1
            P_new = periodic(-P_new, N)
            A_theta[P_new[0], P_new[1]] = 1
        A_theta_FT = np.fft.fft2(A_theta)
        FT = W_FT * A_theta_FT
        IFT = np.fft.ifft2(FT)
        IFT[IFT >= 0.9] = 1
        IFT = np.real(IFT)
        IFT = IFT.astype(int)
        C_space[times,:,:] = IFT
        times = times+1
    return C_space

    # 3D picture for C_space
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # [zdata,xdata,ydata] = np.where(C_space[:,:,:]> 0.9)
    # ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
    # ax.scatter(xdata, ydata, c=zdata, cmap='Greens')
    # plt.show()

    #2D picture for C_space

    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # x = np.arange(0, C_space.shape[1],1)
    # y = np.arange(0, C_space.shape[2], 1)
    # xdata, ydata = np.meshgrid(x,y)
    # zdata = C_space[0, :, :]
    # ax.plot_surface(xdata,ydata,zdata, cmap='Blues')
    # plt.show#

#######---------------------field--------------------##############
def ThreeD_field(Goal = np.array([7,7,18])):
    C_space = cspaceCreate()
    cost = 1 / 2 * Wavefront_3D.wavefrontAlgothm(Goal, C_space)
    distance_array = ndimage.distance_transform_edt(1 - C_space)
    # mu = 10
    # D0 = 2
    # D2 = distance_array / 10 + 1
    # repulsive = mu * (1 / D2 - 1 / D0)
    # f = repulsive + cost
    mu = 100
    D0 = 2
    D2 = distance_array / 10 + 1
    repulsive = mu * np.square((1 / D2 - 1 / D0) * (1/D2 > 1/D0))
    f = repulsive + cost
    return f

#####---------------------------------------------Attractive field---------------------
C_space = cspaceCreate()
cost = 1/2 * Wavefront_3D.wavefrontAlgothm(Goal, C_space)

# picture for cost
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# [zdata,xdata,ydata] = np.where(cost[:,:,:]> 0.9)
# ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
# ax.scatter(xdata, ydata, c=zdata, cmap='Greens')
# plt.show()
#####---------------------------------------------Repulsive field---------------------
distance_array = ndimage.distance_transform_edt(1-C_space)
mu = 20
D0 = 2
D2 = distance_array / 10 + 1
repulsive = mu * (1 / D2 - 1 / D0)

#picuture for repulsive
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# x = np.arange(0, repulsive.shape[1],1)
# y = np.arange(0, repulsive.shape[2], 1)
# xdata, ydata = np.meshgrid(x,y)
# zdata = repulsive[1, :, :]
# ax.plot_surface(xdata,ydata,zdata, cmap='Blues')
# plt.show()

#####---------------------------------------------Repulsive + Attractive---------------------
f = repulsive + cost

#picuture for f
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# x = np.arange(0, f.shape[1],1)
# y = np.arange(0, f.shape[2], 1)
# xdata, ydata = np.meshgrid(x,y)
# zdata = f[0, :, :]
# ax.plot_surface(xdata,ydata,zdata, cmap='Blues')
# plt.show()























