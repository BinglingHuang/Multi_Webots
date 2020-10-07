import cv2
import numpy as np
from scipy import ndimage
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from swarm_supervisor_calculation import *
# from PIL import Image
# import ImageDraw


RESOLUTION = 640
SIZE = 2
Box_D = 0.2
goal_position = [20, 120]
sum = []
k = 0
esp = 10**(-10)

def transfer_to_baw_image(path):
    image = cv2.imread(path)
    # cv2.imshow("image", image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY) ## 0 is black, 1 is white
    cv2.imwrite("Black_white_image.png", blackAndWhiteImage)
    cv2.waitKey(0)
    return blackAndWhiteImage

def plot_transfer_to_baw_image(path):
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
    # cv2.imshow('Original image',image)
    # cv2.imshow('Gray image', gray)
    cv2.imshow('Black white image', blackAndWhiteImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return blackAndWhiteImage


def repulsive(image_array, nu, threshold_repulsive,goal_position):
    distance_array = ndimage.distance_transform_edt(image_array)
    normal_array = (distance_array/30)+1
    # print("normal_array: ", normal_array[70])
    # normal_array = SIZE/RESOLUTION*distance_array
    repulsize = nu * (1/normal_array - 1/threshold_repulsive)**2
    # repulsize[normal_array != 1] = repulsize[normal_array] + 1
    repulsize[ normal_array > threshold_repulsive] = 0
    goal_position = [int(round(goal_position[0])), int(round(goal_position[1]))]
    repulsize[goal_position[0]][goal_position[1]]=0
    return repulsize

def attractive(output, goal_position, omega):
    x = np.arange(0, output.shape[0], 1)
    y = np.arange(0, output.shape[1], 1)
    yy1, xx1 = np.meshgrid(x, y)
    goal_position = [int(round(goal_position[0])), int(round(goal_position[1]))]
    attractive = omega * ((xx1-goal_position[0])**2+(yy1-goal_position[1])**2)
    return attractive

def resize(blackAndWhiteImage, percent):
    image = blackAndWhiteImage
    scale_percent = percent
    # calculate the 50 percent of original dimensions
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dsize = (width, height)
    output = cv2.resize(image, dsize)
    return output

def field_creation(path, goal_position):
    print("c")
    print(path)
    blackAndWhiteImage = transfer_to_baw_image(path)
    output = resize(blackAndWhiteImage, 25)
    output[output > 0] = 1
    distance_array = ndimage.distance_transform_edt(output)
    rep = repulsive(output, 400, 2, goal_position)
    att = attractive(output, goal_position, 1/100)
    field = rep + att
    return field, att, rep, distance_array

def distance_cost(position_A, position_B):
    distance_twoPosition = np.sqrt(pow((position_A[0] - position_B[0]), 2) + pow((position_A[1] - position_B[1]), 2))
    return distance_twoPosition

if __name__ == '__main__':
    path = 'C:/Users/Administrator/PycharmProjects/APF_MSAA/APF_MSAA_PYTHON/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
    # path = 'C:/Users/Administrator/PycharmProjects/APF_path planning/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
    blackAndWhiteImage = transfer_to_baw_image(path)
    output = resize(blackAndWhiteImage, 25)
    output[output > 0] = 1
    x = np.arange(0, output.shape[0], 1)
    y = np.arange(0, output.shape[1], 1)
    yy1, xx1 = np.meshgrid(x, y)
    rep = repulsive(output, 400, 3, goal_position)
    # print(rep[60])
    att = attractive(output, goal_position, 1/100)
    # print(rep[60][40])
    # print(rep[60][24])

    z = rep + att

    print(z[61][21])
    print(z[106][44])
    print(z[104][45])
    print(z[103][46])
    print(z[99][50])


    # print(z[])
    # print(z[120][26])
    # print(z[121][27])
    # print(z[122][28])
    # print(z[123][29])
    fig = plt.figure()
    ax3 = plt.axes(projection='3d')
    ax3.plot_surface(xx1, yy1, z, rstride=1, cstride=1, cmap=cm.cool)
    plt.show()
