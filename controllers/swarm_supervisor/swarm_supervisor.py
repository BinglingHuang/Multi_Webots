"""swarm_supervisor controller."""
"""
    the problem is a single objective optimization problem to minimize the objective function
    objective function
        f = alpha * d + beta * (t - tmin)^2 + gama * n
        d: distance between the box and target
        t: time step to accomplish the simulation
        tmin: minimal time step to accomplish the game
        n: number to hit the obstacle
    optimization parameters
        R1: threshold distance regard to social field between epuck [0,1]
        R2: threshold distance regard to task field converge line between obs and box [0,1]
        K1: parameter to calculate the converge line regard to the distance between obstacle and box [0,5]
        k2: attraction parameter to calculate the task field [0,5]
        k3: repellent parameter to calculate the task field [0,5]
        k4: repellent parameter to calculate the social field [0,1]
"""
#   swarm supervisor supervise the whole scenario
#   supervisor use custom data to send the information (obstacle and other agents' position) to each agent

from controller import Robot, Supervisor, Node, Emitter, Field, Camera
import math
from swarm_supervisor_calculation import *
import time
from simulation_reset import simulation_reset
from population import *
import numpy as np
import cv2
from scipy import ndimage
from camera_Image import *
###########-----------scalar-----------###########
TIME_STEP = 64
RESOLUTION = 640
SIZE = 2
num_epuck = 3
num_obstacle = 2
field = {}
node = {}
initial_translation = {}
initial_rotation = {}
distance_threshold = 0.03
times = 0
summation = 0
epuck_D = 0.07
obstacle_D = 0.4
SIMULATION_MODE_PAUSE = 0
SIMULATION_MODE_REAL_TIME = 1
SIMULATION_MODE_RUN = 2
SIMULATION_MODE_FAST = 3
POPULATION_SIZE = 5
GENOTYPE_SIZE = 5
Box_goal_position = np.array([0.3, 0.3])
alpha = 1000
beta = 1/1000
tmin = 800
gama = 1
box_D = 0.2
obstacle_D = 0.4
eps = 10**-10

###########-----------create the node instance-----------###########

robot = Supervisor()
Emitter = robot.getEmitter("emitter")
camera = robot.getCamera("supervisor_camera")
camera.enable(TIME_STEP)
time.sleep(0.1)
bool = True
timestep = int(robot.getBasicTimeStep())
food_node = robot.getFromDef("food")
# imageArray = camera.getImageArray()
# print(imageArray)
# # display the components of each pixel
# for x in range(0,camera.getWidth()):
#   for y in range(0,camera.getHeight()):
#     red   = image[x][y][0]
#     green = image[x][y][1]
#     blue  = image[x][y][2]
#     gray  = (red + green + blue) / 3
#     print 'r='+str(red)+' g='+str(green)+' b='+str(blue)


###########-----------initialize the box node, field and get initial translation and rotation of box -----------###########
node["food"] = robot.getFromDef("food")
field["food_translation"] = node["food"].getField("translation")
field["food_rotation"] = node["food"].getField("rotation")
initial_translation["food_translation"] = field["food_translation"].getSFVec3f()
initial_rotation["food_rotation"] = field["food_rotation"].getSFRotation()


############-----------initialize the node and field and get relative data-----------############
for i in range(num_epuck):
    name = "e-puck_" + "%d"%(i+1)
    epucknode = robot.getFromDef(name)
    node[name] = epucknode
    field_name = name + "_customdata"   # initialize the node and custom field
    customdatafield = epucknode.getField('customData')
    field[field_name] = customdatafield
    field_name = name + "_translation"      # initialize other field and get initial translation
    field[field_name] = node[name].getField("translation")
    initial_translation[field_name] = field[field_name].getSFVec3f()
    field_name = name + "_rotation"        # initialize other field and get initial rotation
    field[field_name] = node[name].getField("rotation")
    initial_rotation[field_name] = field[field_name].getSFRotation()


###########-----------create the initial population-----------###########
ea_algorithm = EA_algorithm(POPULATION_SIZE, GENOTYPE_SIZE)
generation = ea_algorithm.generation
best_fitness = np.array([0])
best_params = generation[0,:]
# print("generation: ", generation)
for gen in range(GENOTYPE_SIZE):
    fitnessValue = np.array([0])
    CV = np.array([0])
    for j in range(POPULATION_SIZE):
        n = 0
        while robot.step(TIME_STEP) != -1:


            summation = summation + 1
            times = times + 1
            ##########--------------get the EA parameter------------##############
            ea_variable = generation[j,:]
            # R1 = ea_variable[0]
            # K2 = ea_variable[3]
            # K3 = ea_variable[4]

            R1 = 0.44
            K2 = 5.00
            K3 = 1.41
            distance_threshold = R1
            ##########--------------emit parameter position------------##############
            send_data = str.encode(str(list(ea_variable)))
            Emitter.send(send_data)
            a = {}
            ##########--------------calculate food position------------##############
            name = "food_position"
            food_position = food_node.getPosition()
            # print(food_position)
            box_position = np.array([food_position[0], food_position[2]])
            food_orientation = food_node.getOrientation()
            yaw = math.atan2(food_orientation[2], food_orientation[8])
            food_position.append(yaw)
            # print("food_position: ", food_position)
            a[name] = food_position

            ##########--------------create the save image------------##############
            if bool:
                image = camera.getImage()
                succe = camera.saveImage('Environment1.png', 100)
                bool = False

            ##########--------------calculate epuck position------------##############

            for i in range(num_epuck):
                name = "e-puck_" + "%d"%(i+1)
                epucknode = node[name]
                epuck_position = epucknode.getPosition()
                epuck_orientation = epucknode.getOrientation()
                yaw = math.atan2(epuck_orientation[2], epuck_orientation[8])
                epuck_position.append(yaw)
                a[name] = epuck_position
            ##########--------------calculate obstacle position------------##############

            blackAndWhiteImage = transfer_to_baw_image('Environment.png')
            map_array = resize(blackAndWhiteImage, 25)
            map_array[map_array > 0] = 1
            ReprMeter = (map_array.shape[0]-1) / SIZE
            distance_array = ndimage.distance_transform_edt(map_array)
            box_position = (box_position + SIZE/2)*ReprMeter
            box_position = box_position[[1, 0]]
            box_position_x = int(round(box_position[0]))
            box_position_y = int(round(box_position[1]))
            distance = distance_array[box_position_x][box_position_y] - box_D * ReprMeter /2 * 2 ** (1/2)
            if distance <= 1: n = n +1


            ##########--------------set custom data field------------##############
            for i in range(num_epuck):
                b = a.copy()
                E_puck_position = a["e-puck_" + "%d"%(i+1)]
                E_puck_position = [E_puck_position[0], E_puck_position[2]]
                for key in a.keys():
                    other_position = [b[key][0], b[key][2]]
                    dis = 0
                    if "e-puck" in key:
                        dis = distance_position(other_position, E_puck_position) - epuck_D
                    if dis > distance_threshold:    b.pop(key)
                b.pop("e-puck_" + "%d" % (i + 1))
                field["e-puck_" + "%d" % (i + 1) + "_customdata"].setSFString(str(b))
            # print(summation)

            distance_box_target = distance_position(box_position, Box_goal_position)
            # print("dis: ", distance_box_target)

            if times ==2000 or distance_box_target < 0.1:
                f = np.array([alpha * distance_box_target + beta * (times - tmin) ** 2 + gama * n])
                fitnessValue = np.vstack((fitnessValue, f))
                constrains = np.array([K3 - K2])
                CV = np.vstack((CV, constrains))
                times = 0
                simulation_reset(node, field, initial_translation, initial_rotation)
                break
    # print("finish %d generation" % gen)
    fitnessValue = np.delete(fitnessValue, 0, axis=0)
    best_index = np.argmax(fitnessValue)
    if best_fitness > fitnessValue[best_index]:
        best_fitness = fitnessValue[best_index]
        best_params = generation[best_index,:]
    CV = np.delete(CV, 0, axis=0)
    generation = ea_algorithm.reproduce_generation(fitnessValue, CV, gen)
    # if gen == 0:
    #     np.savetxt("result1.txt", generation)
    # if gen == GENOTYPE_SIZE-1:
    #     np.savetxt("result2.txt", generation)
    #     np.savetxt("best_population.txt", best_params)
np.savetxt("best_population.txt", best_params)


