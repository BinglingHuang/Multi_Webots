"""main controller"""
# from field_theory_controller.field_theory import converge_line_calculation, magnetic_field, social_field

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
# import Search
from controller import Robot, DistanceSensor, Motor, Camera, CameraRecognitionObject, GPS, Receiver, InertialUnit
import time
from field_theory_calculation import *
from field_theory import *
from diff_drive import diff_drive
import numpy as np
import math
import sys
sys.path.append("C:\\Users\\Administrator\\PycharmProjects\\APF_MSAA\\APF_MSAA_PYTHON\\new_obstacle_environment_version2\\controllers\\swarm_supervisor")
import camera_Image
import path_planning_new
import swarm_supervisor_calculation
import random

#########----------------constant---------------############
PI = math.pi
TIME_STEP = 64
#
MAX_SPEED = 6.28
SPEED_CONSTANT = 6.28/1000
DISTANCE_THRESHOLD = 10

CAMERA_SIZE_X = 640
CAMERA_SIZE_Y = 480


MIN_IR_SENSOR = 55
MAX_IR_SENSOR = 2700
IR_RANGE = 1024

Box_initial_position = np.array([-0.6, -0.6])
Box_goal_position = np.array([-0.6, 0.6])

# initial food parame# using the fielfood_Dd theory to converge to the boxter
food_position = np.array([1,0,0,0])
food_D = 0.2
SIZE = 2
max_its = 3000
FLAG = True

#########--------------initialiazation-------------############
# create the Robot instance.
robot = Robot()
robot_ID = robot.getName()
# robot_search = Search.Search()
recognization = CameraRecognitionObject()
# motor
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
# Camera temporary not use
camera = robot.getCamera("recognization_camera")
camera.enable(TIME_STEP)
time.sleep(0.1)
# cameraData = camera.getImage()
# print(cameraData)
camera.recognitionEnable(TIME_STEP)
# IMU initialization
IMU = robot.getInertialUnit("inertial unit")
IMU.enable(TIME_STEP)
time.sleep(0.1)
IMU_data = IMU.getRollPitchYaw()
# GPS initialization
gps = robot.getGPS('gps')
gps.enable(TIME_STEP)
# receiver
receiver = robot.getReceiver("receiver")
receiver.enable(TIME_STEP)
box_position = []
# diff model
epuck_diff_model = diff_drive()
route_index = 0
position_list = [[0,0,0]] * 10
left_wheel_speed = 0
right_wheel_speed = 0
t1 = time.time()
# print("start the main loop")
#########--------------loop-------------############
while robot.step(TIME_STEP) != -1:
    #########--------------seunsor_data_update-------------############


    # R1 = ea_variable[0]
    # R2 = ea_variable[1]
    # K1 = ea_variable[2]
    # K2 = ea_variable[3]
    # K3 = ea_variable[4]
    # K4 = ea_variable[5]

    R1 = 0.44
    R2 = 0.44
    K1 = 2.89
    K2 = 5.00
    K3 = 1.41
    K4 = 0.38


    #########--------------processing the picture------------############
    if FLAG:
        path = 'C:/Users/Administrator/PycharmProjects/APF_MSAA/APF_MSAA_PYTHON/new_obstacle_environment_version2/controllers/swarm_supervisor/Environment.png'
        bAWImage = camera_Image.transfer_to_baw_image(path)
        map_array = camera_Image.resize(bAWImage, 25)
        map_array[map_array > 0] = 1
        ReprMeter = (map_array.shape[0] - 1) / SIZE
        Goal_position = (Box_goal_position + SIZE / 2) * ReprMeter
        Goal_position = Goal_position[[1, 0]]
        start_position = (Box_initial_position + SIZE /2) * ReprMeter
        start_position = start_position[[1,0]]
        filed, att, rep, distance_array = camera_Image.field_creation(path, Goal_position)
        # lmin = path_planning_new.find2D_array_localminimum(filed, Goal_position)
        route = path_planning_new.route_webots(filed, att, start_position, Goal_position, max_its, distance_array)
        route = np.array(route)
        FLAG = False


    #########--------------field_theory_implementation------------############
    # judge the converge line
    # construct the field and let the epuck to converge to the box
    # calculate the e-puck input
    # while route_flag==True:
    Customdata = eval(robot.getCustomData())
    IMU_data = IMU.getRollPitchYaw()
    yaw = IMU_data[2]  # robot_orientation
    if receiver.getQueueLength() > 0:  # box position from supervisor the position of the box can be added into custom data
        receiver_data = receiver.getData()
        ea_variable = eval(bytes.decode(receiver_data))
        receiver.nextPacket()
    else:
        continue
    gpsV = gps.getValues()  # robot_position
    e_puck_position = np.array([gpsV[0], gpsV[2], yaw])
    food_position = Customdata["food_position"]
    Food_position = np.array([food_position[0], food_position[2], food_position[3]])
    Food_position_xy = np.array([food_position[0], food_position[2]])
    box_temp_goal_position = route[route_index]
    # print("temp_position: ", box_temp_goal_position)
    if distance(Food_position_xy, box_temp_goal_position) < 0.05:
        route_index = route_index + 1
    if route_index == route.shape[0]:
        left_wheel_speed = 0
        right_wheel_speed = 0
        leftMotor.setVelocity(left_wheel_speed)
        rightMotor.setVelocity(right_wheel_speed)
        break
    converge_list = converge_line_calculation(Food_position, food_D, box_temp_goal_position, ReprMeter, map_array)
    # print("converge_list in controller: ", converge_list)
    task_vector = np.array(magnetic_field(e_puck_position,Food_position,food_D,converge_list, K2, K3))
    # print("task_field: ", task_vector)
    # task_vector = np.array(Gaussian_field(e_puck_position, food_position, food_D, converge_list))
    social_vector = np.array(social_field(Customdata, e_puck_position, K4, ReprMeter, map_array, rep))
    # print("social_field: ", social_vector)
    vector = task_vector + social_vector
    # print("vector: ", vector)
    # print('robotname = ', robot_ID ,'social_vector = ', social_vector, 'task_vector =', task_vector)
    torward_angle = -1 * math.atan2(vector[1], vector[0]) - math.pi / 2
    diff_angle = torward_angle - yaw # if diff_angle > 0 turn left if diff_angle< 0 turn right
    if diff_angle > math.pi:
        diff_angle = diff_angle - 2 * math.pi
    if diff_angle < -1 * math.pi:
        diff_angle = diff_angle + 2 * math.pi
    v = 0.15
    w = 1.5 * (v / 0.1) * diff_angle





    epuck_diff_model.inverse_kinematic(v,w)
    left_wheel_speed = epuck_diff_model.wl
    right_wheel_speed = epuck_diff_model.wr
    ############-----------------motor_actuation---------------#############
    if left_wheel_speed > MAX_SPEED:
        left_wheel_speed = MAX_SPEED
    if left_wheel_speed < -1 * MAX_SPEED:
        left_wheel_speed = -1 * MAX_SPEED
    if right_wheel_speed > MAX_SPEED:
        right_wheel_speed = MAX_SPEED
    if right_wheel_speed < -1 * MAX_SPEED:
        right_wheel_speed = -1 * MAX_SPEED
    # left_wheel_speed = 6.28
    # right_wheel_speed = 6.28
    # print(left_wheel_speed)
    position_list.insert(0, e_puck_position)
    position_list.pop()
    if np.array(position_list[9]).any() != 0:
        sum = 0
        for i in range(10):
            sum = sum + distance(position_list[0], position_list[i])
        if sum < 0.01:
            print("start: ", time.time() - t1)
            left_wheel_speed = MAX_SPEED * random.uniform(-1,1)
            right_wheel_speed = MAX_SPEED * random.uniform(-1,1)
    leftMotor.setVelocity(left_wheel_speed)
    rightMotor.setVelocity(right_wheel_speed)

