# from Search import Search
import Search
from retrieval import Retrieval
from controller import Robot, DistanceSensor, Motor, Camera, CameraRecognitionObject, GPS, Receiver, InertialUnit
import time
import numpy as np
from calculation import *
import math
from field_theory import magnetic_field
#scalar
PI = math.pi
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28
SPEED_CONSTANT = 6.28/1000
DISTANCE_THRESHOLD = 10

CAMERA_SIZE_X = 640
CAMERA_SIZE_Y = 480

FLAG = 0
retrieval_flag = 0

MIN_IR_SENSOR = 55
MAX_IR_SENSOR = 2700
IR_RANGE = 1024

Box_initial_position = [0, 0]
Box_goal_position = [0.3, 0.3]

# initial food parameter
food_position = [0,0,0,0]
food_D = 0.2

# create the Robot instance.

robot = Robot()
robot_search = Search.Search()
recognization = CameraRecognitionObject()
robot_retrieval = Retrieval()
# initialize devices
#sensorsmath.pi
print(robot.getName())
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)
#motor
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
#Camera
camera = robot.getCamera("recognization_camera")
camera.enable(TIME_STEP)
time.sleep(0.1)
cameraData = camera.getImage()
camera.recognitionEnable(TIME_STEP)
#IMU initialization
IMU = robot.getInertialUnit("inertial unit")
IMU.enable(TIME_STEP)
time.sleep(0.1)
IMU_data = IMU.getRollPitchYaw()
print(IMU_data)
# GPS initialization
gps = robot.getGPS('gps')
gps.enable(TIME_STEP)
# receiver
receiver = robot.getReceiver("receiver")
receiver.enable(TIME_STEP)
box_position = []
# print(gray)
# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    ############################-------############################
    IMU_data = IMU.getRollPitchYaw()
    yaw = IMU_data[2]
    print(yaw)
    #Receiver data
    if receiver.getQueueLength() > 0:
        receiver_data = receiver.getData()
        food_position = eval(bytes.decode(receiver_data))
        receiver.nextPacket()
    psValues = []
    C_position = []
    for i in range(8):
        psValues.append((ps[i].getValue()-MIN_IR_SENSOR)*IR_RANGE/(MAX_IR_SENSOR-MIN_IR_SENSOR))
    #read GPS data
    gpsV=gps.getValues()
    # read camera_data
    num = camera.getRecognitionNumberOfObjects()
    capture_object = camera.getRecognitionObjects()
    for i in range(num):
        C_position = capture_object[i].get_position_on_image()
    ############################-------#############################
    distance_list = distance_target(food_position, food_D, Box_goal_position) #5 distance between center and goal
    converge_list = [0,0,0,0]
    converge_line = []
    for i in range(4):
        if distance_list[i] >= distance_list[4]:
            converge_list[i] = 1
    # print(converge_list)
    for i in range(4):
        if converge_list[i] == 1:
            pointA = [food_position[0] + pow(2,1/2)/2*food_D*np.cos(PI/4+food_position[3]+i*PI/2),
            food_position[2] - pow(2,1/2)/2*food_D*np.sin(PI/4+food_position[3]+i*PI/2)]
            pointB = [food_position[0] + pow(2,1/2)/2*food_D*np.cos(PI/4+food_position[3]+(i+1)*PI/2),
            food_position[2] - pow(2,1/2)/2*food_D*np.sin(PI/4+food_position[3]+(i+1)*PI/2)]
            converge_line.append([pointA, pointB])
    line_judgement = [] # distance between epuck and line
    n = int(np.size(converge_line)/4)
    for i in range(n):
        line_judgement.append(whichside(converge_line[i][0],converge_line[i][1],gpsV, food_position))
    line_judgement_min = min(line_judgement)
    # calculate the converge_angle
    converge_angle = magnetic_field(gpsV,food_position,food_D,converge_list)
    diff_angle = yaw - converge_angle #if diff_angle > 0 turn right if diff_angle< 0 turn left
    if diff_angle > math.pi:
        diff_angle = diff_angle - 2*math.pi
    if diff_angle < -1*math.pi:
        diff_angle = diff_angle + 2*math.pi
    # print(line_judgement)
    robot_retrieval.swarm_retrieval(psValues,DISTANCE_THRESHOLD,line_judgement_min,C_position,num,diff_angle)
    left_wheel_speed = robot_retrieval.get_retrieval_left_wheel_speed()*SPEED_CONSTANT
    right_wheel_speed = robot_retrieval.get_retrieval_right_wheel_speed()*SPEED_CONSTANT

    # # write actuators inputs
    if left_wheel_speed > MAX_SPEED:
        left_wheel_speed = MAX_SPEED
    if left_wheel_speed < -1*MAX_SPEED:
        left_wheel_speed = -1*MAX_SPEED
    if right_wheel_speed > MAX_SPEED:
        right_wheel_speed = MAX_SPEED
    if right_wheel_speed < -1*MAX_SPEED:
        right_wheel_speed = -1*MAX_SPEED
    leftMotor.setVelocity(left_wheel_speed)
    rightMotor.setVelocity(right_wheel_speed)
