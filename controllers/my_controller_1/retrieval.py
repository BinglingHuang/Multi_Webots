# retrieval -- Follow and push behavior
# Made to make the e-puck converge and push the box
import numpy as np
from stagnation import stagnation
from Search import Search
# -------Constant------#
NB_LEDS = 8
ON = 1
OFF = 0
TRUE = 1
FALSE = 0
PUSH_THRESHOLD = 150#150
converge = FALSE
push = FALSE
LED = [0,0,0,0,0,0,0,0]
left_wheel_speed = 0
right_wheel_speed = 0
CAMERA_SIZE_X = 640
CAMERA_SIZE_Y = 480
retrieval_flag =  0
BOX_D = 0.2
class Retrieval:

    def __init__(self):
        self.stagnation = stagnation()
        self.pre_distance_value = [0,0,0,0,0,0,0,0]
        self.robot_search = Search()
    def update_speed(self, IR_number):
        global left_wheel_speed
        global right_wheel_speed
        if IR_number == 0:
            left_wheel_speed = left_wheel_speed + 700
        elif IR_number == 7:
            right_wheel_speed = right_wheel_speed + 700
        elif IR_number == 1:
            left_wheel_speed = left_wheel_speed + 350
        elif IR_number == 6:
            right_wheel_speed = right_wheel_speed + 350
        elif IR_number == 2:
            left_wheel_speed = left_wheel_speed + 550
            right_wheel_speed = right_wheel_speed - 300
        elif IR_number == 5:
            right_wheel_speed = right_wheel_speed + 550
            left_wheel_speed = left_wheel_speed - 300
        elif IR_number == 3:
            left_wheel_speed = left_wheel_speed + 500
        elif IR_number == 4:
            right_wheel_speed = right_wheel_speed + 500
        # print("left:", left_wheel_DIST_THRESHOLDspeed)
        # print("right:", right_wheel_speed)

    # The movement for converging to the box
    def converge_to_box(self, IR_sensors_value, IR_threshold):
        global left_wheel_speed
        global right_wheel_speed
        left_wheel_speed = 0
        right_wheel_speed = 0
        if IR_sensors_value[0] < IR_threshold and IR_sensors_value[7] < IR_threshold:
            left_wheel_speed = 1000
            right_wheel_speed = 1000
        for i in range(NB_LEDS):
            if IR_sensors_value[i] > IR_threshold:
                LED[i] = ON
                self.update_speed(i)
            else:
                LED[i] = OFF


    # The behavior when pushing the box
    def push_box(self, IR_sensors_value, IR_threshold):
        global left_wheel_speed
        global right_wheel_speed
        left_wheel_speed = 0
        right_wheel_speed = 0
        IR_threshold = IR_threshold + 90
        for i in range(NB_LEDS):
            if LED[i]:
                LED[i] = OFF
            else:
                LED[i] = ON
            if IR_sensors_value[i] > IR_threshold:
                self.update_speed(i)

        # if IR_sensors_value[0] > IR_threshold and IR_sensors_value[7] > IR_threshold:
        #     left_wheel_speed = 1000
        #     right_wheel_speed = 1000

    # Selects the behavior push or converge
    def select_behavior(self, IR_sensor_value, converge_judgement, C_position, num):
        global push
        global converge
        global search
        global retrieval_flag
        search = TRUE
        push = FALSE
        converge = FALSE
        max_value = max(IR_sensor_value)
        if num >= 1 and abs(C_position[0]-CAMERA_SIZE_X/2) < 200 and abs(C_position[1]-CAMERA_SIZE_Y/2) < 200 or retrieval_flag == 1:
            converge = TRUE
            search = FALSE
            retrieval_flag = 1
            if max_value < 20:
                retrieval_flag = 0
            for i in range(NB_LEDS):
                if IR_sensor_value[i] > PUSH_THRESHOLD:
                    if converge_judgement >0:
                        push = FALSE
                        converge = FALSE
                        break
                    push = TRUE
                    converge = FALSE
                    search = FALSE
                    break

    # Converge, push, and stagnation recovery
    def swarm_retrieval(self, IR_sensor_value, IR_threshold, converge_judgement, C_position, num, diff_angle):
        global left_wheel_speed
        global right_wheel_speed
        global push
        global converge
        global search
        self.select_behavior(IR_sensor_value, converge_judgement,C_position, num)
        # print('push = ',push)
        # print('converge = ',converge)
        # print('search = ',search)
        if push:
            self.push_box(IR_sensor_value, IR_threshold)
            self.stagnation.valuate_pushing(IR_sensor_value,self.pre_distance_value)
            self.pre_distance_value = IR_sensor_value
            if self.stagnation.get_stagnation_state():
                self.stagnation.stagnation_recovery(IR_sensor_value,IR_threshold)
                self.stagnation.reset_stagnation()
                left_wheel_speed = self.stagnation.get_stagnation_left_wheel_speed()
                right_wheel_speed = self.stagnation.get_stagnation_right_wheel_speed()
                print("enter stagnation")
            # print(self.stagnation.align_counter)
        elif converge:
            self.converge_to_box(IR_sensor_value, IR_threshold)
        else:#search
            self.robot_search.update_search_speed(IR_sensor_value,IR_threshold,diff_angle)
            left_wheel_speed = self.robot_search.get_search_left_wheel_speed()
            right_wheel_speed = self.robot_search.get_search_right_wheel_speed()

    @staticmethod
    def get_retrieval_left_wheel_speed():
        return left_wheel_speed

    @staticmethod
    def get_retrieval_right_wheel_speed():
        return right_wheel_speed

    @staticmethod
    # Returns the state (ON/OFF) of the given LED number
    def get_LED_state(LED_num):
        return LED[LED_num]
