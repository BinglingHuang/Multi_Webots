# import search
# import retrieval

import numpy as np
import time
import random

#-------------------- constant -------------------#
TRUE = 1
FALSE = 0
NEUTRAL = 3
ON = 1
OFF = 0
IR_DIFF_THRESHOLD = 4
DISTANCE_DIFF_THRESHOLD = 10
REVERSE_LIMIT = 20
TURN_LIMIT = 10
FORWARD_LIMIT = 40
NEIGHBOR_LIMIT = 300
ALIGN_STRAIGTH_THRESHOLD = 10
LOW_DIST_VALUE = 10

class stagnation:

    def __init__(self):
        ###############------ wheel speed variables ------#################
        self.left_wheel_speed = 0
        self.right_wheel_speed = 0
        ###############------ boolean variables ------#################
        self.has_recovered = FALSE
        self.turn_left = NEUTRAL
        ###############------ LED ------#################
        self.green_LED_state = OFF
        ###############------ Counters ------#################
        self.reverse_counter = 0
        self.turn_counter = 0
        self.forward_counter = 0
        self.twice = 0
        self.align_counter = 0

    #--------------------internal function-------------------#

    def LED_blink(self):
        if self.green_LED_state:
            self.green_LED_state = OFF;
        else:
            self.green_LED_state = ON;

    def realign(self, distance_value):
        distance_value = list(map(float,distance_value))
        dist_diff_front = int(distance_value[7] - distance_value[0])
        if abs(dist_diff_front) > ALIGN_STRAIGTH_THRESHOLD:
            if distance_value[0] < LOW_DIST_VALUE:
                self.right_wheel_speed = -500
                self.left_wheel_speed = 500
            elif distance_value[7] < LOW_DIST_VALUE:
                self.right_wheel_speed = 500
                self.left_wheel_speed = -500
            elif distance_value[1] < LOW_DIST_VALUE:
                self.right_wheel_speed = -1000
                self.left_wheel_speed = 700
            elif distance_value[6] < LOW_DIST_VALUE:
                self.right_wheel_speed = 700
                self.left_wheel_speed = -1000
        else:
            ran = random.random()
            if ran > 0.5:
                self.right_wheel_speed = -500
                self.left_wheel_speed = 500
            else:
                self.right_wheel_speed = 500
                self.left_wheel_speed = -500
        self.has_recovered = TRUE
        self.green_LED_state = OFF

    #--------------------external function-------------------#

    def find_new_spot(self, distance_value, DIST_THRESHOLD):
        distance_value = list(map(float,distance_value))
        DIST_THRESHOLD = int(DIST_THRESHOLD)
        if self.twice == 2:
            self.has_recovered = TRUE
            self.green_LED_state = OFF
            self.align_counter = 0
        elif self.reverse_counter != REVERSE_LIMIT:
            self.reverse_counter = self.reverse_counter + 1
            self.left_wheel_speed = -800
            self.right_wheel_speed = -800
        elif self.turn_counter != TURN_LIMIT:
            self.turn_counter = self.turn_counter + 1
            self.forward_counter = 0
            if self.turn_left == NEUTRAL:
                ran = random.random()
                if ran > 0.5:
                    self.turn_left = FALSE
                else:
                    self.turn_left = TRUE
                if self.turn_left:
                    self.left_wheel_speed = -300
                    self.right_wheel_speed = 700
                else:
                    self.left_wheel_speed = 700
                    self.right_wheel_speed = -300
        elif self.forward_counter != FORWARD_LIMIT:
            self.forward_counter = self.forward_counter + 1
            if self.forward_counter == FORWARD_LIMIT - 1:
                self.twice = self.twice + 1
                self.turn_counter = 0
                if self.turn_left:
                    self.turn_left = FALSE
                else:
                    self.turn_left = TRUE
            search.update_search_speed(distance_value, DIST_THRESHOLD)
            self.left_wheel_speed = search.get_search_left_wheel_speed()
            self.right_wheel_speed = search.get_search_right_wheel_speed()
            if self.left_wheel_speed > 0 and self.right_wheel_speed > 0:
                self.right_wheel_speed = 1000
                self.left_wheel_speed = 1000

    def reset_stagnation(self):
        self.has_recovered = FALSE
        self.reverse_counter = 0
        self.turn_counter = 0
        self.forward_counter = 0
        self.turn_left = NEUTRAL
        self.twice = 0

    def stagnation_recovery(self, distance_value, DIST_THRESHOLD):
        distance_value = list(map(float,distance_value))
        DIST_THRESHOLD = int(DIST_THRESHOLD)
        if self.align_counter < 2:
            self.align_counter = self.align_counter + 1
            self.realign(distance_value)
        elif self.align_counter > 0:
            self.LED_blink()
            self.find_new_spot(distance_value,DIST_THRESHOLD)

    #--------------------judge if continue pushing-------------------#

    def valuate_pushing(self, distance_value, pre_distance_value):
        distance_value = list(map(float,distance_value))
        # print(pre_distance_value)
        # prev_distance_value = list(map(float,prev_distance_value))
        dist_diff7 = int(pre_distance_value[7] - distance_value[7])
        dist_diff0 = int(pre_distance_value[0] - distance_value[0])
        if abs(dist_diff7) > DISTANCE_DIFF_THRESHOLD and abs(dist_diff0) > DISTANCE_DIFF_THRESHOLD:
            self.has_recovered = TRUE
            self.green_LED_state = OFF
            self.align_counter = 0
        elif distance_value[5] > NEIGHBOR_LIMIT and distance_value[2] > NEIGHBOR_LIMIT:
            self.has_recovered = TRUE
            self.green_LED_state = OFF
            self.align_counter = 0
        elif distance_value[5] > NEIGHBOR_LIMIT or distance_value[2] > NEIGHBOR_LIMIT:
            ran = random.random()
            if ran > 0.5:
                self.has_recovered = TRUE
                self.green_LED_state = OFF
                self.align_counter = 0

    #--------------------return the boolean value-------------------#
    def get_stagnation_state(self):
        if self.has_recovered:
            return FALSE
        return TRUE
    def get_green_LED_state(self):
        return self.green_LED_state
    def get_stagnation_left_wheel_speed(self):
        return self.left_wheel_speed
    def get_stagnation_right_wheel_speed(self):
        return self.right_wheel_speed
