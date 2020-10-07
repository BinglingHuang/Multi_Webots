# Search -- Search and Avoid behavior
# Made to calculate the speed from distance sensors input
# By getting sensor input form the four front distance sensors on the e-puck.
# it will be determined the speed of the left and right wheel according to the case script and a threshold.
# if nothing is in your way - search

import numpy as np
import random
COUNTLIMIT = 20
counter = 0
rand_double_left = 0
rand_double_right = 0
left_wheel_speed = 0
right_wheel_speed = 0
case_script = np.array(
    [(0, 0, 0, 0, 1, 1), (0, 0, 0, 1, 1, 0), (0, 0, 1, 0, 1, 0), (0, 0, 1, 1, 1, 0), (0, 1, 0, 0, 0, 1),
     (0, 1, 0, 1, 1, 0),
     (0, 1, 1, 0, 0, 1), (0, 1, 1, 1, 1, 0), (1, 0, 0, 0, 0, 1), (1, 0, 0, 1, 1, 0), (1, 0, 1, 0, 0, 1),
     (1, 0, 1, 1, 1, 0), (1, 1, 0, 0, 0, 1),
     (1, 1, 0, 1, 1, 0), (1, 1, 1, 0, 0, 1), (1, 1, 1, 1, 0, 1)])#(1,0)_left (0,1)_right


class Search:

    # Generates random double for left and right search speed
    def rand_double(self):
        global rand_double_left
        global rand_double_right
        rand_double_left = random.random()
        rand_double_right = random.random()

    # Given the input compared to the case script; where do we want to go?
    def calculate_search_speed(self, threshold_list,diff_angle):
        global counter
        global left_wheel_speed
        global right_wheel_speed
        counter = counter + 1
        for i in range(16):
            if threshold_list[0] == case_script[i][0] and threshold_list[1] == case_script[i][1] and threshold_list[
             2] == case_script[i][2] and threshold_list[3] == case_script[i][3]:
                if counter == COUNTLIMIT:
                    counter = 0
                    self.rand_double()
                if case_script[i][4] == case_script[i][5]:
                    left_wheel_speed = (diff_angle * 200) + 500
                    right_wheel_speed = (-1*diff_angle * 200) + 500
                elif case_script[i][4] == 1 and case_script[i][5] == 0:
                    left_wheel_speed = -300
                    right_wheel_speed = 700
                else:
                    left_wheel_speed = 700
                    right_wheel_speed = -300
                return

    # Calculate if there is an obstacle or not, depending on the threshold
    def calculate_threshold(self, sensors, distance_threshold, diff_angle):
        threshold_list = []
        for i in range(4):
            if sensors[i] > distance_threshold:
                threshold_list.append(1)
            else:
                threshold_list.append(0)
        self.calculate_search_speed(threshold_list, diff_angle)

    # Given the sensors input and threshold, calculates the speed for survival
    def update_search_speed(self, sensors_value, distance_threshold, diff_angle):
        sensors = [sensors_value[6], sensors_value[7], sensors_value[0], sensors_value[1]]
        self.calculate_threshold(sensors, distance_threshold, diff_angle)

    @staticmethod
    def get_search_left_wheel_speed():
        return left_wheel_speed

    @staticmethod
    def get_search_right_wheel_speed():
        return right_wheel_speed
