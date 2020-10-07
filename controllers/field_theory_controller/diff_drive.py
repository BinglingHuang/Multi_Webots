"""different velocity drive model"""
import numpy
import math
class diff_drive:
    def __init__(self):
        ###########--------parameter of the epuck-----------##########
        self.wheel_diameter = 0.04
        self.robot_diameter = 0.074
        self.v = 0
        self.w = 0
        self.wl = 0
        self.wr = 0

    def forward_kinematic(self, wl, wr):
        self.wl = wl
        self.wr = wr
        self.v = self.wheel_diameter / 2 * (self.wl + self.wr)
        self.w = self.wheel_diameter / self.robot_diameter * (self.wr - self.wl)

    def inverse_kinematic(self, v, w):
        self.v = v
        self.w = w
        self.wl = 1 / self.wheel_diameter * (self.v - self.w * self.robot_diameter / 2)
        self.wr = 1 / self.wheel_diameter * (self.v + self.w * self.robot_diameter / 2)
