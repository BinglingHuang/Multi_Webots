import numpy as np
import math
from calculation import *
from field_theory_Gaussian import Gaussian_field

a = Gaussian_field(e_puck_position=(20, 20, 0), box_position=(0, 0, 0),BOX_D=10, converge_list=[1,0,0,1])

print(a)
