#! /usr/bin/env python3
import numpy as np
import math
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import random


# toggle error
toggle_noise = 1.0

error = 0.01

# alpha parameters
# a1 = 0.05
# a2 = 15.0 * math.pi / 180.0
# a3 = 0.05
# a4 = 0.01

# # alpha parameters
a1 = 0.003                       # unitless
a2 = 0.007                      # rad^2 / m^2 (drift)
a3 = 0.003                      # unitless
a4 = 0.007                       # m^2 / rad^2 (moves forward/backward during a turn)

# Util ---------------------------------------------------------------------------------------------------------------------------

def get_delta(prev_odom, curr_odom):
    dx = curr_odom[0] - prev_odom[0]
    dy = curr_odom[1] - prev_odom[1]
  
    theta2 = curr_odom[2]
    theta1 = prev_odom[2]

    return dx, dy, theta1, theta2


def sample_gaussian(sigma):
    x1 = x2 = w = r = 0.0

    while True:
        # x1 = 2.0 * random.random() - 1.0
        # x2 = 2.0 * random.random() - 1.0
        while True:
            r = random.random()
            if r != 0.0:
                break
        
        x1 = 2.0 * r - 1.0
        while True:
            r = random.random()
            if r != 0.0:
                break
        x2 = 2.0 * r - 1.0
        w = x1**2 + x2**2

        if w < 1.0 and w != 0.0:
            break
    
    return sigma * x2 * math.sqrt(-2.0 * math.log(w) / w)

#---------------------------------------------------------------------------------------------------------------------------


def get_noisy_odom_1(prev_odom, curr_odom):
    dx, dy, theta1, theta2 = get_delta(prev_odom, curr_odom)

    trans = math.sqrt(dx**2 + dy**2)
    rot1  = math.atan2(dy, dx) - theta1
    rot2  = theta2 - theta1 - rot1

    sd_rot1 = a1 * abs(rot1) + a2 * abs(trans)
    sd_rot2 = a1 * abs(rot2) + a2 * abs(trans)
    sd_trans = a3 * trans + a4 * (abs(rot1) + abs(rot2))

    trans = trans + np.random.normal(0, sd_trans**2) * toggle_noise
    rot1 = rot1 + np.random.normal(0, sd_rot1**2) * toggle_noise
    rot2 = rot2 + np.random.normal(0, sd_rot2**2) * toggle_noise
    
    dx = trans * math.cos(theta1 + rot1)
    dy = trans * math.sin(theta1 + rot1)
    dyaw = rot1 + rot2

    return dx, dy, dyaw



def get_noisy_odom_2(prev_odom, curr_odom):
    dx, dy, theta1, theta2 = get_delta(prev_odom, curr_odom)

    trans = math.sqrt(dx**2 + dy**2)
    rot1  = math.atan2(dy, dx) - theta1
    rot2  = theta2 - theta1 - rot1

    sd_rot1 = a1 * (rot1**2) + a2 * (trans**2)
    sd_rot2 = a1 * (rot2**2) + a2 * (trans**2)
    sd_trans = a3 * (trans**2) + a4 * (rot1**2 + rot2**2)

    trans = trans + sample_gaussian(sd_trans) * toggle_noise
    rot1 = rot1 + sample_gaussian(sd_rot1) * toggle_noise
    rot2 = rot2 + sample_gaussian(sd_rot2) * toggle_noise
    
    dx = trans * math.cos(theta1 + rot1)
    dy = trans * math.sin(theta1 + rot1)
    dyaw = rot1 + rot2

    return dx, dy, dyaw



def get_simple_gaussian_noisy_odom(prev_odom, curr_odom):
    dx, dy, theta1, theta2 = get_delta(prev_odom, curr_odom)
    dyaw = theta2 - theta1

    dis_x = dx + np.random.normal(0, error) * toggle_noise
    dis_y = dy + np.random.normal(0, error) * toggle_noise
    dis_yaw = dyaw + np.random.normal(0, error) * toggle_noise

    return dis_x, dis_y, dis_yaw



# def get_uniform_noisy_odom(prev_odom, curr_odom):
#     dx, dy, theta1, theta2 = get_delta(prev_odom, curr_odom)
#     dyaw = theta2 - theta1

#     dis_x = dx + random.uniform(-error, error) * toggle_noise
#     dis_y = dy + random.uniform(-error, error) * toggle_noise
#     dis_yaw = dyaw + random.uniform(-error, error) * toggle_noise

#     return dis_x, dis_y, dis_yaw


# def plot(x, y):
#     fig, ax = plt.subplots()
#     ax.scatter(x, y)
#     ax.set_title('Gaussian noise of motion model')
#     plt.show()


# if __name__ == '__main__':
#     get_noisy_odom()