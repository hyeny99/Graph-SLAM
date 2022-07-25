#! /usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import random


# initial pose
x = 0.1
y = 0.1
theta1 = 25.0 * math.pi / 180.0 # in rad

# dx, dy, dyaw
dx = 0.05
dy = 0.15
theta2 = 10.0 * math.pi / 180.0

# alpha parameters
a1 = 0.005                       # unitless
a2 = 0.005                     # rad^2 / m^2 (drift)
a3 = 0.005                       # unitless
a4 = 0.005                      # m^2 / rad^2 (moves forward/backward during a turn)




def get_old_noisy_odom():
    error = 0.01
    toggle_noise = 1.0

    x_trans = []
    y_trans = []
    
    for i in range(1000):
        dis_x = dx + random.uniform(-error, error) * toggle_noise
        dis_y = dy + random.uniform(-error, error) * toggle_noise
        d_yaw = theta1 + theta2 + random.uniform(-error, error) * toggle_noise
        x_trans.append(x + dis_x)
        y_trans.append(y + dis_y)
    
    fig, ax = plt.subplots()
    ax.scatter(x_trans, y_trans)
    plt.show()
    
   



def test_noisy_odom():
    trans = math.sqrt(dx**2 + dy**2)
    rot1  = math.atan2(dy, dx) - theta1
    rot2  = theta2 - theta1 - rot1

    sd_rot1 = a1 * (rot1**2) + a2 * (trans**2)
    sd_rot2 = a1 * (rot2**2) + a2 * (trans**2)
    sd_trans = a3 * (trans**2) + a4 * (rot1**2 + rot2**2)
    # sd_rot1 = a1 * abs(rot1) + a2 * abs(trans)
    # sd_rot2 = a1 * abs(rot2) + a2 * abs(trans)
    # sd_trans = a3 * trans + a4 * (abs(rot1) + abs(rot2))
    
    x_no_noise = x + math.cos(theta1 + rot1) * math.sqrt(dx**2 + dy**2)
    y_no_noise = y + math.sin(theta1 + rot1) * math.sqrt(dx**2 + dy**2)
    
    x_noise = []
    y_noise = []

    for i in range(1000):
        # t = trans + np.random.normal(0, sd_trans**2) 
        # r1 = rot1 + np.random.normal(0, sd_rot1**2)
        # r2 = rot2 + np.random.normal(0, sd_rot2**2)

        t = trans + sample_gaussian(sd_trans)
        r1 = rot1 + sample_gaussian(sd_rot1)
        r2 = rot2 + sample_gaussian(sd_rot2)

        # t = trans + np.random.normal(0, sd_trans) 
        # r1 = rot1 + np.random.normal(0, sd_rot1)
        # r2 = rot2 + np.random.normal(0, sd_rot2)

        x_noise.append(x + t * math.cos(theta1 + r1))
        y_noise.append(y + t * math.sin(theta1 + r1))
        # yaw = yaw + r1 + r2


    fig, ax = plt.subplots()
    # ax.set_xlim([0.05, 0.2])
    # ax.set_ylim([0.05, 0.3])
    ax.scatter(x_noise, y_noise, color='blue', s=20, label="with Gaussian noise")
    ax.scatter(x_no_noise, y_no_noise, color='hotpink', s=20, label="without noise")
    ax.set_title('Gaussian noise of motion model')
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.legend()

    # path = '../images/leg_stretched.png'
    # offset = OffsetImage(plt.imread(path), zoom=0.08)
    # ab = AnnotationBbox(offset, (x, y), frameon=False)
    # ax.add_artist(ab)
    plt.show()


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
        
    
    #print("w", w)
    
    return sigma * x2 * math.sqrt(-2.0 * math.log(w) / w)
    
    


if __name__ == '__main__':
    #test_noisy_odom()
    get_old_noisy_odom()