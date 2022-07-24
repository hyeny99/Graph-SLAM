#! /usr/bin/env python3
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib
import matplotlib.pyplot as plt
import random


# toggle error
toggle_noise = 1.0

#error = 0.01

# alpha parameters
a1 = 0.01
a2 = 5.0 * math.pi / 180.0
a3 = 0.01
a4 = 0.01



def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    return yaw

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

# def set_prev_odom(odom):
#     pose[0] = odom.pose.pose.position.x
#     pose[1] = odom.pose.pose.position.y
#     pose[2] = get_rotation(odom)
#     return pose

# def get_old_noisy_odom():
#     # curr_pos = rospy.wait_for_message("/odom", Odometry, timeout=None)
#     #print("it's working")
#     #curr_yaw = get_rotation(odom)

#     x = []
#     y = []
    
#     for i in range(1000):
#         dis_x = dx + random.uniform(-error, error) * toggle_noise
#         dis_y = dy + random.uniform(-error, error) * toggle_noise
#         d_yaw = theta1 + theta2 + random.uniform(-error, error) * toggle_noise
#         x.append(dis_x)
#         y.append(dis_y)
    
#     plot(x, y)
    
#     return pose[0] + dis_x, pose[1] + dis_y, pose[2] + d_yaw



def get_noisy_odom(prev_pose, curr_odom):
    dx = curr_odom.pose.pose.position.x - prev_pose[0]
    dy = curr_odom.pose.pose.position.y - prev_pose[1]
  
    theta2 = get_rotation(curr_odom)
    theta1 = prev_pose[2]

    trans = math.sqrt(dx**2 + dy**2)
    rot1  = math.atan2(dy, dx) - theta1
    rot2  = theta2 - theta1 - rot1

    sd_rot1 = a1 * abs(rot1) + a2 * trans
    sd_rot2 = a1 * abs(rot2) + a2 * trans
    sd_trans = a3 * trans + a4 * (abs(rot1) + abs(rot2))

    trans = trans + np.random.normal(0, sd_trans**2) * toggle_noise
    rot1 = rot1 + np.random.normal(0, sd_rot1**2) * toggle_noise
    rot2 = rot2 + np.random.normal(0, sd_rot2**2) * toggle_noise
    
    dx = trans * math.cos(theta1 + rot1)
    dy = trans * math.sin(theta1 + rot1)
    dyaw = rot1 + rot2

    return dx, dy, dyaw


def plot(x, y):
    fig, ax = plt.subplots()
    ax.scatter(x, y)
    ax.set_title('Gaussian noise of motion model')
    plt.show()


# if __name__ == '__main__':
#     get_noisy_odom()