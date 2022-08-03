#! /usr/bin/env python3

from cProfile import label
import rospy
from sensor_msgs.msg import LaserScan
#from noisy_sensor import Noisy_sensor
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import itertools as it


import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir) 
import noisy_sensor

dt = 0.1

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



def get_noisy_x_y(pose, scan):
    x = pose[0]
    y = pose[1]
    yaw = pose[2]

    x_scans = []
    y_scans = []

    angle_min = scan.angle_min
    angle_incre = scan.angle_increment
    angles = list(it.chain(range(0, 31), range(330, 360)))


    for i in range(int(len(scan.ranges))):
        angle = angle_min + angle_incre * i
        if (scan.ranges[i] == float('inf')):
            continue
        x_scan = x + math.cos(math.radians(angles[i]) + yaw) * scan.ranges[i]
        y_scan = y + math.sin(math.radians(angles[i]) + yaw) * scan.ranges[i]

        x_scans.append(x_scan)
        y_scans.append(y_scan)
    
    
    return x_scans, y_scans    

def get_x_y(pose, scan):
    x = pose[0]
    y = pose[1]
    yaw = pose[2]

    x_scans = []
    y_scans = []

    angle_min = scan.angle_min
    angle_incre = scan.angle_increment


    for i in range(int(len(scan.ranges))):
        angle = angle_min + angle_incre * i
        if (scan.ranges[i] == float('inf')):
            continue
        x_scan = x + math.cos(angle + yaw) * scan.ranges[i]
        y_scan = y + math.sin(angle + yaw) * scan.ranges[i]

        x_scans.append(x_scan)
        y_scans.append(y_scan)
    
    
    return x_scans, y_scans    



if __name__== "__main__":
    rospy.init_node('rangefinder_test', anonymous=True)
    rate = rospy.Rate(1/dt)
    
    while not rospy.is_shutdown():
        odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=None)
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = get_rotation(odom_msg)


        scan_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        noisy_scan_msg = noisy_sensor.Noisy_sensor(scan_msg)

        x_scans, y_scans = get_x_y([x, y, yaw], scan_msg)
        x_noisy_scans, y_noisy_scans = get_noisy_x_y([x, y, yaw], noisy_scan_msg)

        # print("raw", x_scans, y_scans)
        # print("filtered", x_noisy_scans, y_noisy_scans)

        fig, ax = plt.subplots()
        ax.scatter(x_scans, y_scans, s=4, label='without noise')
        ax.scatter(x_noisy_scans, y_noisy_scans, s=4, label='with gaussian noise')
        ax.set_title('2D range finer gaussian noise model')
        ax.set_xlabel('x(cm)')
        ax.set_ylabel('y(cm)')
        ax.legend()
        plt.show()
        # plt.plot(x_scans, y_scans, label="raw sensor measurement")


    
    
        rate.sleep()
