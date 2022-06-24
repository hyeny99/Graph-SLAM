#! /usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

# Config parameters
dt = 0.01 # time between measurements

# sensor noise
toggle_noise = 1
error = 0.01 # std_dev

# Utils 
# ----------------------------------------------------------------------------------------------------------
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

def normalize_angle(angle):
    if angle < 0:
        return angle+np.ceil(-angle/(2*np.pi))*2*np.pi
    elif angle > 2*np.pi:
        return angle-(np.ceil(angle/(2*np.pi))-1)*2*np.pi

    return angle 

# Give the previous measurements and it will count displacement
def get_noisy_odometry(x,y,yaw):
    curr_pos = rospy.wait_for_message("/odom", Odometry, timeout=None)
    curr_yaw = get_rotation(curr_pos)
    dis_x = curr_pos.pose.pose.position.x-x + random.uniform(-error, error) * toggle_noise
    dis_y = curr_pos.pose.pose.position.y-y + random.uniform(-error, error) * toggle_noise
    d_yaw = curr_yaw-yaw + random.uniform(-error, error) * toggle_noise

    return dis_x, dis_y, d_yaw

def get_noisy_measurement(scan_msg):
    ranges = scan_msg.ranges
    noisy_scan = []
    for r in ranges:
        noisy_r = r + random.uniform(-error, error) * toggle_noise
        noisy_scan.append(r)
    
    return noisy_scan, scan_msg
# ----------------------------------------------------------------------------------------------------------

if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("slam", anonymous=True)
    # Define ROS rate
    rate = rospy.Rate(1/dt)

    previous_position =  rospy.wait_for_message("/odom", Odometry, timeout=None)
    prev_x = previous_position.pose.pose.position.x
    prev_y = previous_position.pose.pose.position.y
    prev_yaw = get_rotation(previous_position)


    while not rospy.is_shutdown():
        # Measurements (odom)
        dx,dy,dyaw   = get_noisy_odometry(prev_x, prev_y, prev_yaw)
        prev_x       = prev_x + dx
        prev_y       = prev_y + dy
        prev_yaw     = prev_yaw + dyaw
        
        # Measurements (laser)
        scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
        noisy_sensor = get_noisy_measurement(scan_msg)


        rate.sleep()


