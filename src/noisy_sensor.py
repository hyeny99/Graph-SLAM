#! /usr/bin/env python3
import random
import math
import numpy as np

class Noisy_sensor():

    angle_max_limit = 0.523599
    angle_min_limit = -0.523599


    def __init__(self, scan_msg):
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_incre = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.ranges = self.add_error(scan_msg.ranges, self.angle_incre)

    def add_error(self, ranges, angle_increment):
        noisy_scan = []
        error = 0.01
        toggle_noise = 1.0
        
        # define the angle range: -30 degrees to +30 degrees
        angle_incre = angle_increment


        for i in range(len(ranges)):
            # actual measurement
            angle = change_angle(self.angle_min + i * angle_incre) 
            if angle < self.angle_min or angle > self.angle_max:
                continue

            noisy_r = ranges[i] + random.uniform(-error, error) * toggle_noise
            noisy_scan.append(noisy_r)

    def x_y_data(self, pose):
        data_x = []
        data_y = []
        x = pose[0,0]
        y = pose[1,0]
        yaw = pose[2,0]

        ranges = self.ranges
        angle_min = self.angle_min
        angle_incre = self.angle_incre

        #print(ranges)

        for i in range(len(ranges)):
            angle = angle_min + angle_incre * i
            t_yaw = yaw + angle

            beam_x = x + math.cos(t_yaw) * ranges[i]
            beam_y = y + math.sin(t_yaw) * ranges[i]
            data_x.append(beam_x)
            data_y.append(beam_y)

        data = np.array([data_x, data_y])
        
        return data



def change_angle(angle):
    if angle > math.radians(180):
        return angle - math.radians(360)
    return angle