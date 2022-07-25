#! /usr/bin/env python3
import random
import math
import numpy as np

angle_max_limit = -0.523599
angle_min_limit = 0.523599
range_max = 7.0


class Noisy_sensor():

   
    def __init__(self, scan_msg):
        self.angle_min = angle_min_limit
        self.angle_max = angle_max_limit
        self.angle_increment = scan_msg.angle_increment
        #print(self.angle_increment)

        self.range_min = scan_msg.range_min
        self.range_max = range_max
        self.ranges = add_error(scan_msg)



# def change_angle(angle):
#     if angle > math.radians(180):
#         return angle - math.radians(360)
#     return angle


def add_error(scan_msg):
        noisy_scan = []
        error = 0.01
        toggle_noise = 0.0
        
        # define the angle range: -30 degrees to +30 degrees
        angle_incre = scan_msg.angle_increment

        for i in range(len(scan_msg.ranges)):
            # actual measurement
            angle = scan_msg.angle_min + i * angle_incre 
            if angle > angle_min_limit or angle < angle_max_limit:
                continue

            beam = scan_msg.ranges[i]
            if beam > range_max:
                noisy_scan.append(float('inf'))
            else:
                noisy_r = beam + random.uniform(-error, error) * toggle_noise
                noisy_scan.append(noisy_r)
        
        return noisy_scan
