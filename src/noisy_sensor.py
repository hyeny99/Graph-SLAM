#! /usr/bin/env python3
import random
import math
import numpy as np
import itertools as it

angle_max_limit = 0.523599
angle_min_limit = -0.523599
# range_max = 7.0



class Noisy_sensor():

   
    def __init__(self, scan_msg):
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        #print(self.angle_increment)

        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.ranges = add_error(scan_msg)



# def change_angle(angle):
#     if angle > math.radians(180):
#         return angle - math.radians(360)
#     return angle


def add_error(scan_msg):
        noisy_scan = []
        mean = 0
        sigma = 0.01
        toggle_noise = 1.0
        range_max = scan_msg.range_max
        
        # define the angle range: -30 degrees to +30 degrees
        angle_incre = scan_msg.angle_increment


        for i in it.chain(range(0, 31), range(330, 360)):
            # actual measurement
            #angle = scan_msg.angle_min + i * angle_incre
            #angle = define_angle(angle) 

            beam = scan_msg.ranges[i]
            if beam > range_max:
                noisy_scan.append(float('inf'))
            else:
                noisy_r = beam + np.random.normal(mean, sigma) * toggle_noise
                noisy_scan.append(noisy_r)
        
        
       
        
        return noisy_scan

def define_angle(angle):
    angle = math.degrees(angle) - 360.0
    return math.radians(angle)
