#! /usr/bin/env python3
import numpy as np
import itertools as it

class Noisy_sensor():

    def __init__(self, scan_msg):
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.raw_ranges = scan_msg.ranges
        self.ranges = self.add_error()

      
    def add_error(self):
        noisy_scan = []
        mean = 0
        sigma = 0.01
        toggle_noise = 1.0
        
        range_max = self.range_max
        ranges = self.raw_ranges

        for r in ranges:
            if r > range_max:
                noisy_scan.append(float('inf'))
            else:
                noisy_r = r + np.random.normal(mean, sigma) * toggle_noise
                noisy_scan.append(noisy_r)
        
        return noisy_scan


