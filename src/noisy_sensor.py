#! /usr/bin/env python3
import random


class Sensor():

    error = 0.01 
    toggle_noise = 1.0

    def __init__(self, scan_msg):
        self.max_angle = scan_msg.angle_max
        self.min_angle = scan_msg.angle_min
        self.angle_increment = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.ranges = self.get_noisy_sensor(scan_msg.ranges)

    def get_noisy_sensor(self, ranges):
        noisy_ranges = []
        for r in ranges:
            noisy_r = r + random.uniform(-self.error, self.error) * self.toggle_noise
            noisy_ranges.append(noisy_r)
        
        return noisy_ranges
