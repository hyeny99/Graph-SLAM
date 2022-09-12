#! /usr/bin/env python3
import os
import sys
curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, curr_dir) 

import icp
import numpy as np


class Loop_closure():
    def __init__(self, first):
        self.first = first

    
    def detect_loop(self, second):
        _, distances, i, tolerance, is_converged = icp.icp(second, self.first, tolerance=5e-14)
        if is_converged and np.mean(distances) < 2.5:
            print("loop closure iterations", i)
            print("loop closure tolerance", tolerance)
            print("loop closure mean distances", np.mean(distances))
            return True
        
        return False