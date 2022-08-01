#! /usr/bin/env python3
import icp
import numpy as np

class Loop_closure():
    def __init__(self, first):
        self.first = first

    
    def detect_loop(self, second):
        _, distances, _, is_converged = icp.icp(second, self.first, tolerance=0.000000001)
        if is_converged and np.mean(distances) < 0.05:
            return True
        
        return False