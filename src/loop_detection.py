#! /usr/bin/env python3
import icp
import numpy as np

class Loop_closure():
    def __init__(self, first):
        self.first = first

    
    def detect_loop(self, second):
        _, distances, i, tolerance, is_converged = icp.icp(second, self.first, tolerance=1e-14)
        if is_converged and np.mean(distances) < 3.0:
            print("loop closure iterations", i)
            print("loop closure tolerance", tolerance)
            print("loop closure mean distances", np.mean(distances))
            return True
        
        return False