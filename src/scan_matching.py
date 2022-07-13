#! /usr/bin/env python3

# ---------------------------------------------------------------------#
# Scan matching and Loop closure algorithms
# Author: Hyewon Jeon
# ---------------------------------------------------------------------#

# ICP algorithm

#from sklearn.neighbors import NearestNeighbors, KNearest
import numpy as np
from graph import Vertex, Edge, Graph
import sys
import cv2


threadshold = 0.2
convergence_flag = False

def uniform_subsampling(scan_data):
# raw data has 60 points (x, y) from -30 degrees to +30 degrees
# error of a range finder is uniform over the distance
# It should be noted that the accuracy error of the laser rangefinder is not proportional to the measurement distance, 
# the entire distance is the same, but if the distance is too long, the error will increase by ±0.5mm/100m.

    length = len(scan_data)
    x = []
    y = []
    for i in range(0, length, 2):
        x.append(scan_data[0][i])
        print("x: " + str(scan_data[0][i]))
        y.append(scan_data[1][i])

    subsampling = np.array([x, y])
    
    return subsampling


# def nearest_neighbours():
#     nbrs = 

def del_miss(indices, dist, max_dist, th_rate = 0.8):
    th_dist = max_dist * th_rate
    return np.array([indices[0][np.where(dist.T[0] < th_dist)]])

def is_converge(Tr, scale):
    delta_angle = 0.0001
    delta_scale = scale * 0.0001
    
    min_cos = 1 - delta_angle
    max_cos = 1 + delta_angle
    min_sin = -delta_angle
    max_sin = delta_angle
    min_move = -delta_scale
    max_move = delta_scale
    
    return min_cos < Tr[0, 0] and Tr[0, 0] < max_cos and \
           min_cos < Tr[1, 1] and Tr[1, 1] < max_cos and \
           min_sin < -Tr[1, 0] and -Tr[1, 0] < max_sin and \
           min_sin < Tr[0, 1] and Tr[0, 1] < max_sin and \
           min_move < Tr[0, 2] and Tr[0, 2] < max_move and \
           min_move < Tr[1, 2] and Tr[1, 2] < max_move



def icp(a: Vertex, b: Vertex, init_pose, n_iteration):
    #print(a.scan_data)
    data_a = np.array(uniform_subsampling(a.scan_data))
    print(len(data_a))
    data_b = np.array(uniform_subsampling(b.scan_data))

    src = np.array([data_a.T], copy=True).astype(np.float32)
    #print(len(src))
    #print(src)
    dst = np.array([data_b.T], copy=True).astype(np.float32)

    #knn = NearestNeighbors(n_neighbors=1, algorithm='auto')
    knn = cv2.ml.KNearest_create()
    responses = np.array(range(len(data_b[0]))).astype(np.float32)
    knn.train((src[0]), responses)

    Tr = np.array([[np.cos(init_pose[0]), -np.sin(init_pose[1]), 0],
                   [np.sin(init_pose[0]), np.cos(init_pose[1]),  0],
                   [0,         0,          1]])

    dst = cv2.transform(dst, Tr[0:2])
    max_err = sys.maxint

    scale_x = np.max(data_a[0]) - np.min(data_a[0])
    scale_y = np.max(data_a[1]) - np.min(data_a[1])
    scale = max(scale_x, scale_y)

    for i in range(n_iteration):
        ret, results, neighbours, dist = knn.find_nearest(dst[0], 1)
        
        indeces = results.astype(np.int32).T     
        indeces = del_miss(indeces, dist, max_dist)  
        
        T = cv2.estimateRigidTransform(dst[0, indeces], src[0, indeces], True)

        max_dist = np.max(dist)
        dst = cv2.transform(dst, T)
        Tr = np.dot(np.vstack((T,[0,0,1])), Tr)
        
        if (is_converge(T, scale)):
            break
        
    return Tr[0:2]


# def transfer(x, y):



    





