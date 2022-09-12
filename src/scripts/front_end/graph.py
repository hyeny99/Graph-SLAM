#! /usr/bin/env python3
import math
from copy import deepcopy
import numpy as np
import itertools as it


class Vertex():
    verticies = []
    def __init__(self, pose, scan_data):
        self.pose = pose
        self.scan_data = scan_data
        self.x_y_data = x_y_data(pose, scan_data)
        

class Edge():
    def __init__(self, vi:Vertex, vj:Vertex, uij):
        self.vi = vi
        self.vj = vj
        self.uij = uij
    # def update_uij(self, uij):
    #     self.uij = uij



class Graph():
    verticies:Vertex = []
    edges:Edge = []
    def __init__(self):
        pass

    def add_vertex(self, vertex):
        self.verticies.append(vertex)

    def add_edges(self, edge):
        self.edges.append(edge)
    
    def update_scan_data(self, vertex:Vertex, x_y_data):
        vertex.x_y_data = x_y_data
        return
    
    def update_vertex_pose(self, vertex:Vertex, pose):
        vertex.pose = pose
    
    def get_index_vertex(self, vertex):
        for i in range(len(self.verticies)):
            if np.all(vertex.pose == self.verticies[i].pose):
                return i
        
        return None
    



def x_y_data(pose, scan_msg):
    if scan_msg == None:
        return
    data = []
    yaw = pose[2]
    resolution = 0.01
    x = int(pose[0] / resolution)
    y = int(pose[1] / resolution)

    ranges = scan_msg.ranges
    angle_min = scan_msg.angle_min
    angle_incre = scan_msg.angle_increment


    for i in range(len(ranges)):
        angle = angle_min + angle_incre * i
        t_yaw = yaw + angle
        beam = ranges[i]
        if beam == float('inf'):
            beam = 0
        beam_x = int(math.cos(t_yaw) * (beam / resolution)) + x
        beam_y = int(math.sin(t_yaw) * (beam / resolution)) + y
        data.append([beam_x, beam_y])

        
    return data
