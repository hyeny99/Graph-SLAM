#! /usr/bin/env python3
import math
from copy import deepcopy

class Vertex():
    verticies = []
    def __init__(self, pose, scan_data):
        self.pose = pose
        self.scan_data = scan_data
        #x_y_data = Noisy_sensor.x_y_data(pose=pose)
        self.x_y_data = x_y_data(pose, scan_data)
        #self.verticies.append(self)
        

class Edge():
    def __init__(self, v1, v2):
        self.v1 = v1
        self.v2 = v2


class Graph():
    verticies = []
    edges = []
    def __init__(self):
        pass

    def add_vertex(self, vertex):
        self.verticies.append(vertex)

    def add_edges(self, edge):
        self.edges.append(edge)
    
    def update_scan_data(self, vertex:Vertex, x_y_data):
        vertex.x_y_data = x_y_data
        return



def x_y_data(pose, scan_msg):
    data = []
    x = pose[0,0]
    y = pose[1,0]
    yaw = pose[2,0]

    ranges = scan_msg.ranges
    angle_min = scan_msg.angle_min
    angle_incre = scan_msg.angle_increment

    for i in range(len(ranges)):
        angle = angle_min + angle_incre * i
        t_yaw = yaw + angle
        beam = ranges[i]
        if beam == float('inf'):
            beam = 0
        beam_x = x + math.cos(t_yaw) * beam
        beam_y = y + math.sin(t_yaw) * beam
        data.append([beam_x, beam_y])

        
    return data
