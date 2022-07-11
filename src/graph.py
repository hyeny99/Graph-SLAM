#! /usr/bin/env python3

class Vertex():
    verticies = []
    def __init__(self, pose, scan_data):
        self.pose = pose
        self.scan_data = scan_data
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


