#! /usr/bin/env python3
import numpy as np


err = 0.01

class Node():
    def __init__(self, pose):
        self._pose = pose


class Edge():
    def __init__(self, node_1:Node, node_2:Node, scan):
        self._nodei = node_1
        self._nodej = node_2
        self._scan = scan
        
    
    def get_err_info(self, z_pred, z_actual):
        self._err = z_actual - z_pred
        cov_matrix = np.matrix([[err**2, 0, 0],
                                [0, err**2, 0],
                                [0, 0, err**2]])
        self._info_matrix = cov_matrix.I

class Graph():
    edges = []
    nodes = []

    def __init__(self):
        print("Graph created")
    
    def add_edge(self, edge:Edge):
        Graph.edges.append(edge)
    
    def add_node(self, node:Node):
        Graph.nodes.append(node)
        

        
    