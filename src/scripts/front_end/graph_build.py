import os
import sys

curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, curr_dir) 

from copy import deepcopy
import numpy as np
from graph import Graph, Vertex, Edge

sys.path.append("..") 
from scan_matching import icp



def init_graph():
    global graph
    graph = Graph()

def get_graph():
    return graph


def build_graph(scan, pose):
    vi = graph.verticies[len(graph.verticies) - 1]
    vj = deepcopy(create_vertex(scan, pose))
    edge = create_edge(vi, vj)

    graph.add_vertex(vj)
    graph.add_edges(edge)

    scan_matching(vi, vj)

    return graph



def create_vertex(scan, pose):
    v = Vertex(pose, scan)
    graph.add_vertex(v)
    return v


def create_edge(vi, vj):
    uij = np.subtract(vj.pose, vi.pose)
    edge = Edge(vi, vj, uij)
    return edge


def scan_matching(vi, vj):
    A = np.array(vi.x_y_data) # destination
    B = np.array(vj.x_y_data) # source

    T, distances, i, tolerance, _ = icp.icp(B, A, tolerance=0.0001)
    B_trans = np.ones((len(vj.scan_data.ranges), 3))
    B_trans[:,0:2] = np.copy(B) # [[x,y], [x,y],...]
    B_trans = np.dot(T[0:2], B_trans.T).T.astype(int)  # change v2 scan data
    graph.update_scan_data(vj, list(B_trans))
            
    print("iterations", i)
    print("tolerance", tolerance)
    print("mean distances", np.mean(distances))
            
    assert np.all(np.array(vj.x_y_data) == np.array(B_trans))
    


