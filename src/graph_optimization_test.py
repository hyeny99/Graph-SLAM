#! /usr/bin/env python3
from graph_optimization import optimize_graph, plot_path
from graph import Graph, Vertex, Edge
import numpy as np

def test():
    graph = Graph()
    pose_i = np.array([0, 0, 0])
 
    vi = Vertex(pose_i, None)
    graph.add_vertex(vi)


    pose_j = np.array([0.7, 0, np.pi/2])

    vj = Vertex(pose_j, None)
    graph.add_vertex(vj)

    uij = np.array([1, 0, np.pi/3])

    edge = Edge(vi, vj, uij)
    graph.add_edges(edge)

    X = optimize_graph(graph)
    print("X", X)

    #plot_path()


if __name__ == "__main__":
    test()



