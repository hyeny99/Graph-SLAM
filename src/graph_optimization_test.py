#! /usr/bin/env python3
from graph_optimization import optimize_graph, plot_path, calculate_err
from graph import Graph, Vertex, Edge
import numpy as np

def test():
    graph = Graph()

    # actual poses (ground truth)
    x_0 = np.array([0, 0, 0]) 
    x_1 = np.array([1, 0, 0])
    x_2 = np.array([2, 0, 0])
    x_3 = np.array([3, 0, 0])
    x_4 = np.array([0, 0, 0])

    # measurements
    u_01 = np.array([1.1, 0, 0])
    u_12 = np.array([1.0, 0.0, 0.0])
    u_23 = np.array([1.1, 0, 0])
    u_34 = np.array([-2.7, 0, 0])
    u_40 = np.array([0, 0, 0])

    # noisy poses (by odom)
    p_0 = np.array([0, 0, 0])
    p_1 = np.array([1.1, 0.0, 0.0])
    p_2 = np.array([2.1, 0.0, 0.0])
    p_3 = np.array([3.2, 0, 0])
    p_4 = np.array([0.5, 0, 0])

    v0 = Vertex(p_0, None)
    v1 = Vertex(p_1, None)
    v2 = Vertex(p_2, None)
    v3 = Vertex(p_3, None)
    v4 = Vertex(p_4, None)

    graph.add_vertex(v0)
    graph.add_vertex(v1)
    graph.add_vertex(v2)
    graph.add_vertex(v3)
    graph.add_vertex(v4)
   

    edge = Edge(v0, v1, u_01)
    graph.add_edges(edge)
    edge = Edge(v1, v2, u_12)
    graph.add_edges(edge)
    edge = Edge(v2, v3, u_23)
    graph.add_edges(edge)
    edge = Edge(v3, v4, u_34)
    graph.add_edges(edge)
    edge = Edge(v4, v0, u_40)
    graph.add_edges(edge)

    #err = calculate_err(v4.pose, v0.pose, u_40)
    #print("err", err)


    ground = np.array([x_0, x_1, x_2, x_3, x_4]) # 2 x 3
    noisy = np.array([p_0, p_1, p_2, p_3, p_4]) # 2 x 3

    # err = calculate_err(vi.pose, vj.pose, uij)
    # print("err", err)

    

    X = optimize_graph(graph) # 2 x 3
    print("ground", ground)
    print("odom", noisy)
    print("X", X)
    #print("v1 pose", )



    plot_path(ground, noisy, X.T)


if __name__ == "__main__":
    test()



