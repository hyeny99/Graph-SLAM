#! /usr/bin/env python3
from graph_optimization import optimize_graph, plot_mean_err, plot_path, calculate_err
from graph import Graph, Vertex, Edge
import numpy as np
from math import radians

def test_simple():
    graph = Graph()

<<<<<<< HEAD


    x_0 = np.array([0, 0, 0]) 
    x_1 = np.array([1, 1, 0])
    x_2 = np.array([2, 3, 0])
    x_3 = np.array([3, 4, 0])
    x_4 = np.array([0, 0, 0])

    # measurements
    u_01 = np.array([1.1, 1.1, 0])
    u_12 = np.array([1.0, 1.9, 0.0])
    u_23 = np.array([1.1, 1.1, 0])
    u_34 = np.array([-2.7, -3.8, 0])
    u_40 = np.array([0, 0, 0])

    # noisy poses (by odom)
    p_0 = np.array([0, 0, 0])
    p_1 = np.array([1.1, 1.1, 0.0])
    p_2 = np.array([2.1, 3.0, 0.0])
    p_3 = np.array([3.2, 4.1, 0])
    p_4 = np.array([0.5, 0.3, 0])
=======
    # actual poses (ground truth)
    x_0 = np.array([15., 15., 0.]) 
    x_1 = np.array([15.00265393, 14.9991644, -0.43602482])
    x_2 = np.array([15.06220433, 14.9077783, -1.00980009])
    x_3 = np.array([15.17420529, 14.73218362, -1.00613437])
    x_4 = np.array([15.21291986, 14.68368857, -0.62018425])
    x_5 = np.array([15.25183201, 14.6690675,   0.04987883])
    x_6 = np.array([15.38470688, 14.6908947,   0.48330871])
    x_7 = np.array([15.43933986, 14.75385049,  0.99216444])
    x_8 = np.array([15.49705066, 14.87334905,  1.3929219])
    x_9 = np.array([15.49184818, 14.96697988,  1.86561174])
    x_10 = np.array([15.43758571, 15.05345461,  2.3982945])
    x_11 = np.array([15.35957597, 15.09315271,  2.89214192])
    x_12 = np.array([15.26493044, 15.09765592, -3.1017871])
    x_13 = np.array([15.07013387, 15.08938166, -3.09405688])
    x_14 = np.array([14.99105506, 15.08566532,  3.02450873])
    x_15 = np.array([14.99916207, 15.09064176, -2.18244214])
    x_16 = np.array([15.00379455, 15.01704172, -1.54360329])
    #x_17 = np.array([15.00216419, 14.99221539, -0.84110497])
    x_17 = np.array([15., 15., 0.]) 
    
    
    # noisy poses (by odom)
    p_0 = np.array([15.,         15.,          0.])
    p_1 = np.array([15.01507568, 15.00330925, -0.42649871])
    p_2 = np.array([15.06045818, 14.90550995, -1.0326606])
    p_3 = np.array([15.1869545,  14.72813225, -1.02700365])
    p_4 = np.array([15.21125698, 14.67980576, -0.61124688])
    p_5 = np.array([15.2580471,  14.68680382,  0.05667137])
    p_6 = np.array([15.38594341, 14.6890583 ,  0.48169428])
    p_7 = np.array([15.44649029, 14.75174236,  0.99233633])
    p_8 = np.array([15.50089455, 14.88170719,  1.40287828])
    p_9 = np.array([15.51630116, 14.97918415,  1.87136686])
    p_10 = np.array([15.44171906, 15.06013393,  2.38993907])
    p_11 = np.array([15.35223579, 15.09278297,  2.89758468])
    p_12 = np.array([15.26709461, 15.10097027, -3.11276412])
    p_13 = np.array([15.06439018, 15.09509087, -3.08760095])
    p_14 = np.array([14.97629261, 15.07715511,  3.0056169 ])
    p_15 = np.array([15.00144863, 15.08319283, -2.17473793])
    p_16 = np.array([14.97014141, 15.02025509, -1.54403925])
    p_17 = np.array([14.92888134, 14.90716545, -0.829862])
 
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

    v0 = Vertex(p_0, None)
    v1 = Vertex(p_1, None)
    v2 = Vertex(p_2, None)
    v3 = Vertex(p_3, None)
    v4 = Vertex(p_4, None)
    v5 = Vertex(p_5, None)
    v6 = Vertex(p_6, None)
    v7 = Vertex(p_7, None)
    v8 = Vertex(p_8, None)
    v9 = Vertex(p_9, None)
    v10 = Vertex(p_10, None)
    v11 = Vertex(p_11, None)
    v12 = Vertex(p_12, None)
    v13 = Vertex(p_13, None)
    v14 = Vertex(p_14, None)
    v15 = Vertex(p_15, None)
    v16 = Vertex(p_16, None)
    v17 = Vertex(p_17, None)

    graph.add_vertex(v0)
    graph.add_vertex(v1)
    graph.add_vertex(v2)
    graph.add_vertex(v3)
    graph.add_vertex(v4)
    graph.add_vertex(v5)
    graph.add_vertex(v6)
    graph.add_vertex(v7)
    graph.add_vertex(v8)
    graph.add_vertex(v9)
    graph.add_vertex(v10)
    graph.add_vertex(v11)
    graph.add_vertex(v12)
    graph.add_vertex(v13)
    graph.add_vertex(v14)
    graph.add_vertex(v15)
    graph.add_vertex(v16)
    graph.add_vertex(v17)
   

    edge = Edge(v0, v1, np.subtract(p_1, p_0))
    graph.add_edges(edge)
    edge = Edge(v1, v2, np.subtract(p_1, p_2))
    graph.add_edges(edge)
    edge = Edge(v2, v3, np.subtract(p_3, p_2))
    graph.add_edges(edge)
    edge = Edge(v3, v4, np.subtract(p_4, p_3))
    graph.add_edges(edge)
    edge = Edge(v4, v5, np.subtract(p_5,  p_4))
    graph.add_edges(edge)
    edge = Edge(v5, v6, np.subtract(p_6, p_5))
    graph.add_edges(edge)
    edge = Edge(v6, v7, np.subtract(p_7, p_6))
    graph.add_edges(edge)
    edge = Edge(v7, v8, np.subtract(p_8, p_7))
    graph.add_edges(edge)
    edge = Edge(v8, v9, np.subtract(p_9, p_8))
    graph.add_edges(edge)
    edge = Edge(v9, v10, np.subtract(p_10, p_9))
    graph.add_edges(edge)
    edge = Edge(v10, v11, np.subtract(p_11, p_10))
    graph.add_edges(edge)
    edge = Edge(v11, v12, np.subtract(p_12, p_11))
    graph.add_edges(edge)
    edge = Edge(v12, v13, np.subtract(p_13, p_12))
    graph.add_edges(edge)
    edge = Edge(v13, v14, np.subtract(p_14, p_13))
    graph.add_edges(edge)
    edge = Edge(v14, v15, np.subtract(p_15, p_14))
    graph.add_edges(edge)
    edge = Edge(v15, v16, np.subtract(p_16, p_15))
    graph.add_edges(edge)
    edge = Edge(v16, v17, np.subtract(p_17, p_16))
    graph.add_edges(edge)

    # loop detected
    edge = Edge(v0, v17, np.array([0,0,0]))
    graph.add_edges(edge)

    #err = calculate_err(v4.pose, v0.pose, u_40)
    #print("err", err)


    ground = np.array([x_0, x_1, x_2, x_3, x_4, x_5, x_6, x_7, x_8, x_9, x_10, x_11, x_12, x_13, x_14, x_15, x_16, x_17, x_0]) # 2 x 3
    noisy = np.array([p_0, p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8, p_9, p_10, p_11, p_12, p_13, p_14, p_15, p_16, p_17, p_0]) # 2 x 3

    print("v1 before:", v1.pose)
    print("graph v1 before", graph.verticies[1].pose)
    print("edge v1 before", graph.edges[1].vi.pose)
    

    X, mean_err = optimize_graph(graph) # 2 x 3
    #plot_mean_err(mean_err)



<<<<<<< HEAD


    plot_path(ground, noisy, X)


def test():
    graph = Graph()
 # actual poses (ground truth)
    x_0 = np.array([15., 15., 0.]) 
    x_1 = np.array([15.00265393, 14.9991644, -0.43602482])
    x_2 = np.array([15.06220433, 14.9077783, -1.00980009])
    x_3 = np.array([15.17420529, 14.73218362, -1.00613437])
    x_4 = np.array([15.21291986, 14.68368857, -0.62018425])
    x_5 = np.array([15.25183201, 14.6690675,   0.04987883])
    x_6 = np.array([15.38470688, 14.6908947,   0.48330871])
    x_7 = np.array([15.43933986, 14.75385049,  0.99216444])
    x_8 = np.array([15.49705066, 14.87334905,  1.3929219])
    x_9 = np.array([15.49184818, 14.96697988,  1.86561174])
    x_10 = np.array([15.43758571, 15.05345461,  2.3982945])
    x_11 = np.array([15.35957597, 15.09315271,  2.89214192])
    x_12 = np.array([15.26493044, 15.09765592, -3.1017871])
    x_13 = np.array([15.07013387, 15.08938166, -3.09405688])
    x_14 = np.array([14.99105506, 15.08566532,  3.02450873])
    x_15 = np.array([14.99916207, 15.09064176, -2.18244214])
    x_16 = np.array([15.00379455, 15.01704172, -1.54360329])
    #x_17 = np.array([15.00216419, 14.99221539, -0.84110497])
    x_17 = np.array([15., 15., 0.]) 
=======
    # x_0 = np.array([0, 0, 0]) 
    # x_1 = np.array([1, 0, 0])
    # x_2 = np.array([2, 0, 0])
    # x_3 = np.array([3, 0, 0])
    # x_4 = np.array([0, 0, 0])

    # # measurements
    # u_01 = np.array([1.1, 0, 0])
    # u_12 = np.array([1.0, 0.0, 0.0])
    # u_23 = np.array([1.1, 0, 0])
    # u_34 = np.array([-2.7, 0, 0])
    # u_40 = np.array([0, 0, 0])

    # # noisy poses (by odom)
    # p_0 = np.array([0, 0, 0])
    # p_1 = np.array([1.1, 0.0, 0.0])
    # p_2 = np.array([2.1, 0.0, 0.0])
    # p_3 = np.array([3.2, 0, 0])
    # p_4 = np.array([0.5, 0, 0])

    # v0 = Vertex(p_0, None)
    # v1 = Vertex(p_1, None)
    # v2 = Vertex(p_2, None)
    # v3 = Vertex(p_3, None)
    # v4 = Vertex(p_4, None)

    # graph.add_vertex(v0)
    # graph.add_vertex(v1)
    # graph.add_vertex(v2)
    # graph.add_vertex(v3)
    # graph.add_vertex(v4)
   

    # edge = Edge(v0, v1, u_01)
    # graph.add_edges(edge)
    # edge = Edge(v1, v2, u_12)
    # graph.add_edges(edge)
    # edge = Edge(v2, v3, u_23)
    # graph.add_edges(edge)
    # edge = Edge(v3, v4, u_34)
    # graph.add_edges(edge)
    # edge = Edge(v4, v0, u_40)
    # graph.add_edges(edge)

    # #err = calculate_err(v4.pose, v0.pose, u_40)
    # #print("err", err)


    # ground = np.array([x_0, x_1, x_2, x_3, x_4]) # 2 x 3
    # noisy = np.array([p_0, p_1, p_2, p_3, p_4]) # 2 x 3

    print("v1 before:", v1.pose)
    print("graph v1 before", graph.verticies[1].pose)
    print("edge v1 before", graph.edges[1].vi.pose)
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a
    
    
    # noisy poses (by odom)
    p_0 = np.array([15.,         15.,          0.])
    p_1 = np.array([15.01507568, 15.00330925, -0.42649871])
    p_2 = np.array([15.06045818, 14.90550995, -1.0326606])
    p_3 = np.array([15.1869545,  14.72813225, -1.02700365])
    p_4 = np.array([15.21125698, 14.67980576, -0.61124688])
    p_5 = np.array([15.2580471,  14.68680382,  0.05667137])
    p_6 = np.array([15.38594341, 14.6890583 ,  0.48169428])
    p_7 = np.array([15.44649029, 14.75174236,  0.99233633])
    p_8 = np.array([15.50089455, 14.88170719,  1.40287828])
    p_9 = np.array([15.51630116, 14.97918415,  1.87136686])
    p_10 = np.array([15.44171906, 15.06013393,  2.38993907])
    p_11 = np.array([15.35223579, 15.09278297,  2.89758468])
    p_12 = np.array([15.26709461, 15.10097027, -3.11276412])
    p_13 = np.array([15.06439018, 15.09509087, -3.08760095])
    p_14 = np.array([14.97629261, 15.07715511,  3.0056169 ])
    p_15 = np.array([15.00144863, 15.08319283, -2.17473793])
    p_16 = np.array([14.98014141, 15.02025509, -1.54403925])
    p_17 = np.array([14.98888134, 14.98716545, -0.829862])
 

<<<<<<< HEAD
    v0 = Vertex(p_0, None)
    v1 = Vertex(p_1, None)
    v2 = Vertex(p_2, None)
    v3 = Vertex(p_3, None)
    v4 = Vertex(p_4, None)
    v5 = Vertex(p_5, None)
    v6 = Vertex(p_6, None)
    v7 = Vertex(p_7, None)
    v8 = Vertex(p_8, None)
    v9 = Vertex(p_9, None)
    v10 = Vertex(p_10, None)
    v11 = Vertex(p_11, None)
    v12 = Vertex(p_12, None)
    v13 = Vertex(p_13, None)
    v14 = Vertex(p_14, None)
    v15 = Vertex(p_15, None)
    v16 = Vertex(p_16, None)
    v17 = Vertex(p_17, None)
=======
    X = optimize_graph(graph) # 2 x 3
   


>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

    graph.add_vertex(v0)
    graph.add_vertex(v1)
    graph.add_vertex(v2)
    graph.add_vertex(v3)
    graph.add_vertex(v4)
    graph.add_vertex(v5)
    graph.add_vertex(v6)
    graph.add_vertex(v7)
    graph.add_vertex(v8)
    graph.add_vertex(v9)
    graph.add_vertex(v10)
    graph.add_vertex(v11)
    graph.add_vertex(v12)
    graph.add_vertex(v13)
    graph.add_vertex(v14)
    graph.add_vertex(v15)
    graph.add_vertex(v16)
    graph.add_vertex(v17)
   
  
    edge = Edge(v0, v1, np.subtract(v1.pose, v0.pose))
    graph.add_edges(edge)
    edge = Edge(v1, v2, np.subtract(v2.pose, v1.pose))
    graph.add_edges(edge)
    edge = Edge(v2, v3, np.subtract(v3.pose, v2.pose))
    graph.add_edges(edge)
    edge = Edge(v3, v4, np.subtract(p_4, p_3))
    graph.add_edges(edge)
    edge = Edge(v4, v5, np.subtract(p_5, p_4))
    graph.add_edges(edge)
    edge = Edge(v5, v6, np.subtract(p_6, p_5))
    graph.add_edges(edge)
    edge = Edge(v6, v7, np.subtract(p_7, p_6))
    graph.add_edges(edge)
    edge = Edge(v7, v8, np.subtract(p_8, p_7))
    graph.add_edges(edge)
    edge = Edge(v8, v9, np.subtract(p_9, p_8))
    graph.add_edges(edge)
    edge = Edge(v9, v10, np.subtract(p_10, p_9))
    graph.add_edges(edge)
    edge = Edge(v10, v11, np.subtract(p_11, p_10))
    graph.add_edges(edge)
    edge = Edge(v11, v12, np.subtract(p_12, p_11))
    graph.add_edges(edge)
    edge = Edge(v12, v13, np.subtract(p_13, p_12))
    graph.add_edges(edge)
    edge = Edge(v13, v14, np.subtract(p_14, p_13))
    graph.add_edges(edge)
    edge = Edge(v14, v15, np.subtract(p_15, p_14))
    graph.add_edges(edge)
    edge = Edge(v15, v16, np.subtract(p_16, p_15))
    graph.add_edges(edge)
    edge = Edge(v16, v17, np.subtract(p_17, p_16))
    graph.add_edges(edge)

    # loop detected
    edge = Edge(v17, v0, np.array([0,0,0]))
    graph.add_edges(edge)

<<<<<<< HEAD
    #err = calculate_err(v4.pose, v0.pose, u_40)
    #print("err", err)
=======
    plot_path(ground, noisy, X)
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a


    ground = np.array([x_0, x_1, x_2, x_3, x_4, x_5, x_6, x_7, x_8, x_9, x_10, x_11, x_12, x_13, x_14, x_15, x_16, x_17, x_0]) # 2 x 3
    noisy = np.array([p_0, p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8, p_9, p_10, p_11, p_12, p_13, p_14, p_15, p_16, p_17, p_0]) # 2 x 3

    # # err = calculate_err(vi.pose, vj.pose, uij)
    # print("err", err)
    X, mean_err = optimize_graph(graph) # 2 x 3
   


    plot_path(ground, noisy, X)
    #plot_mean_err(mean_err)


if __name__ == "__main__":
    #test_simple()
    test()
