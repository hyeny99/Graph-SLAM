#! /usr/bin/env python3
import enum
from turtle import color
import numpy as np
from numpy.linalg import inv
import math
from graph import Graph, Vertex
import matplotlib
import matplotlib.pyplot as plt
import scipy.sparse.linalg
import scipy.sparse
from scipy.linalg import cholesky
#from scipy.sparse.cholmod import cholesky

'''
reference:
Kolkir, Oct 14, 2021
slam-playground
github
visited: July 31, 2022
https://github.com/Kolkir/slam-playground/blob/main/doc/slam_se2_derivation.ipynb

The implementation structure is from Kolkir's code.
The code is studied and modified to suit the project.

'''


'''
A pose can be represented as a homogeneous transformation matrix
X = |R t| = |cos -sin   x|            
    |0 1|   |sin  cos   y|
            |0     0    1|
or as a 3 component vector
x = [x, y, theta]
For the graph:
X = [(x1, y1, theta1), (x2, y2, theta2), ... ,(xn, yn, thetan)] in vector.
'''


'''
        odom_data and trans_data are x_y_data.
        e(xi, xj, zij) is a function that computes a difference between
        the exepected observation z'ij (transformed observation) and the real observation zij.
        The edge represents the odom informataion between two nodes.
        eij(xi, xj) = zij - z'ij(xi, xj)
'''

# --util----------------------------------------------------------------------------------------------------------------------------
def t2v(trans):
    # transformation matrix to vector
    v = np.zeros((3,1))
    v[:2,0] = trans[:2,2]
    v[2] = np.arctan2(trans[1,0], trans[0,0])
    return v

def v2t(v):
    # vector to transformation matrix
    cos = math.cos(v[2])  # v[2] = theta
    sin = math.sin(v[2])
    trans = np.array([[cos, -sin, v[0]],
                      [sin,  cos, v[1]],
                      [0,    0,   1]])    
    return trans
# -----------------------------------------------------------------------------------------------------------------------------------


def calculate_err(xi, xj, uij):
    '''
    for 2D SLAM, the error function is
        eij(xi, xj) = t2v(inv_Uij(inv_Xi * Xj))
    where Uij is the odometry measurement and
    Uij, Xi and Xj are represented as homogeneous transformation matrices
    
    The position and orientation difference between the pose xi and the pose xj
    is written in a form of the transformation matrix.
    '''

    # convert a vector form (xi, xj and uji) into 
    # a homogeneous transformation matrix form
    trans_i = v2t(xi)
    trans_j = v2t(xj)
    trans_uij = v2t(uij)

    # calculate error matrix
    #err_trans = np.dot(inv(trans_uij), np.dot(inv(trans_i), trans_j))
    err_trans = inv(trans_uij) @ (inv(trans_i) @ trans_j)

    # convert error matrix to a vector
    err = t2v(err_trans) # 3 * 1
    return err


def calculate_jacobian(vi, vj, uij):
    '''
        inv(Xi)Xj = |R t|
                    |0 1|
        represents the conversion matrix from the coordinate system j to
        the coordinate system i.
        => the pose of j from i
        Zij is the measurement of the pose of j from i.
    params:
        vi = xi (pose at i in a vector form (xi, yi, thetai))
        vj = xj (pose at j in a vector form (xj, yj, thetaj))
        uij = odom info
    '''

   

    si = np.sin(vi[2])
    ci = np.cos(vi[2])
    dr_i = np.array([[-si, ci], [-ci, -si]]).T
    dt_ij = np.array([vj[:2] - vi[:2]]).T

    t_i = v2t(vi)
    t_j = v2t(vj)
    t_u = v2t(uij)
    R_i = t_i[:2,:2]
    R_z = t_u[:2,:2]

    A_i = np.vstack((np.hstack((-R_z.T @ R_i.T, R_z.T @ (dr_i.T @ dt_ij))), 
                         [0, 0, -1]))
    B_j = np.vstack((np.hstack((R_z.T @ R_i.T, np.zeros((2,1)))),
                         [0, 0, 1]))


    assert A_i.shape == B_j.shape
    
    print("A_i", A_i)

    return A_i, B_j



<<<<<<< HEAD
def optimize_graph(graph:Graph, tolerance=1e-5, iterations=2):
=======
def optimize_graph(graph:Graph, tolerance=1e-5, iterations=10):
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a
    #cov = 0.01

    sigma_x = 0.01
    sigma_y = 0.01
    sigma_theta = 0.01

    omega = np.zeros((3, 3))
    omega[0,0] = sigma_x
    omega[1,1] = sigma_y
    omega[2,2] = sigma_theta

    mean_errors = []

    # degree of freedom for 2D (x, y, theta)
    n = 3
    
    for _ in range(iterations):

        edges = graph.edges
        vertices = graph.verticies
        m = len(vertices)
        poses = []
        for vertex in vertices:
            poses.append(vertex.pose)
    
        poses = np.array(poses)
    
        X = np.array(poses).T
        print("X shape", X.shape)

        # define 
<<<<<<< HEAD
        H = np.zeros((m * n, n * m)).astype(np.float)

        # define a coefficient vector
        b = np.zeros((m * n, 1)).astype(np.float)
=======
        H = scipy.sparse.csc_matrix((m * n, n * m))
        #H = np.zeros((m * n, n * m)).astype(np.float)

        # define a coefficient vector
        b = scipy.sparse.csc_matrix((m * n, 1))
        #b = np.zeros((m * n, 1)).astype(np.float)
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

        for count, edge in enumerate(edges):
            vi = edge.vi
            vj = edge.vj
            uij = edge.uij

            xi = vi.pose
            xj = vj.pose

            if count == len(edges) -1:
                e_ij = calculate_err(xi, xj, uij) 
            # e_ij = calculate_err(xi, xj, uij)  
            else:
                e_ij = np.zeros((3, 1))
            #e_ij = calculate_err(xi, xj, uij) 
            print("err", e_ij)

            A_ij, B_ij = calculate_jacobian(xi, xj, uij)

            # compute the contribution of this constraint to the linear system
            H_ii = A_ij.T @ omega @ A_ij
            H_ij = A_ij.T @ omega @ B_ij
            H_ji = B_ij.T @ omega @ A_ij
            H_jj = B_ij.T @ omega @ B_ij

            # compute the coefficient vector
            b_i = A_ij.T @ omega @ e_ij
            b_j = B_ij.T @ omega @ e_ij

            # get the index of the vertex
            i_id = graph.get_index_vertex(vi)
            j_id = graph.get_index_vertex(vj)

            # print("index i", i)
            # print("index j", j)

<<<<<<< HEAD
=======
            def id2index(id):
                return slice((n*id), (n*(id+1)))
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

            # index_i = i * n
            # index_j = j * n

<<<<<<< HEAD
            # update the linear system
            H[index_i:index_i+n, index_i:index_i+n] += H_ii
            H[index_i:index_i+n, index_j:index_j+n] += H_ij
            H[index_j:index_j+n, index_i:index_i+n] += H_ji
            H[index_j:index_j+n, index_j:index_j+n] += H_jj
=======

            # # update the linear system
            # H[index_i:index_i+n, index_i:index_i+n] += H_ii
            # H[index_i:index_i+n, index_j:index_j+n] += H_ij
            # H[index_j:index_j+n, index_i:index_i+n] += H_ji
            # H[index_j:index_j+n, index_j:index_j+n] += H_jj
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

            # # update the coefficient vector
            # b[index_i:index_i+n] += b_i
            # b[index_j:index_j+n] += b_j

<<<<<<< HEAD
=======
            H[id2index(i_id), id2index(i_id)] += H_ii
            H[id2index(i_id), id2index(j_id)] += H_ij
            H[id2index(j_id), id2index(i_id)] += H_ij.T
            H[id2index(j_id), id2index(j_id)] += H_jj
            b[id2index(i_id)] += b_i
            b[id2index(j_id)] += b_j

>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

        
        # fix the position of the first vertex (init pose)
        H[:n,:n] += np.eye(n)
        
<<<<<<< HEAD
        L = np.linalg.cholesky(H)

        X_update = -(inv(L.T) @ inv(L)) @ b
        X_update = np.reshape(X_update, (m, n)).astype(np.float)
=======
        X_update = -scipy.sparse.linalg.spsolve(H, b)
        X_update[np.isnan(X_update)] = 0
        X_update = np.reshape(X_update, (m, n)).astype(np.float)
        # print("X_update", X_update)

        #assert (H == H.T).all()  # H is a symmetric matrix

        #L = np.tril(H)
        #U = np.triu(H)
        #assert (L == U.T).all()

        #L = np.linalg.cholesky(H)
        # H = L @ L.T.conj()

        # X_update = -inv(H) @ b
        # #X_update = -(inv(L.T) @ inv(L)) @ b
        # X_update = np.reshape(X_update, (m, n)).astype(np.float)
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

        print("dx shape", np.shape(X_update))
        # print("dx", dx)
    

        for count, value in enumerate(X_update):
<<<<<<< HEAD
            #print("update value", value)
            print("value shape", value.shape)
            poses[count] += value

            graph.update_vertex_pose(vertices[count], poses[count])

        
=======
            print("update value", value)
            poses[count] += (value * 0.01) 
            graph.update_vertex_pose(vertices[count], poses[count])

        

        
        # for i in range(0, len(edges)-1):
        #     edges[i].uij = poses[i+1] - poses[i]
        
        # print("edgse", edges[0].uij)
        # print("last", edges[m-1].uij)

                
    
        #print("updated pose vi", edges[0].vi.pose)



        # j = 0
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

        converged, mean_err = is_converged(edges, tolerance)
        mean_errors.append(mean_err)

        if converged:
            break
<<<<<<< HEAD
    
    return np.array(poses).astype(np.float), np.array(mean_errors).astype(np.float)
=======

        #print("poses", poses)

    # print("poses shape", np.shape(poses))

    # poses = []
    # for vertex in graph.verticies:
    #     poses.append(vertex.pose)

    
    return np.array(poses).astype(np.float)
>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a



def is_converged(edges, tolerance=1e-5):
    mean_err = 0
    for edge in edges:
        xi = edge.vi.pose
        xj = edge.vj.pose
        uij = edge.uij
        err = calculate_err(xi, xj, uij)
        mean_err += err
    
    mean_err /= len(edges)

    if np.all(mean_err <= tolerance):
        return True, mean_err
    
    return False, mean_err





def plot_path(ground_pose, raw_pose, X):
    print("ground pose", ground_pose)
    print("transformed", X)
    print("raw pose", raw_pose)
    
    # assert np.shape(ground_pose) == np.shape(raw_pose) == np.shape(X)
<<<<<<< HEAD

    ground_pose_T = ground_pose.T
    raw_pose_T = raw_pose.T
    X_T = X.T
=======

    ground_pose_T = ground_pose.T
    raw_pose_T = raw_pose.T
    X_T = X.T

    # ground_x = []
    # ground_y = []

    # raw_x = []
    # raw_y = []


    # for i in range(len(ground_pose)):
    #     ground_x.append(ground_pose[i][0])
    #     ground_y.append(ground_pose[i][1])

    #     raw_x.append(raw_pose[i][0])
    #     raw_y.append(raw_pose[i][1])

>>>>>>> 4ceedfc58d2dbd0451104456167d35ddd8dc3f4a

    plt.scatter(ground_pose_T[0], ground_pose_T[1], color='g', alpha=0.3, label="ground truth path")
    plt.scatter(raw_pose_T[0], raw_pose_T[1], color='r', alpha=0.3, label="path with odometry error")
    plt.scatter(X_T[0], X_T[1], color='k', alpha=0.3, label="optimized path")

    plt.plot(ground_pose_T[0], ground_pose_T[1], color='g')
    plt.plot(raw_pose_T[0], raw_pose_T[1], color='r')
    plt.plot(X_T[0], X_T[1], color='k')


    plt.legend()
    plt.show()



def plot_mean_err(mean_err):
    print("lenth", len(mean_err))
    for i in range(len(mean_err)):
        plt.scatter(i, mean_err[i][0], color='pink')
        #plt.scatter(i, mean_err[i][1], color='blue')
        #plt.scatter(i, mean_err[i][2], color='g')
    plt.show()