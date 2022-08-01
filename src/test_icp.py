import numpy as np
import time
from graph_optimization import is_converged
import icp
import matplotlib.pyplot as plt

# Constants
N = 240                                    # number of random points in the dataset
num_tests = 100                             # number of test iterations
dim = 2                                     # number of dimensions of the points
noise_sigma = .01                           # standard deviation error to be added
translation = .1                            # max translation of the test set
rotation = .1                               # max rotation (radians) of the test set


def rotation_matrix(axis, theta):
    axis = axis/np.sqrt(np.dot(axis, axis))
    a = np.cos(theta/2.)
    b, c, d = -axis*np.sin(theta/2.)

    return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                  [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                  [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])


def test_best_fit():

    # Generate a random dataset
    A = np.random.rand(N, dim)

    total_time = 0

    for i in range(num_tests):

        B = np.copy(A)

        # Translate
        t = np.random.rand(dim)*translation
        B += t

        # Rotate
        R = rotation_matrix(np.random.rand(dim), np.random.rand()*rotation)
        B = np.dot(R, B.T).T

        # Add noise
        B += np.random.randn(N, dim) * noise_sigma

        # Find best fit transform
        start = time.time()
        T, R1, t1 = icp.best_fit_transform(B, A)
        total_time += time.time() - start

        # Make C a homogeneous representation of B
        C = np.ones((N, 4))
        C[:,0:3] = B

        # Transform C
        C = np.dot(T, C.T).T

        # assert np.allclose(C[:,0:3], A, atol=6*noise_sigma) # T should transform B (or C) to A
        # assert np.allclose(-t1, t, atol=6*noise_sigma)      # t and t1 should be inverses
        # assert np.allclose(R1.T, R, atol=6*noise_sigma)     # R and R1 should be inverses

    print('best fit time: {:.3}'.format(total_time/num_tests))

    return


def test_icp(A, B):

    # Generate a random dataset
    #A = np.random.rand(N, dim)

    total_time = 0
    #plot(A, B)


    for i in range(num_tests):

        #B = np.copy(A)

        # Translate
        # t = np.random.rand(dim)*translation
        # B += t

        # # Rotate
        # R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
        # B = np.dot(R, B.T).T

        # # Add noise
        # B += np.random.randn(N, dim) * noise_sigma

        # # Shuffle to disrupt correspondence
        # np.random.shuffle(B)

        # Run ICP
        start = time.time()
        T, distances, iterations, tolerance, is_converged  = icp.icp(B, A, tolerance=0.000000001)
        total_time += time.time() - start

        # Make C a homogeneous representation of B
        C = np.ones((N, dim+1))
        C[:,0:dim] = np.copy(B)

        # Transform C
        C = np.dot(T, C.T).T

        #assert np.mean(distances) < 6*noise_sigma                   # mean error should be small
        # assert np.allclose(T[0:3,0:3].T, R, atol=6*noise_sigma)     # T and R should be inverses
        # assert np.allclose(-T[0:3,3], t, atol=6*noise_sigma)        # T and t should be inverses

    #print('icp time: {:.3}'.format(total_time/num_tests))
    print("tolerance", tolerance)
    print("iteration", iterations)
    print("mean distance", np.mean(distances))
    plot(A, B, C)

    return

def plot(A, B, C):
    A_x = []
    A_y = []
    for i in range(len(A)):
        A_x.append(A[i][0])
        A_y.append(A[i][1])
    
    B_x = []
    B_y = []
    for i in range(len(B)):
        B_x.append(B[i][0])
        B_y.append(B[i][1])

    C_x = []
    C_y = []
    for i in range(len(C)):
        C_x.append(C[i][0])
        C_y.append(C[i][1])
    
    plt.plot(A_x, A_y,  mec = 'hotpink', mfc = 'hotpink', label='A')
    plt.plot(B_x, B_y,  mec = '#4CAF50', mfc = '#4CAF50', label='B')
    plt.plot(C_x, C_y,  mec = 'r', mfc = 'r', label="transformed")
    plt.legend()
    plt.show()

def plot_poses(ground_pose, prev_pose, trans_pose=None):
    plt.plot(ground_pose[0,0], ground_pose[1,0], 'o', mec = 'hotpink', mfc = 'hotpink', label='ground truth')
    plt.plot(prev_pose[0,0], prev_pose[1,0], 'o', mec = '#4CAF50', mfc = '#4CAF50', label='uniform noise position')
    #plt.plot(trans_pose[0], trans_pose[1], 'o', mec = 'r', mfc = 'r', label='gaussian noise position')
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.legend()
    plt.show()



def create_data():
   
    th = np.pi / 8
    move = np.array([[0.30], [0.5]])
    rnd_scale = 0.03
    x1 = np.linspace(0, 1.1, N)
    y1 = np.sin(x1 * np.pi)
    data1 = np.array([x1, y1])
    d1 = []

    for i in range(N):
        d1.append([data1[0][i], data1[1][i]])
    
    d1 = np.array(d1).astype(np.float32)


    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    rand = np.random.rand(2, N)*rnd_scale
    data2 = np.dot(rot, data1) + move
    #data2 = np.add(data2, rand)


    #data2[0:,0:6] = 1.0
    # data2[0,0] = 1.0
    # data2[0,1] = 1.0
    # data2[0,2] = 1.0

    # data2[0, 13] = 0.5
    # data2[0, 14] = 0.6
   
    d2 = []

    for i in range(N):
        d2.append([data2[0][i], data2[1][i]])
    d2 = np.array(d2).astype(np.float32)

    #print(np.shape(d1), np.shape(d2))

    # A_x = []
    # A_y = []
    # for i in range(len(d1)):
    #     A_x.append(d1[i][0])
    #     A_y.append(d1[i][1])

    return d1, d2

    # plt.plot(data1[0], data1[1], label="original A")
    # plt.plot(data2[0], data2[1], label="original B")
    # plt.plot(A_x, A_y, label="modified A")
    # plt.legend()
    # plt.show()
    




if __name__ == "__main__":
    d1, d2 = create_data()
    # A_x = []
    # A_y = []
    # for i in range(len(d1)):
    #     A_x.append(d1[i][0])
    #     A_y.append(d1[i][1])
    # plt.plot(A_x, A_y, label="modified A")
    # plt.legend()
    # plt.show()
    #test_best_fit()
    test_icp(d1, d2) # d1 = dst, d2 = src