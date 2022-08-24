from copy import deepcopy
import numpy as np
from sklearn.neighbors import NearestNeighbors
from graph import Vertex
import statistics

outlier_rejection = True


def uniform_sampling(A, B):
    # raw data has 60 points (x, y) from -30 degrees to +30 degrees
    # error of a range finder is uniform over the distance
    # It should be noted that the accuracy error of the laser rangefinder is not proportional to the measurement distance, 
    # the entire distance is the same, but if the distance is too long, the error will increase by ±0.5mm/100m.
    assert A.shape == B.shape
    A_sample = []
    B_sample = []

    for i in range(0, len(A), 2):
        A_sample.append(A[i])
        B_sample.append(B[i])
    
    A_sample = np.array(A_sample)
    B_sample = np.array(B_sample)

    return A_sample, B_sample

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''
    # print(A.shape)
    # print(B.shape)
    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, D, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor [src, dst]
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    # indices = [src, dst]

    return distances.ravel(), indices.ravel()


def check_outlier(distances, mean_error, indices):
    diff = np.copy(distances)
    #diff = [distance - mean_error for distance in d]
    diff.sort()
    q1 = []
    q3 = []
    median = int(len(diff) / 2)
    if len(diff) % 2 == 0:
        q1 = diff[:median-1]
    else:
        q1 = diff[:median]

    q3 = diff[median+1:]

    q1_median = statistics.median(q1)
    q3_median = statistics.median(q3)

    iqr = q3_median - q1_median
    upper_fence = q3_median + (1.5 * iqr)
    lower_fence = q1_median - (1.5 * iqr)

    src_indicies = []
    dst_indicies = []

    for i in range(len(distances)):
        if distances[i] < lower_fence or distances[i] > upper_fence:
            continue

        src_indicies.append(i)
        dst_indicies.append(indices[i])
    passed = [src_indicies, dst_indicies]
    
    return passed



def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''
    A = np.array(A)
    B = np.array(B)

    assert A.shape == B.shape
    
    A, B = uniform_sampling(A, B)
    assert A.shape == B.shape

    is_converged = False

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = float('inf')

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points (m = num of dimensions)
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)
        mean_error = np.mean(distances)

        if outlier_rejection:
            passed_pairs = check_outlier(np.copy(distances), mean_error, indices) # [src, dst]
            assert len(passed_pairs[0]) == len(passed_pairs[1])
            T,_,_ = best_fit_transform(src[:m,passed_pairs[0]].T, dst[:m,passed_pairs[1]].T)


        else:
            # compute the transformation between the current source and nearest destination points
            T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        
        # update the current source
        src = np.dot(T, src)

        # if the error does not decrease, break
        if prev_error < mean_error:
            break

        # check error // converges
        if np.abs(prev_error - mean_error) < tolerance:
            is_converged = True
            break

            
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i, np.abs(prev_error - mean_error), is_converged
