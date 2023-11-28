import numpy as np

def epipole(flow_x, flow_y, smin, thresh, num_iterations=None):
    """
    Compute the epipole from the flows,
    
    Inputs:
        - flow_x: optical flow on the x-direction - shape: (H, W)
        - flow_y: optical flow on the y-direction - shape: (H, W)
        - smin: confidence of the flow estimates - shape: (H, W)
        - thresh: threshold for confidence - scalar
    	- Ignore num_iterations
    Outputs:
        - ep: epipole - shape: (3,)
    """
    # Logic to compute the points you should use for your estimation
    # We only look at image points above the threshold in our image
    # Due to memory constraints, we cannot use all points on the autograder
    # Hence, we give you valid_idx which are the flattened indices of points
    # to use in the estimation estimation problem 
    good_idx = np.flatnonzero(smin>thresh)
    permuted_indices = np.random.RandomState(seed=10).permutation(
        good_idx
    )
    valid_idx=permuted_indices[:3000]

    ### STUDENT CODE START - PART 1 ###
    
    # 1. For every pair of valid points, compute the epipolar line (use np.cross)
    # Hint: for faster computation and more readable code, avoid for loops! Use vectorized code instead.
    
    H, W = flow_x.shape

    x_valid = valid_idx % W - W/2
    y_valid = valid_idx // W - H/2

    # 3000x3 shape
    x_p = np.hstack((x_valid.reshape(-1,1), y_valid.reshape(-1,1), np.ones_like(y_valid.reshape(-1,1))))

    # 3000x3 shape
    u = np.hstack((flow_x.flat[valid_idx].reshape(-1,1), flow_y.flat[valid_idx].reshape(-1,1), np.zeros_like(flow_y.flat[valid_idx].reshape(-1,1))))

    # 3000x3
    A = np.cross(x_p, u)
    _, _, V = np.linalg.svd(A)

    # Extract epipole from the null space
    ep = V.T[:, -1]
    # print(ep)
    return ep