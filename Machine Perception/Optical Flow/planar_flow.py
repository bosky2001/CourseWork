import numpy as np

def compute_planar_params(flow, K, up=[256, 0], down=[512, 256]):
    """
    Use the flow field to compute the 8 parameters of the planar motion.
    
    Inputs:
        - flow: optical flow - shape: (H, W, 2)
        - K: intrinsic calibration matrix - shape: (3, 3)
        - up: upper left corner of the patch - list of 2 integers
        - down: lower right corner of the patch - list of 2 integers
    Outputs:
        - sol: solution to the linear equation - shape: (8,)
    """
    
    ### STUDENT CODE START ###
    
    # 1. Extract the flow in the patch
    # 2. Normalize the flow by the intrinsic matrix
    # 3. Convert the pixel coordinates to normalized projective coordinates
    # 4. Solve the linear equation of 8 parameters
    # Useful functions: np.meshgrid(), np.linalg.lstsq()
    
    K_inverse = np.linalg.inv(K)
    # print(K_inverse)
    x = np.arange(up[1],down[1])
    y = np.arange(up[0], down[0])

    xx, yy = np.meshgrid(x,y)
    calib_x = np.array([])
    calib_y = np.array([])
    p = 0

    for i in range(up[0], down[0]):
        for j in range(up[1], down[1]):
            calibrated_points = K_inverse @ np.array([j,i,1])
            # print(calibrated_points)
            calibrated_points = calibrated_points/calibrated_points[2]
            calib_x = np.append(calib_x, calibrated_points[0])
            calib_y = np.append(calib_y, calibrated_points[1])

            p = p+1

    # print(np.shape(calib_x))
    # print(calib_x)
    x_2 = calib_x ** 2
    y_2 = calib_y**2
    # print(x_2)
    xy = calib_x * calib_y

    A_up = np.hstack((x_2.reshape(-1,1), xy.reshape(-1,1), calib_x.reshape(-1,1),
                       calib_y.reshape(-1,1),np.ones((p,1)), np.zeros((p,3))))
    A_down = np.hstack((xy.reshape(-1,1),y_2.reshape(-1,1), np.zeros((p,3)),
                          calib_y.reshape(-1,1), calib_x.reshape(-1,1), np.ones((p,1))))

    A = np.vstack((A_up, A_down))

    flow_x_calibrated = flow[yy,xx, 0]
    flow_y_calibrated = flow[yy,xx, 1]

    flow_x_flatten = flow_x_calibrated.flatten()
    flow_y_flatten = flow_y_calibrated.flatten()

    b = np.vstack((flow_x_flatten, flow_y_flatten, np.zeros(flow_x_calibrated.size)))
    b_calib = K_inverse@b
    b_calib = b_calib[0:2, :]
    b = np.vstack((b_calib[0].reshape(-1,1), b_calib[1].reshape(-1,1)))
    A_values = np.linalg.lstsq(A,b, rcond=None)[0]
    sol = A_values.flatten()
    ### STUDENT CODE END ###
    
    return sol
    
