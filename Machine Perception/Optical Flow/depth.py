import numpy as np

def depth(flow, confidence, ep, K, thres=10):
    """
    Compute the depth map from the flow and confidence map.
    
    Inputs:
        - flow: optical flow - shape: (H, W, 2)
        - confidence: confidence of the flow estimates - shape: (H, W)
        - ep: epipole - shape: (3,)
        - K: intrinsic calibration matrix - shape: (3, 3)
        - thres: threshold for confidence (optional) - scalar
    
    Output:
        - depth_map: depth at every pixel - shape: (H, W)
    """
    depth_map = np.zeros_like(confidence)

    ### STUDENT CODE START ###
    
    # 1. Find where flow is valid (confidence > threshold)
    # 2. Convert these pixel locations to normalized projective coordinates
    # 3. Same for epipole and flow vectors
    # 4. Now find the depths using the formula from the lecture slides
    confidence_threshold = np.where(confidence>thres)
    u_flattened = (flow[:,:,0][confidence_threshold]).reshape(-1,1)
    v_flattened = (flow[:,:,1][confidence_threshold]).reshape(-1,1)

    zero_arrays = np.zeros(u_flattened.shape).reshape(-1,1)
    u_v = np.hstack((u_flattened, v_flattened, zero_arrays))
    K_inverse = np.linalg.inv(K)
    p_translation = K_inverse@u_v.T
    X_p = np.hstack((confidence_threshold[1].reshape(-1,1), confidence_threshold[0].reshape(-1,1),
                     np.ones(confidence_threshold[0].shape[0]).reshape(-1,1))).T
    X_calibrated = K_inverse@X_p

    calibrated_ep = K_inverse@ep
    # print(calibrated_ep.shape)
    calibrated_ep = calibrated_ep.reshape(1,3)

    p_trans_difference = np.linalg.norm((X_calibrated.T - calibrated_ep),axis = 1)

    p_den = np.linalg.norm(p_translation.T, axis=1)

    depth = p_trans_difference/p_den

    depth_map[confidence_threshold] = depth
    
    ### STUDENT CODE END ###
    
    
    ## Truncate the depth map to remove outliers
    
    # require depths to be positive
    truncated_depth_map = np.maximum(depth_map, 0) 
    valid_depths = truncated_depth_map[truncated_depth_map > 0]
    
    # You can change the depth bound for better visualization if you depth is in different scale
    depth_bound = valid_depths.mean() + 10 * np.std(valid_depths)
    print(f'depth bound: {depth_bound}')

    # set depths above the bound to 0 and normalize to [0, 1]
    truncated_depth_map[truncated_depth_map > depth_bound] = 0
    truncated_depth_map = truncated_depth_map / truncated_depth_map.max()

    return truncated_depth_map
