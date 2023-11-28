import numpy as np


def flow_lk_patch(Ix, Iy, It, x, y, size=5):
    """
    Find the Lucas-Kanade optical flow on a single square patch.
    The patch is centered at (y, x), therefore it generally extends
    from x-size//2 to x+size//2 (inclusive), same for y, EXCEPT when
    exceeding image boundaries!
    
    WARNING: Pay attention to how you index the images! The first coordinate
    is actually the y-coordinate, and the second coordinate is the x-coordinate.
    
    Inputs:
        - Ix: image gradient along the x-dimension - shape: (H, W)
        - Iy: image gradient along the y-dimension - shape: (H, W)
        - It: image time-derivative - shape: (H, W)
        - x: SECOND coordinate of patch center - integer in range [0, W-1]
        - y: FIRST coordinate of patch center - integer in range [0, H-1]
        - size: optional parameter to change the side length of the patch in pixels
    
    Outputs:
        - flow: flow estimate for this patch - shape: (2,)
        - conf: confidence of the flow estimates - scalar
    """

    ### STUDENT CODE START ###
    H, W = Ix.shape
    half_size = size // 2
    y_id = np.array([y-half_size , y + half_size + 1])
    np.clip(y_id, 0, H, y_id)

    x_id = np.array([x-half_size , x + half_size + 1])
    np.clip(x_id, 0, W, x_id)

   

    # Extract the patch and gradients
    patch_Ix = Ix[y_id[0] : y_id[1], x_id[0] : x_id[1]]
    patch_Iy = Iy[y_id[0] : y_id[1], x_id[0] : x_id[1]]
    patch_It = It[y_id[0] : y_id[1], x_id[0] : x_id[1]]

    # Flatten the matrices
   
    patch_Ix = patch_Ix.flatten()
    patch_Iy = patch_Iy.flatten()
    # Construct the A matrix
    A = np.column_stack((patch_Ix, patch_Iy))
    B = -np.array(patch_It).reshape(-1,1)
    #Solve for flow
    flow,_,_,eval =np.linalg.lstsq(A,B, rcond=-1)
    
    flow = [float(i) for i in flow]
    
    conf = min(eval)
    
    ### STUDENT CODE END ###

    return flow, conf


def flow_lk(Ix, Iy, It, size=5):
    """
    Compute the Lucas-Kanade flow for all patches of an image.
    To do this, iteratively call flow_lk_patch for all possible patches.
    
    WARNING: Pay attention to how you index the images! The first coordinate
    is actually the y-coordinate, and the second coordinate is the x-coordinate.
    
    Inputs:
        - Ix: image gradient along the x-dimension - shape: (H, W)
        - Iy: image gradient along the y-dimension - shape: (H, W)
        - It: image time-derivative
    Outputs:
        - image_flow: flow estimate for each patch - shape: (H, W, 2)
        - confidence: confidence of the flow estimates - shape: (H, W)
    """

    ### STUDENT CODE START ###
    # double for-loop to iterate over all patches

    H, W = Ix.shape
    image_flow = np.zeros((H, W, 2))
    confidence = np.zeros((H, W))

    # Iterate over all possible patches
    for y in range(H):
        for x in range(W):
            # Call flow_lk_patch for each patch
            flow, conf = flow_lk_patch(Ix, Iy, It, x, y, size)
            
            # Update image_flow and confidence arrays
            image_flow[y, x, :] = flow
            confidence[y, x] = conf
    
    ### STUDENT CODE END ###
    
    
    return image_flow, confidence

    

