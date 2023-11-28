import numpy as np

def est_pixel_world(pixels, R_wc, t_wc, K):
    """
    Estimate the world coordinates of a point given a set of pixel coordinates.
    The points are assumed to lie on the x-y plane in the world.
    Input:
        pixels: N x 2 coordiantes of pixels
        R_wc: (3, 3) Rotation of camera in world
        t_wc: (3, ) translation from world to camera
        K: 3 x 3 camara intrinsics
    Returns:
        Pw: N x 3 points, the world coordinates of pixels
    """

    ##### STUDENT CODE START #####
    
    #Compute the world coordinates of the pixels
    
    t = -R_wc.T @ t_wc
    Pw = np.zeros((pixels.shape[0], 3))
    
    TransformMat = np.hstack((R_wc.T, t.reshape(3,1)))[:,[0,1,3]] 
    H = np.linalg.inv(K @ TransformMat) #Compute H matrix from world to camera
    pixel_vec = np.hstack((pixels, np.ones((pixels.shape[0],1)))) 

    #Compute the world coordinates of the pixels
    Pw = (H @ pixel_vec.T)
    Pw = Pw/Pw[2,:]
    Pw[2] = 0
    
    ##### STUDENT CODE END #####
    return Pw.T
