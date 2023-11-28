from est_homography import est_homography
import numpy as np

def PnP(Pc, Pw, K=np.eye(3)):
    """
    Solve Perspective-N-Point problem with collineation assumption, given correspondence and intrinsic

    Input:
        Pc: 4x2 numpy array of pixel coordinate of the April tag corners in (x,y) format
        Pw: 4x3 numpy array of world coordinate of the April tag corners in (x,y,z) format
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3, ) numpy array describing camera translation in the world (t_wc)

    """

    ##### STUDENT CODE START #####

    # Homography Approach
    # Following slides: Pose from Projective Transformation
    Pw = Pw[:,:2]
    H = est_homography(Pw, Pc)
    H = H / H[2][2]

    H_prime = np.linalg.inv(K) @ H
    a = H_prime[:,0]
    b = H_prime[:,1]
    c = H_prime[:,2]
    svd_mat = np.zeros((3,2))
    svd_mat[:,0] = a
    svd_mat[:,1] = b
    # svd_mat[:,2] = np.cross(a,b)
    # Up, S, Vh = np.linalg.svd(svd_mat, full_matrices=False)
    Up, S, Vh = np.linalg.svd(svd_mat, full_matrices=False)
    
    
    r1 = Up@Vh[:,0]
    r2 = Up@Vh[:, 1]
    r3 = np.cross(r1,r2)
    R = np.zeros((3,3))
    R[:,0] = r1
    R[:,1] = r2
    R[:,2] = r3
    
    # print(R)
    t = c/np.linalg.norm(a)
    t = -R.T @ t
    R = np.linalg.inv(R)
    # print(R,t)
    # # print(c/l)
    ##### STUDENT CODE END #####

    return R, t
