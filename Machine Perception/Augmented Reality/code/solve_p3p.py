import numpy as np

def P3P(Pc, Pw, K=np.eye(3)):
    """
    Solve Perspective-3-Point problem, given correspondence and intrinsic

    Input:
        Pc: 4x2 numpy array of pixel coordinate of the April tag corners in (x,y) format
        Pw: 4x3 numpy array of world coordinate of the April tag corners in (x,y,z) format
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """

     ##### STUDENT CODE START #####
    
    
    
    #Distance between the points
    a = np.linalg.norm(Pw[1] - Pw[2])
    b = np.linalg.norm(Pw[0] - Pw[2])
    c = np.linalg.norm(Pw[0] - Pw[1])

    #Calculating square of the distance between the points
    a2 = a**2
    b2 = b**2
    c2 = c**2

    #Converting the pixels into world cordinates upto a scale
    d1, d2, d3, _ = (np.linalg.inv(K) @ np.hstack((Pc, np.ones((Pc.shape[0], 1)))).T).T

    #Getting unit vectors for j1, j2, and j3
    d1 /= np.linalg.norm(d1)
    d2 /= np.linalg.norm(d2)
    d3 /= np.linalg.norm(d3)

    #Getting the cosine of angles between the vectors
    gamma = np.dot(d1, d2)
    beta = np.dot(d1, d3)
    alpha = np.dot(d2, d3)
    
    A4 = (((a2 - c2)/b2)-1)**2 - 4*c2*(alpha**2)/b2
    A3 = 4*( (((a2 - c2)/b2))*(1 - ((a2 - c2)/b2))*beta - (1 - (a2+c2)/b2)*alpha*gamma + 2*c2*alpha*alpha*beta/b2 )
    A2 = 2*( (((a2 - c2)/b2))**2 - 1 + 2*(((a2 - c2)/b2)*((a2 - c2)/b2))*beta*beta + 2*(b2 - c2)*alpha*alpha/b2 - 4*((a2 + c2)/b2)*alpha*beta*gamma + 2*gamma*gamma*(b2 - a2)/b2 )
    A1 = 4*(   -(((a2 - c2)/b2))*(1+((a2 - c2)/b2))*beta + 2*a2*gamma*gamma*beta/b2 - (1 - (a2 + c2)/b2)*alpha*gamma)
    A0 =  (1+((a2 - c2)/b2))**2 - 4*a2*gamma**2/b2

    #Calculating the roots of the 4rth degree polynomial
    roots = np.roots([A4, A3, A2, A1, A0])
    v = roots[~np.iscomplex(roots)]
    # print(v)

    u = ((-1 + (a2-c2)/b2)*v**2 -2*(a2-c2)*beta*v/b2 + 1 + (a2-c2)/b2) / (2*(gamma - v*alpha))
    # print(u)
    
    s1 = np.sqrt(c2/(1 + u**2 -2*u*gamma))
    s2 = u*s1
    s3 = v*s1

    idx = 0
    p1 = s1[idx]*d1
    p2 = s2[idx]*d2
    p3 = s3[idx]*d3
    # print(p1)
    Pc_3d = np.vstack((p1,p2,p3))

    # Invoke Procrustes function to find R, t
    # You may need to select the R and t that could transoform all 4 points correctly. 
    R,t = Procrustes(Pc_3d, Pw[0:3])
    ##### STUDENT CODE END #####

    return R, t

def Procrustes(X, Y):
    """
    Solve Procrustes: Y = RX + t

    Input:
        X: Nx3 numpy array of N points in camera coordinate (returned by your P3P)
        Y: Nx3 numpy array of N points in world coordinate
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """
    ##### STUDENT CODE START #####
    #Calculating the mean of the points
    X_mean = np.mean(X, axis=0)
    Y_mean = np.mean(Y, axis=0)

    #Calculating the centered points
    X_points = np.transpose(X - X_mean)
    Y_points = np.transpose(Y - Y_mean)

    A = Y_points @ X_points.T
    U, _, Vt = np.linalg.svd(A, full_matrices=0)
    sigma = np.array([[1,0,0],[0,1,0],[0,0,np.linalg.det(U @ Vt)]])
  
    R = U @ sigma @ Vt
    t = Y_mean - (R@X_mean)

    ##### STUDENT CODE END #####

    return R, t