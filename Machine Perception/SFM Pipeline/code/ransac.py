from lse import least_squares_estimation
import numpy as np
from copy import deepcopy


def ransac_estimator(X1, X2, num_iterations=60000):
    sample_size = 8

    eps = 10**-4

    best_num_inliers = -1
    best_inliers = None
    best_E = None
    # https://groups.csail.mit.edu/graphics/classes/6.837/F01/Lecture09/Slide16.html
    e3_skew = np.matrix('0 -1 0; 1 0 0; 0 0 0')
    for i in range(num_iterations):
        # permuted_indices = np.random.permutation(np.arange(X1.shape[0]))
        permuted_indices = np.random.RandomState(seed=(i*10)).permutation(np.arange(X1.shape[0]))
        sample_indices = permuted_indices[:sample_size]
        test_indices = permuted_indices[sample_size:]

        """ YOUR CODE HERE
        """
        X1_sample = deepcopy(X1[sample_indices])
        X2_sample = deepcopy(X2[sample_indices])
        E_est = least_squares_estimation(X1_sample, X2_sample)

        inliers = sample_indices
        
        for i in test_indices:
            X2_test = X2[i].reshape(-1,1)
            X1_test = X1[i].reshape(-1,1)
            
            d2 = np.square(X2_test.T@E_est@X1_test)/ np.linalg.norm(e3_skew@E_est@X1_test)**2
            d1 = np.square(X1_test.T@E_est.T@X2_test) / np.linalg.norm(e3_skew@E_est.T@X2_test)**2
            # print(d1+d2)

            if (d1+d2)[0,0]<eps:
                inliers = np.append(inliers,i)

        """ END YOUR CODE
        """
        if inliers.shape[0] > best_num_inliers:
            best_num_inliers = inliers.shape[0]
            best_E = E_est
            best_inliers = inliers


    return best_E, best_inliers