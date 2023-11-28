import numpy as np

def least_squares_estimation(X1, X2):
  """ YOUR CODE HERE
  """
  N = X1.shape[0]
  A = np.zeros((N, 9))

  for i in range(N):
    A[i] = np.matmul(X1[i].reshape(-1,1),X2[i].reshape(-1,1).T).reshape(-1)

#Last column of V is the null-space matrix of A which is e
  _,_,Vt = np.linalg.svd(A)
  E_est = (Vt[-1,:].reshape(3,3)).T  

  U,_,Vt = np.linalg.svd(E_est)
  
  E = U@np.diag((1,1,0))@Vt
  """ END YOUR CODE
  """
  return E
