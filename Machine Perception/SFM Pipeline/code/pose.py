import numpy as np

def Rz(theta):
  return np.matrix([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
def pose_candidates_from_E(E):
  transform_candidates = []
  ##Note: each candidate in the above list should be a dictionary with keys "T", "R"
  """ YOUR CODE HERE
  """
  # better to define a function for Rz given angle?
  # Rz = np.matrix('0 -1 0; 1 0 0; 0 0 1')


  U, _, V = np.linalg.svd(E)

  candidate1 = {'T':U[:,-1], 'R':U @ Rz(np.pi/2).T @V}
  candidate2 = {'T':U[:,-1], 'R':U @ Rz(-np.pi/2).T @V}
  candidate3 = {'T': -U[:,-1], 'R':U @ Rz(np.pi/2).T @V}
  candidate4 = {'T': -U[:,-1], 'R':U @ Rz(-np.pi/2).T @V}

  transform_candidates.append(candidate1)
  transform_candidates.append(candidate2)
  transform_candidates.append(candidate3)
  transform_candidates.append(candidate4)
  
  """ END YOUR CODE
  """
  return transform_candidates