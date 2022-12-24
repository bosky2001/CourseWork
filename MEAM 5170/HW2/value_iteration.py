import numpy as np
from math import *
from grid_world import *

import matplotlib.pyplot as plt

def plot_value_function_and_optimal_policy(world, V, u_opt):
  plt.clf()
  v_plot = plt.imshow(V, interpolation='nearest')
  colorbar = plt.colorbar()
  colorbar.set_label("Value function")
  plt.xlabel("Column")
  plt.ylabel("Row")
  arrow_length = 0.25
  for row in range(world.rows):
    for col in range(world.cols):
      if u_opt[row, col] == 0: #N
        plt.arrow(col, row, 0, -arrow_length, head_width=0.1)
      elif u_opt[row, col] == 1: #E
        plt.arrow(col, row, arrow_length, 0, head_width=0.1)
      elif u_opt[row, col] == 2: #S
        plt.arrow(col, row, 0, arrow_length, head_width=0.1)
      elif u_opt[row, col] == 3: #W
        plt.arrow(col, row, -arrow_length, 0, head_width=0.1)
      else:
        raise ValueError("Invalid action")
  plt.savefig('value_function.png', dpi=240)
  plt.show()

def value_iteration(world, threshold, gamma, plotting=True):
  V = np.zeros((world.rows, world.cols))
  Q = np.zeros((world.rows, world.cols, 4))
  u_opt = np.zeros((world.rows, world.cols))
  grid_x, grid_y = np.meshgrid(np.arange(0, world.rows, 1), 
                               np.arange(0, world.cols, 1))
  delta = 10.0

  fig = plt.figure("Gridworld")
  # TODO: calculate V and the optimal policy u using value iteration

  # Begin solution code here
  while delta > threshold:
    delta = 0
    for col in range(world.cols):
      for row in range(world.rows):
        V_temp = V[row, col]
        Q[row, col] = np.zeros(4)
        state = world.map_row_col_to_state(row, col)
        for action in range(4):
          prob, next_state, cost = world.eval_action(state, action)[0]
          Q[row, col, action] += prob * (cost + gamma * V[world.map_state_to_row_col(next_state)])
          prob, next_state, cost = world.eval_action(state, action)[1]
          Q[row, col, action] += prob * (cost + gamma * V[world.map_state_to_row_col(next_state)])
        V[row, col] = np.min(Q[row, col])
        delta = np.maximum(delta, np.absolute(V_temp - V[row, col]))
    if plotting:
      plt.clf()
      v_plot = plt.imshow(V, interpolation='nearest')
      plt.pause(0.05)

  u_opt = np.argmin(Q, axis = 2)

  # End solution code
  if plotting:
    plot_value_function_and_optimal_policy(world, V, u_opt)

  return V, u_opt

if __name__=="__main__":
  world = GridWorld() 
  threshold = 0.000001
  gamma = 0.9
  # value_iteration(world, threshold, gamma, False)
  value_iteration(world, threshold, gamma, False)
