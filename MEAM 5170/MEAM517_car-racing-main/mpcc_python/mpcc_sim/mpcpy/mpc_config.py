import numpy as np


class Params:
    def __init__(self):
        self.N = 4  # number of state variables
        self.M = 2  # number of control variables
        self.T = 8  # Prediction Horizon
        self.DT = 0.  # discretization step
        self.path_tick = 0.05
        self.L = 0.2  # vehicle wheelbase
        self.MAX_SPEED = 2.5 # m/s
        self.MAX_ACC = 2.0  # m/ss
        self.MAX_D_ACC = 1.5  # m/sss
        self.MAX_STEER = np.radians(60)  # rad
        self.MAX_D_STEER = np.radians(45)  # rad/s
