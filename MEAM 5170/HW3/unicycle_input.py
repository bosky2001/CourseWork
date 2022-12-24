import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def unicycle_input(t : float, y_spline : CubicSpline, z_spline: CubicSpline) -> np.ndarray:
  #UNICYCLE_INPUT returns input to the unicycle
  #   @param t - current time
  #   @param y_spline - spline object for desired y trajectory
  #   @param z_spline - spline object for desired z trajectory
  #   
  #   @return u - input u(t) to the unicycle system

  # TODO: modify u to return the correct input for time t.
  u = np.zeros(2);
  u[0] = (y_spline(t,1)*z_spline(t,2)-z_spline(t,1)*y_spline(t,2))/(z_spline(t,1)**2 + y_spline(t,1)**2)
  u[1] = np.sqrt([z_spline(t,1)**2 + y_spline(t,1)**2])  

  return u
