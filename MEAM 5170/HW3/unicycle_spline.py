import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from random import uniform
from numpy import sqrt
def unicycle_spline(t0, tf, obs):
  # UNICYCLE_SPLINE returns a spline object representing a path from
  # (y(t0),z(t0)) = (0,0) to (y(t0),z(t0)) = (10,0) that avoids a circular
  # obstacle, such that d\dt y(t) > 0
  #   @param t0 - initial time
  #   @param tf - final time
  #
  #   @return y_spline - spline object for desired y trajectory
  #   @return z_spline - spline object for desired z trajectory
  y0 = 0;
  z0 = 0;
  yf = 10;
  zf = 0;

  t = np.array([t0, tf])
  y = np.array([y0, yf])
  z = np.array([z0, zf])
  y_spline = CubicSpline(t, y);
  z_spline = CubicSpline(t, z);
  # start of solution
  t = np.linspace(t0, tf, 3)
  y = np.array([y0, obs.y, yf])
  if obs.z > 0:
    z = np.array([z0, obs.z - obs.radius - 1, zf])
  else:
    z = np.array([z0, obs.z + obs.radius + 1, zf])
  y_spline = CubicSpline(t, y);
  z_spline = CubicSpline(t, z);


  return y_spline, z_spline
