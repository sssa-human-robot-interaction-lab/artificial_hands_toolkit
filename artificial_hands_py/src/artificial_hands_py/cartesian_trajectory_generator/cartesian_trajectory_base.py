from cmath import pi
import numpy as np

def harmonic_pos(h : float, t : float, ts : float):
  y = h*(t/ts - 1/(2*pi)*np.sin(2*pi*t/ts))
  return y

def harmonic_vel(h : float, t : float, ts : float):
  y = h/ts*(1 - np.cos(2*pi*t/ts))
  return y

def harmonic_accel(h : float, t : float, ts : float):
  y = 2*pi*h/pow(ts,2)*np.sin(2*pi*t/ts)
  return y
