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

def poly_345(h : float, ts : float, v0 : float = 0, a0 : float = 0, vf : float = 0, af : float = 0):
  coeff = []
  coeff.append(0)
  coeff.append(v0)
  coeff.append(a0/2)

  a = np.ndarray(shape=(3,3))
  a[0,0] = pow(ts,3)
  a[0,1] = pow(ts,4)
  a[0,2] = pow(ts,5)
  a[1,0] = 3*pow(ts,2)
  a[1,1] = 4*pow(ts,3)
  a[1,2] = 5*pow(ts,4)
  a[2,0] = 6*ts
  a[2,1] = 12*pow(ts,2)
  a[2,2] = 20*pow(ts,3)

  b = np.ndarray(shape=(3,1))
  b[0,0] = h - coeff[0] - coeff[1]*ts - coeff[2]*pow(ts,2)
  b[1,0] = vf - coeff[1] - 2*coeff[2]*ts
  b[2,0] = af - 2*coeff[2]

  x = np.linalg.solve(a,b)
  
  coeff.append(x[0])
  coeff.append(x[1])
  coeff.append(x[2])

  return coeff

def poly_pos(coeff : list, t : float):
  y = coeff[0] + coeff[1]*t + coeff[2]*pow(t,2) + coeff[3]*pow(t,3) + coeff[4]*pow(t,4) + coeff[5]*pow(t,5)
  return y

def poly_vel(coeff : list, t : float):
  y = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*pow(t,2) + 4*coeff[4]*pow(t,3) + 5*coeff[5]*pow(t,4)
  return y

def poly_accel(coeff : list, t : float):
  y = 2*coeff[2] + 6*coeff[3]*t + 12*coeff[4]*pow(t,2) + 20*coeff[5]*pow(t,3)
  return y

def poly_7(h : float, ts : float, v0 : float = 0, a0 : float = 0, vf : float = 0, af : float = 0, j0 : float = 0, jf : float = 0):
  
  coeff = []

  coeff.append(0)
  coeff.append(v0)
  coeff.append(a0/2)
  coeff.append(j0/6)

  a = np.ndarray(shape=(4,4))

  a[0,0] = pow(ts,4)
  a[0,1] = pow(ts,5)
  a[0,2] = pow(ts,6)
  a[0,3] = pow(ts,7)

  a[1,0] = 4*pow(ts,3)
  a[1,1] = 5*pow(ts,4)
  a[1,2] = 6*pow(ts,5)
  a[1,3] = 7*pow(ts,6)

  a[2,0] = 12*pow(ts,2)
  a[2,1] = 20*pow(ts,3)
  a[2,2] = 30*pow(ts,4)
  a[2,3] = 42*pow(ts,5)

  a[3,0] = 24*ts
  a[3,1] = 60*pow(ts,2)
  a[3,2] = 120*pow(ts,3)
  a[3,3] = 210*pow(ts,4)

  b = np.ndarray(shape=(4,1))
  b[0,0] = h - coeff[0] - coeff[1]*ts - coeff[2]*pow(ts,2) - coeff[3]*pow(ts,3)
  b[1,0] = vf - coeff[1] - 2*coeff[2]*ts - 3*coeff[3]*pow(ts,2)
  b[2,0] = af - 2*coeff[2] - 6*coeff[3]*pow(ts,2)
  b[3,0] = jf - 6*coeff[3]

  x = np.linalg.solve(a,b)
  
  coeff.append(x[0])
  coeff.append(x[1])
  coeff.append(x[2])
  coeff.append(x[3])

  return coeff

def poly_pos_7(coeff : list, t : float):
  y = coeff[0] + coeff[1]*t + coeff[2]*pow(t,2) + coeff[3]*pow(t,3) + coeff[4]*pow(t,4) + coeff[5]*pow(t,5) + coeff[6]*pow(t,6) + coeff[7]*pow(t,7)
  return y

def poly_vel_7(coeff : list, t : float):
  y = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*pow(t,2) + 4*coeff[4]*pow(t,3) + 5*coeff[5]*pow(t,4) + 6*coeff[6]*pow(t,5) + 7*coeff[7]*pow(t,6) 
  return y

def poly_accel_7(coeff : list, t : float):
  y = 2*coeff[2] + 6*coeff[3]*t + 12*coeff[4]*pow(t,2) + 20*coeff[5]*pow(t,3) + 30*coeff[6]*pow(t,4) + 42*coeff[7]*pow(t,5) 
  return y
