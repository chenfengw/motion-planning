import numpy as np

def discretize_line_segment(start, end, r=0.1):
  """discriteze a line segment

  Args:
    start (np array): starting point of line segment. shape (3,)
    end (np array): ending of line segment. shape (3,)
    r (float, optional): resolution. Defaults to 0.1.

  Returns:
    list: list of array of size (3,)
  """
  if not isinstance(start,np.ndarray):
    start = np.array(start)
  if not isinstance(end,np.ndarray):
    end = np.array(end)
  v = end - start
  return [start + k*v for k in np.arange(0,1+r,r)]

def dist(point1, point2):
  """
  distance between two points
  point1, point2: shape (3,)
  """
  if not isinstance(point1, np.ndarray):
    point1 = np.array(point1)
  if not isinstance(point2, np.ndarray):
    point2 = np.array(point2)
  return np.linalg.norm(point1 - point2)