import numpy as np
np.seterr(divide='ignore', invalid='ignore')


def discretize_line_segment(start, end, r=0.1):
  """discriteze a line segment

  Args:
    start (np array): starting point of line segment. shape (3,)
    end (np array): ending of line segment. shape (3,)
    r (float, optional): resolution. Defaults to 0.1.

  Returns:
    list: list of array of size (3,)
  """
  if not isinstance(start, np.ndarray):
    start = np.array(start)
  if not isinstance(end, np.ndarray):
    end = np.array(end)
  v = end - start
  return [start + k*v for k in np.arange(0, 1+r, r)]


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


def ray_cube_intersection(start, goal, obstacle):
  """
  Detect intersection with one obstacle

  obstacle: shape (6,): [xmin, ymin, zmin, xmax, ymax, zmax]
  return: true if given segment defined by start->goal intersect with
  the given cube.
  """
  if not isinstance(start, np.ndarray):
    start = np.array(start)
  if not isinstance(goal, np.ndarray):
    goal = np.array(goal)

  obstacle = obstacle.reshape(2, -1)
  delta = goal - start
  t = (obstacle - start) / delta
  t_min = np.nanmin(t, axis=0).max()
  t_max = np.nanmax(t, axis=0).min()

  return t_max > max(0, t_min) and ((0 <= t_min <= 1) or (0 <= t_max <= 1))


def ray_cubes_intersection(start, goal, obstacles):
  """
  start: tuple or array (3,)
  goal: tuple or array (3,)
  obstacles: shape (n_obstacles, 6)
  return: True if the ray intersects with any of the obstacles.
  """
  if not isinstance(start, np.ndarray):
    start = np.array(start)
  if not isinstance(goal, np.ndarray):
    goal = np.array(goal)

  obstacles = obstacles.reshape(-1, 2, 3) # obstacles.shape = (n_blocks, 2(min, max), 3(x,y,z))
  delta = goal - start + 1e-8 # delta.shape = (3,)
  t = (obstacles - start) / delta  # t.shape = (n_blocks, 2(min, max), 3(x,y,z))
  t_enter = np.min(t, axis=1).max(axis=1)  # t_enter.shape = (n_blocks,) ray first hit the box
  t_exit = np.max(t, axis=1).min(axis=1)  # t_exit.shape = (n_blocks,) ray exit the box

  return np.any(
                (t_exit > t_enter) &
                ((0 <= t_enter) & (t_enter <= 1))
               )
