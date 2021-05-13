import numpy as np
import utils
from pqdict import pqdict
from collections import defaultdict, deque

class BasePlanner:
  def __init__(self, boundary, blocks):
    """Initialize planner with boundary of the space and obstacles

    Args:
        boundary (np array): shape: (1, 9). each row ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']
        blocks (np array): shape: (n_obstacles, 9). each row ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']
    """
    self.boundary = boundary
    self.blocks = blocks
    self.n_obstacles = blocks.shape[0]

class PathChecker(BasePlanner):
  def __init__(self, boundary, blocks):
    super().__init__(boundary, blocks)
  
  def is_point_inside_boundary(self, point):
    """
    point: the point you want to check. point.shape = (3,)
    """
    if point[0] < self.boundary[0,0] or point[0] > self.boundary[0,3] or \
       point[1] < self.boundary[0,1] or point[1] > self.boundary[0,4] or \
       point[2] < self.boundary[0,2] or point[2] > self.boundary[0,5]:
      return False
    else:
      return True
  
  def is_point_inside_obstacles(self, point):
    """
    check if a point is inside any obstacles
    point: the point you want to check. point.shape = (3,)
    """
    for k in range(self.n_obstacles):
      if point[0] > self.blocks[k,0] and point[0] < self.blocks[k,3] and\
         point[1] > self.blocks[k,1] and point[1] < self.blocks[k,4] and\
         point[2] > self.blocks[k,2] and point[2] < self.blocks[k,5]:
        return True
    return False
  
  def is_point_outside_obstacles(self, point):
    """
    check if a point is out all obstacles
    point: the point you want to check. point.shape = (3,)
    """
    return not self.is_point_inside_obstacles(point)

  def does_segment_intersect_obstacles(self, start, end, defaultpass=False):
    """
    Test if line segment intersect with all obstacles

    return: True if intersect with at least one obstacles.
    """
    if defaultpass:
      return False

    return utils.ray_cubes_intersection(start, end, self.blocks[:,:6])

  def is_near_goal(self, point, goal, d=0.1):
    """
    Check if given point is near goal

    point: shape (3,)
    goal: shape (3,)
    """
    return utils.dist(point, goal) <= d

class MyPlanner(BasePlanner):
  def __init__(self, boundary, blocks):
    super().__init__(boundary, blocks)

  def plan(self,start,goal):
    path = [start]
    numofdirs = 26 # 26 connected grid
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k] # next.shape = (3,)
        print(f"next is {next}, shape is {next.shape}, type is {type(next)}")

        # Check if this direction is valid
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue
        
        valid = True
        for k in range(self.blocks.shape[0]):
          if( next[0] > self.blocks[k,0] and next[0] < self.blocks[k,3] and\
              next[1] > self.blocks[k,1] and next[1] < self.blocks[k,4] and\
              next[2] > self.blocks[k,2] and next[2] < self.blocks[k,5] ):
            valid = False
            break
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

class AStarPlanner:
  def __init__(self, boundary, blocks, res=0.1):
    # res: resolution of the 26 connected grid
    self.checker = PathChecker(boundary, blocks)
    self.res = res
  
  def plan(self, start, goal, eps=10):
    """
    Plan optimal path using A*
    start: np array, shape (3,)
    goal: np array, shape (3,)
    """
    start = tuple(start)
    goal = tuple(goal)
    numofdirs = 26 # 26 connected grid
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten())) # dR.shape = (3,27)
    dR = np.delete(dR,13,axis=1)  # delete [0,0,0]
    dR = dR / np.linalg.norm(dR,axis=0) * self.res

    # initialize label
    arrival_costs = defaultdict(lambda:float("inf"))
    arrival_costs[start] = 0
    open_pq = pqdict({start: 0})
    closed = set()
    parent = {}
    node_j = start

    # main A* algorithm
    while utils.dist(node_j, goal) > self.res:
      node_i, _ = open_pq.popitem() # node_i is tuple
      closed.add(node_i)

      # iterate all children of node i
      for k in range(numofdirs):
        node_j = np.array(node_i) + dR[:,k] # node_j.shape = (3,)
        node_j = tuple(node_j)

        # check if node_j is valid node
        if node_j not in closed and \
           self.checker.is_point_inside_boundary(node_j) and \
           not self.checker.does_segment_intersect_obstacles(node_i, node_j, defaultpass=False):

          new_cost = arrival_costs[node_i] + utils.dist(node_i, node_j) # g_j
          
          # see if node j is worth going
          if new_cost < arrival_costs[node_j]:
            arrival_costs[node_j] = new_cost
            parent[node_j] = node_i
            
            # check for early stoping
            if self.checker.is_near_goal(node_j, goal, d=self.res):
              return AStarPlanner.get_optimal_path(start, node_j, parent)

            # update label for node j in open_pq or add node_j
            open_pq[node_j] = new_cost + eps*AStarPlanner.heuristic(node_j, goal)

    # retrieve the optimal path
    return AStarPlanner.get_optimal_path(start, node_j, parent)

  @staticmethod 
  def heuristic(point, goal):
    return utils.dist(point, goal)

  @staticmethod
  def get_optimal_path(start, goal, parent):
    """
    start: tuple (x,y,z)
    goal: tuple (x,y,z)
    """
    # retrieve optimal path
    optimal_path = deque([goal])
    node = goal
    
    while node != start:
      optimal_path.appendleft(parent[node])
      node = parent[node]

    return np.array(optimal_path)


