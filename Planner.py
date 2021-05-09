import numpy as np
import geometry_utils

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

  def does_segment_clear_obstacles(self, start, end, r):
    """
    Check if the line segment formed by start and end clears from all obstacles. 
    return True if the linesegment does not intersects with any obstacle.
    
    start: the beginning of line segment. start.shape = (3,)
    end: the end of the line segment. end.shape = (3,)
    r: double, grid resolution
    """
    points = geometry_utils.discretize_line_segment(start, end, r=r)
    return all(map(self.is_point_outside_obstacles, points))