import os
import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

def tic():
  return time.time()


def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]                    # n obstacles
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))                 # 8 vertices 
  fl = np.zeros((6*n,4),dtype='int64')   # 6 faces
  fcl = np.zeros((6*n,3))                # face color
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, r=0.1):
  '''
  # res=1, eps=10, stopping_criteria="res"
  This function:
   * load the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # map name
  map_name = os.path.basename(mapfile).split('.')[0] # name of the map

  # convert start goal to tuple
  start = tuple(start)
  goal = tuple(goal)
  
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  boundary = boundary[0,:6].reshape(3,2,order="F") # [(x_lower, x_upper), (y_lower, y_upper), ...]
  blocks = blocks[:,:6] # blocks.shape = (n_obstacles, 6)

  # RRT config
  Q = np.array([(1, 1)])  # length of tree edges
  r = r  # length of smallest edge to check for intersection with obstacles
  max_samples = 15000  # max number of samples to take before timing out
  rewire_count = 32  # optional, number of nearby branches to rewire
  prc = 0.1  # probability of checking for a connection to goal

  # create Search Space
  X = SearchSpace(boundary, blocks)

  # create rrt_search
  rrt = RRTStar(X, Q, start, goal, max_samples, r, prc, rewire_count)
  t0 = tic()
  path = rrt.rrt_star()
  toc(t0,f"Planning for {os.path.basename(mapfile).split('.')[0]}")

  # plot
  plot = Plot(map_name)
  plot.plot_tree(X, rrt.trees)
  if path is not None:
    plot.plot_path(X, path)
  plot.plot_obstacles(X, blocks)
  plot.plot_start(X, start)
  plot.plot_goal(X, goal)
  plot.draw(auto_open=True)
  
  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection 
  goal_reached = path is not None # path is not none if goal is reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1))) if goal_reached else float("inf")
  return goal_reached, pathlength


def test_single_cube(verbose = False):
  print('Running single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength = runtest('./maps/single_cube.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')
  

def test_maze(verbose = False):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/maze.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = False):
  print('Running window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, pathlength = runtest('./maps/window.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(verbose = False):
  print('Running tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength = runtest('./maps/tower.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_flappy_bird(verbose = False):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength = runtest('./maps/flappy_bird.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = False):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest('./maps/room.txt', start, goal)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(verbose = False):
  print('Running monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  # start = np.array([0.5, 1.0, 0.1])
  goal = np.array([3.8, 1.0, 0.1])
  # goal = np.array([3.8, 20.0, 0.1])
  success, pathlength = runtest('./maps/monza.txt', start, goal, r=0.05)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


if __name__=="__main__":
  # test_single_cube(True)
  # test_maze(True)
  test_flappy_bird(True)
  # test_monza(True)  
  # test_window(True)
  # test_tower(True) 
  # test_room(True)