# %%
import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import importlib
from collections import deque
import Planner
import utils
import main
importlib.reload(Planner)
importlib.reload(utils)
np.seterr(divide='ignore', invalid='ignore')
# %%
boundary, blocks = main.load_map("./maps/window.txt")
start = np.array([0.2, -4.9, 0.2])
goal = np.array([6.0, 18.0, 3.0])
# %%
obstacles = blocks[0,:6]
delta = np.tile(goal - start, 2)
start_xyz = np.tile(start, 2)
t = (obstacles - start_xyz) / delta

t_min1, t_min2, t_min3 = min(t[0], t[0+3]), min(t[1], t[1+3]), min(t[2], t[2+3])
t_max1, t_max2, t_max3 = max(t[0], t[0+3]), max(t[1], t[1+3]), max(t[2], t[2+3]) 
t_min = max(t_min1, t_min2, t_min3)
t_max = min(t_max1, t_max2, t_max3)
print(f"t_min {t_min}, t_max {t_max}")
# %%
obstacles = blocks[0,:6].reshape(2,-1)
delta = goal - start
t = (obstacles - start) / delta
t_min = np.nanmin(t,axis=0).max()
t_max = np.nanmax(t,axis=0).min()
print(f"t_min {t_min}, t_max {t_max}")
# %%
def ray_cube_intersection(start, goal, obstacles):
    """
    obstacles: shape (6,): [xmin, ymin, zmin, xmax, ymax, zmax]
    return true if given segment defined by start->goal intersect with
    the cube 
    """
    obstacles = obstacles.reshape(2,-1)
    delta = goal - start
    t = (obstacles - start) / delta
    t_min = np.nanmin(t,axis=0).max()
    t_max = np.nanmax(t,axis=0).min()

    # print(f"t: {t}")
    # print(f"t_max: {t_max}")
    # print(f"t_max: {t_max}")
    return t_max > max(0,t_min) and ((0 <= t_min <= 1) or (0 <= t_max <= 1))
#%%

def ray_cubes_intersection(start, goal, obstacles):
    obstacles = obstacles.reshape(-1,2,3) # obstacles.shape = (n_blocks, 2(min, max), 3(x,y,z))
    delta = goal - start
    t = (obstacles - start) / delta # t.shape = (n_blocks, 2(min, max), 3(x,y,z))
    t_min = np.nanmin(t,axis=1).max(axis=1) # t_min.shape = (n_blocks,)
    t_max = np.nanmax(t,axis=1).min(axis=1) # t_max.shape = (n_blocks,)

    # return np.any()
    # print(f"t: {t}")
    print(f"t_min: {t_min}")
    print(f"t_max: {t_max}")
    # return np.any((t_max > t_min) &
    #               ((0 <= t_min) & (t_min <= 1)) |
    #               ((0 <= t_max) & (t_max <= 1)))
    return  (t_max > t_min) &\
            ( ((0 <= t_min) & (t_min <= 1)) | ((0 <= t_max) & (t_max <= 1)) )

    # return t_max
# %% test cubes. vectorized version
t_min = ray_cubes_intersection(start, goal, blocks[:,:6])
# %% for non vectorized verison
for block_idx in range(blocks.shape[0]):
    print(ray_cube_intersection(start, goal, blocks[block_idx, :6]))
# %%
