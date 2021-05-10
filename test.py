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
importlib.reload(Planner)
importlib.reload(utils)
# %%
# path = [start]
numofdirs = 26 # 26 connected grid
[dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
dR = np.delete(dR,13,axis=1)
# dR = dR / np.linalg.norm(dR,axis=0) / 2.0
# %%
x = np.arange(-5, 5, 0.1)
y = np.arange(-5, 5, 0.1)
xx, yy = np.meshgrid(x, y, sparse=True)
# %%
x = deque([(3,3,3)])
# %%
x = [np.array([3,4,2])]
x.append(np.array([1,2,1]))
# %%
x = (0,0,0)
y = (1,3,4)
print(utils.dist(x,y))
# %%
