# %% 
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from pqdict import pqdict

# %%
def edge_cost(i, j, lambd, data):
    """
    edge cost between node i and j
    lambd (double): regularization term
    data (array): n_sample x 2
    """
    x = data[i:j+1, 0]  # data, X
    y = data[i:j+1, 1]  # labels
    A = np.vstack((x, np.ones(x.shape[0]))).T
    
    coef, _, _, _ = np.linalg.lstsq(A, y, rcond=None)

    return (lambd + np.linalg.norm(y - A @ coef)**2)


# %%
def dijkstra(children, arrival_costs, open_pq, lambd, data, start, goal):
    parent = {}

    while open_pq:
        node_i, label_i = open_pq.popitem()
        # iterate all children of node i
        for node_j in children[node_i]:
            new_cost = label_i + edge_cost(node_i, node_j, lambd, data) # g_j
            
            # see if node j is worth going
            if new_cost < arrival_costs[node_j] and new_cost < arrival_costs[goal]:
                arrival_costs[node_j] = new_cost
                parent[node_j] = node_i
                
                # update label for node j in open_pq
                if node_j in open_pq.keys():
                    open_pq[node_j] = new_cost

                # put new node in to open_pq
                if node_j not in open_pq.keys() and node_j != goal:
                    open_pq[node_j] = new_cost
    
    # retrieve optimal path
    optimal_path = deque([goal])
    node = goal
    while node != start:
        optimal_path.appendleft(parent[node])
        node = parent[node]

    return list(optimal_path)

# %% load data
data = np.load("data_q3.npy")

# %% create the tree, children of all nodes
lambd = 5
start = 0
goal = 99

children = {key: np.arange(key+1, data.shape[0]) for key in np.arange(data.shape[0])}
arrival_costs = {node: 0 if node==0 else float("inf") for node in np.arange(data.shape[0])}
open_pq = pqdict({0:0})

optimal_path = dijkstra(children, arrival_costs, open_pq, lambd, data, start, goal)
optimal_path = list(optimal_path)
# %%
plt.scatter(data[:,0], data[:,1])
for start, end in zip(optimal_path[:-1],optimal_path[1:]):
    x = data[start:end+1, 0]  # data, X
    y = data[start:end+1, 1]  # labels
    A = np.vstack((x, np.ones(x.shape[0]))).T
    
    (m,c), _, _, _ = np.linalg.lstsq(A, y, rcond=None)
    plt.plot(x, m*x + c)
plt.title(f"lambda: {lambd}")
plt.savefig(f"hw2_q3_lamda{lambd}.svg" , bbox_inches="tight")
# %%
