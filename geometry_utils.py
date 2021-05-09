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
    v = end - start
    return [start + k*v for k in np.arange(0,1+r,r)]