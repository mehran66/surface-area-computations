import numpy as np
from numpy.linalg import inv

def polyfit2d(x, y, z, order):
    if order == 1: # Linear
        G = np.vstack([np.ones(len(x)), x, y]).T
    if order == 2: # BiLinear
        G = np.vstack([np.ones(len(x)), x, y, x*y]).T
    if order == 3: #BiQuadratic6
        G = np.vstack([np.ones(len(x)), x, y, x*y, x**2, y**2]).T
    if order == 4: #BiQuadratic9
        G = np.vstack([np.ones(len(x)), x, y, x*y, x**2, y**2, x**2 * y**2, x**2 * y, y**2 * x]).T
    if order == 5: #BiCubic
        G = np.vstack([np.ones(len(x)), x, y, x*y, x**2, y**2, x**3, y**3, x**2 * y, y**2 * x, x**3 * y, y**3 * x, x**2 * y**2, y**3 * x**3, y**3 * x**2, y**2 * x**3]).T

    m = np.matmul(inv(G), z)
    return m

def polyval2d(x, y, m):

    if len(m) == 3:
        z = np.dot(np.array([1, x, y]),m)
    if len(m) == 4:
        z = np.dot(np.array([1, x, y, x*y]),m)
    if len(m) == 6:
        z = np.dot(np.array([1, x, y, x*y, x**2, y**2]),m)
    if len(m) == 9:
        z = np.dot(np.array([1, x, y, x*y, x**2, y**2, x**2 * y**2, x**2 * y, y**2 * x]),m)
    if len(m) == 16:
        z = np.dot(np.array([1, x, y, x*y, x**2, y**2, x**3, y**3, x**2 * y, y**2 * x, x**3 * y, y**3 * x, x**2 * y**2, y**3 * x**3, y**3 * x**2, y**2 * x**3]),m)
    
    return z


