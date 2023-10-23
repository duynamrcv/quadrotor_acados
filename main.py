import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

from quadrotor import Quad

if __name__ == "__main__":
    quad = Quad(filename="quad.yaml")
    thrust = np.array([5,5,5,5])
    for i in range(10):
        quad.updateState(thrust, 0.1)

    path = np.array(quad.path)
    # print(path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(path[:,0], path[:,1], path[:,2])
    ax.scatter(path[0,0], path[0,1], path[0,2])
    plt.show()
