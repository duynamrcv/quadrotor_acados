import numpy as np
import math
import matplotlib.pyplot as plt

from quadrotor import Quad

if __name__ == "__main__":
    quad = Quad(filename="quad.yaml")
    T = np.array([4,4,4,4])
    for i in range(10):
        quad.updateState(T, 0.1)

    path = np.array(quad.path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(path[:,0], path[:,1], path[:,2])
    plt.show()
