import numpy as np
import math
import timeit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

from quadrotor import Quadrotor3D
from controller import Controller

# def createTrajectory(sim_time, dt):
#     xref = []; yref = []; zref = []
#     for i in range(int(sim_time/dt)):
#         x = 1.0
#         y = 1.0
#         z = 0.5*dt*i
#         xref.append(x)
#         yref.append(y)
#         zref.append(z)
#     return np.array(xref), np.array(yref), np.array(zref)

if __name__ == "__main__":
    dt = 0.1
    N = 10
    
    quad = Quadrotor3D()
    path = []
    controller = Controller(quad, t_horizon=2*N*dt, n_nodes=N)

    goal = np.array([0,5,10])

    while np.linalg.norm(goal-quad.pos) > 0.1:
        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        thrust = controller.run_optimization(initial_state=current, goal=goal)[:4]
        quad.update(thrust, dt)
        path.append(quad.pos)


    path = np.array(path)
    print(path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(path[:,0], path[:,1], path[:,2])
    # ax.plot(xref, yref, zref)
    ax.scatter(goal[0], goal[1], goal[2], c=[1,0,0], label='goal')
    ax.axis('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()
    plt.show()
