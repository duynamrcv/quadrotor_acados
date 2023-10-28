import numpy as np
import math
import timeit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

from quadrotor import Quadrotor3D
from controller import Controller

def createTrajectory(sim_time, dt):
    xref = []; yref = []; zref = []
    for i in range(int(sim_time/dt)):
        x = 1.0
        y = 0.5*dt*i
        z = 1.0
        xref.append(x)
        yref.append(y)
        zref.append(z)
    return np.array(xref), np.array(yref), np.array(zref)

def move2Goal():
    dt = 0.1    # Time step
    N = 10      # Horizontal length
    
    quad = Quadrotor3D()    # Quadrotor model
    controller = Controller(quad, t_horizon=2*N*dt, n_nodes=N)  # Initialize MPC controller

    goal = np.array([0,5,10])
    path = []

    # Main loop
    while np.linalg.norm(goal-quad.pos) > 0.1:
        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        thrust = controller.run_optimization(initial_state=current, goal=goal)[:4]
        quad.update(thrust, dt)
        path.append(quad.pos)

    # Visualization
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

def trackTrajectory():
    dt = 0.1    # Time step
    N = 10      # Horizontal length
    
    quad = Quadrotor3D()    # Quadrotor model
    controller = Controller(quad, t_horizon=2*N*dt, n_nodes=N)  # Initialize MPC controller

    sim_time = 10
    xref, yref, zref = createTrajectory(sim_time, dt)
    path = []

    # Main loop
    time_record = []
    for i in range(int(sim_time/dt)):
        # print(i)
        x = xref[i:i+N+1]; y = yref[i:i+N+1]; z = zref[i:i+N+1]
        if len(x) < N+1:
            x = np.concatenate((x,np.ones(N+1-len(x))*xref[-1]),axis=None)
            y = np.concatenate((y,np.ones(N+1-len(y))*yref[-1]),axis=None)
            z = np.concatenate((z,np.ones(N+1-len(z))*zref[-1]),axis=None)
        goal=np.array([x,y,z]).T
        # print(goal)

        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        start = timeit.default_timer()
        thrust = controller.run_optimization(initial_state=current, goal=goal, mode='traj')[:4]
        time_record.append(timeit.default_timer() - start)
        quad.update(thrust, dt)
        path.append(quad.pos)

    # CPU time
    print("average estimation time is {:.5f}".format(np.array(time_record).mean()))
    print("max estimation time is {:.5f}".format(np.array(time_record).max()))
    print("min estimation time is {:.5f}".format(np.array(time_record).min()))

    # Visualization
    path = np.array(path)
    # print(path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(xref, yref, zref, c=[1,0,0], label='goal')
    ax.plot(path[:,0], path[:,1], path[:,2])
    ax.axis('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()

    plt.figure()
    plt.plot(time_record)
    plt.legend()
    plt.ylabel('CPU Time [s]')
    # plt.yscale("log")

    plt.show()

if __name__ == "__main__":
    # move2Goal()
    trackTrajectory()