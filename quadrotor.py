import yaml
from quaternion import *

class Quad:
    def __init__(self, filename = ""):
        self.m = 1                                        # mass in [kg]
        self.l = 1                                        # arm length
        self.I = np.array([[1,0,0],[0,1,0],[0,0,1]])      # Inertia
        self.I_inv = np.linalg.inv(self.I)                # Inertia inverse
        self.T_max = 5                                    # max thrust [N]
        self.T_min = 0                                    # min thrust [N]
        # self.omega_max = 3                                # max bodyrate [rad/s]
        self.ctau = 0.5                                   # thrust torque coeff.
        self.rampup_dist = 0
        self.T_ramp_start = 5
        self.omega_ramp_start = 3

        self.v_max = None
        self.cd = 0.0

        self.g = 9.801

        if filename:
            self.load(filename)

        self.p = np.zeros([3,1])    # position
        self.q = np.zeros([4,1])    # quaternion rotation [w,x,y,z]
        self.q[0,0] = 1
        self.v = np.zeros([3,1])    # velocity
        self.w = np.zeros([3,1])    # bodyrate
        self.T = np.zeros([4,1])    # thrust

        self.path = [np.concatenate([self.p, self.q, self.v, self.w])[:,0]]

    def updateState(self, T, dt):

        # x = np.array([p, v, q, w])
        self.T = np.array(T)
        g = np.array([[0, 0, -self.g]]).T

        # dynamic model
        p_dot = self.v
        q_dot = 0.5*quat_mult(self.q, np.append([[0]], self.w, axis=0))
        v_dot = rotate_quat(self.q, np.array([[0, 0, (self.T[0]+self.T[1]+self.T[2]+self.T[3])/self.m]]).T) + g - self.v * self.cd
        w_dot = self.I_inv@(np.array([[
                self.l*(self.T[0]-self.T[1]-self.T[2]+self.T[3]),
                self.l*(-self.T[0]-self.T[1]+self.T[2]+self.T[3]),
                self.ctau*(self.T[0]-self.T[1]+self.T[2]-self.T[3])]]).T - np.cross(self.w, self.I@self.w, axisa=0, axisb=0).T)
        # print(np.array([[0, 0, (self.T[0]+self.T[1]+self.T[2]+self.T[3])/self.m]]).T)
        self.p = self.p + p_dot*dt
        self.q = self.q + q_dot*dt
        self.v = self.v + v_dot*dt
        self.w = self.w + w_dot*dt

        # print(self.p.shape, self.q.shape, self.v.shape, self.w.shape)
        self.path.append(np.concatenate([self.p, self.q, self.v, self.w])[:,0])


    def load(self, filename):
        print("Loading track from " + filename)
        with open(filename, 'r') as file:
            quad = yaml.load(file, Loader=yaml.FullLoader)

        if 'mass' in quad:
            self.m = quad['mass']
        else:
            print("No mass specified in " + filename)

        if 'arm_length' in quad:
            self.l = quad['arm_length']
        else:
            print("No arm length specified in " + filename)

        if 'inertia' in quad:
            self.I = np.array(quad['inertia'])
            self.I_inv = np.linalg.inv(self.I)
        else:
            print("No inertia specified in " + filename)


        if 'TWR_max' in quad:
            self.T_max = quad['TWR_max'] * 9.81 * self.m / 4
        elif 'thrust_max' in quad:
            self.T_max = quad['thrust_max']
        else:
            print("No max thrust specified in " + filename)

        if 'TWR_min' in quad:
            self.T_min = quad['TWR_min'] * 9.81 * self.m / 4
        elif 'thrust_min' in quad:
            self.T_min = quad['thrust_min']
        else:
            print("No min thrust specified in " + filename)

        if 'omega_max_xy' in quad:
            self.omega_max_xy = quad['omega_max_xy']
        else:
            print("No max omega_xy specified in " + filename)

        if 'omega_max_z' in quad:
            self.omega_max_z = quad['omega_max_z']
        else:
            print("No max omega_z specified in " + filename)

        if 'torque_coeff' in quad:
            self.ctau = quad['torque_coeff']
        else:
            print("No thrust to drag coefficient specified in " + filename)

        if 'v_max' in quad:
            self.v_max = quad['v_max']
            a_max = 4 * self.T_max / self.m
            a_hmax = np.sqrt(a_max**2 - self.g**2)
            self.cd = a_hmax / self.v_max
        if 'drag_coeff' in quad:
            self.cd = quad['drag_coeff']

        if 'rampup_dist' in quad:
            self.rampup_dist = quad['rampup_dist']
            if 'TWR_ramp_start' in quad and 'omega_ramp_start' in quad:
                self.T_ramp_start = min(quad['TWR_ramp_start'] * 9.81 * self.m / 4, self.T_max)
                self.omega_ramp_start = min(quad['omega_ramp_start'], self.omega_max_xy)
            else:
                print("No TWR_ramp_start or omega_ramp_start specified. Disabling rampup")
                rampup_dist = 0