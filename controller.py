import casadi as ca
import numpy as np
import math
import scipy.linalg
import os
import sys
import shutil
import errno

from quaternion import *
from quadrotor import Quad

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel

class MPCController:
    def __init__(self, quad:Quad, t_horizon=1.0, n_nodes=10):
        self.quad = quad
        self.quadModel()
        self.setupController(t_horizon, n_nodes)

    def quadModel(self):
        self.model = AcadosModel() #  ca.types.SimpleNamespace()
        self.constraint = ca.types.SimpleNamespace()
        
        # # control inputs
        # a = ca.SX.sym('a')
        # omega = ca.SX.sym('omega')
        # controls = ca.vertcat(a, omega)

        # # model states
        # x = ca.SX.sym('x')
        # y = ca.SX.sym('y')
        # psi = ca.SX.sym('psi')
        # v = ca.SX.sym('v')
        # delta = ca.SX.sym('delta')
        # states = ca.vertcat(x, y, psi, v, delta)

        # # acados model
        # x_dot = ca.SX.sym('x_dot')
        # y_dot = ca.SX.sym('y_dot')
        # psi_dot = ca.SX.sym('psi_dot')
        # v_dot = ca.SX.sym('v_dot')
        # delta_dot = ca.SX.sym('delta_dot')
        
        # state_dot = ca.vertcat(x_dot, y_dot, psi_dot, v_dot, delta_dot)
        # f_expl = ca.vertcat(v*ca.cos(psi),
        #                     v*ca.sin(psi),
        #                     v*ca.tan(delta)/self.quad.L,
        #                     a,
        #                     omega)

        # self.model.f_expl_expr = f_expl
        # self.model.f_impl_expr = state_dot - f_expl
        # self.model.x = states
        # self.model.xdot = x_dot
        # self.model.u = controls
        # self.model.p = []
        # self.model.name = 'quad'

        # # constraint
        # self.constraint.v_max = self.quad.max_v
        # self.constraint.v_min = self.quad.min_v
        # self.constraint.delta_max = self.quad.max_delta
        # self.constraint.delta_min = self.quad.min_delta

        # self.constraint.a_max = self.quad.max_a
        # self.constraint.a_min = self.quad.min_a
        # self.constraint.omega_max = self.quad.max_omega
        # self.constraint.omega_min = self.quad.min_omega

        # self.constraint.expr = ca.vcat([a, omega])

    def setupController(self, t_horizon, n_nodes):
        '''
        t_horizon: time horizon for MPC optimization
        n_nodes: number of optimization nodes until time horizon'''
        self.T = t_horizon
        self.N = n_nodes

        # Ensure current working directory is current folder
        # acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        # sys.path.insert(0, acados_source_path)

        # nx = self.model.x.size()[0]
        # self.nx = nx
        # nu = self.model.u.size()[0]
        # self.nu = nu
        # ny = nx + nu
        # n_params = len(self.model.p)

        # # create OCP
        # ocp = AcadosOcp()
        # # ocp.acados_include_path = acados_source_path + '/include'
        # # ocp.acados_lib_path = acados_source_path + '/lib'
        # ocp.model = self.model
        # ocp.dims.N = self.N
        # ocp.solver_options.tf = self.T

        # # initialize parameters
        # ocp.dims.np = n_params
        # ocp.parameter_values = np.zeros(n_params)

        # # cost type
        # Q = np.diag([100.0, 100.0, 1.0, 1.0, 1.0])
        # R = np.diag([0.1, 0.1])
        # ocp.cost.cost_type = 'LINEAR_LS'
        # ocp.cost.cost_type_e = 'LINEAR_LS'
        # ocp.cost.W = scipy.linalg.block_diag(Q, R)
        # ocp.cost.W_e = Q
        # ocp.cost.Vx = np.zeros((ny, nx))
        # ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        # ocp.cost.Vu = np.zeros((ny, nu))
        # ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        # ocp.cost.Vx_e = np.eye(nx)

        # # set constraints
        # ocp.constraints.lbx = np.array([self.constraint.v_min, self.constraint.delta_min])
        # ocp.constraints.ubx = np.array([self.constraint.v_max, self.constraint.delta_max])
        # ocp.constraints.idxbx = np.array([3,4])

        # ocp.constraints.lbu = np.array([self.constraint.a_min, self.constraint.omega_min])
        # ocp.constraints.ubu = np.array([self.constraint.a_max, self.constraint.omega_max])
        # ocp.constraints.idxbu = np.array([0, 1])

        # # initial state
        # ocp.constraints.x0 = np.zeros(nx)
        # ocp.cost.yref = np.zeros(nx+nu)
        # ocp.cost.yref_e = np.zeros(nx)

        # # solver options
        # ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # # explicit Runge-Kutta integrator
        # ocp.solver_options.integrator_type = 'ERK'
        # ocp.solver_options.print_level = 0
        # ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        # # compile acados ocp
        # json_file = os.path.join('./'+self.model.name+'_acados_ocp.json')
        # self.solver = AcadosOcpSolver(ocp, json_file=json_file)