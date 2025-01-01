import casadi as ca
from math import radians, degrees
from numpy import matrix
from state_vector import State_Vector
from math import sin, cos
from collections import namedtuple
from utils import state_space_to_mpc_vector, mpc_vector_to_state_space


class MPCController:
    def __init__(self, gravity=9.81,
                 mass=20,
                 rocket_height=10,
                 T_max=-600,
                 N=10):
        self.gravity = gravity
        self.mass = mass
        self.T_max = T_max
        # N => Number of iterations
        self.N = N
        self.opti = ca.Opti()
        self.nominal = State_Vector()
        self.rocket_height = rocket_height
        self.I = (1/12)*mass*(rocket_height**2)

    def evolution(self, current_state: ca.DM, u: ca.MX, dt: float):
        m = self.mass
        r = self.rocket_height * 0.5
        control = namedtuple('control', ['F_T', 'theta'])
        ctrl = control(F_T=u[0], theta=u[1])

        A = ca.MX(6, 6)
        A[0, 3] = 1
        A[1, 4] = 1
        A[2, 5] = 1
        A[3, 2] = (1/m)*ctrl.F_T * cos(self.nominal.alpha)
        A[3, 2] = -(1/m)*ctrl.F_T * sin(self.nominal.alpha)
        # A = ca.DM([[0, 0, 0, 1, 0, 0],
        #            [0, 0, 0, 0, 1, 0],
        #            [0, 0, 0, 0, 0, 1],
        #            [0, 0, (1/m)*ctrl.F_T * cos(self.nominal.alpha), 0, 0, 0],
        #            [0, 0, -(1/m)*ctrl.F_T * sin(self.nominal.alpha), 0, 0, 0],
        #            [0, 0, 0, 0, 0, 0],
        #            ])

        B = ca.MX(6, 2)
        B[3, 0] = (1/m)*sin(self.nominal.alpha)
        B[3, 1] = (1/m) * ctrl.F_T*cos(self.nominal.alpha)
        B[4, 0] = (1/m)*cos(self.nominal.alpha)
        B[4, 1] = -(1/m) * ctrl.F_T*sin(self.nominal.alpha)
        B[5, 1] = (r/self.I)*ctrl.F_T

        # B = ca.DM([[0, 0],
        #            [0, 0],
        #            [0, 0],
        #            [(1/m)*sin(self.nominal.alpha), (1/m) * ctrl.F_T*cos(self.nominal.alpha)],
        #            [(1/m)*cos(self.nominal.alpha), -(1/m) * ctrl.F_T*sin(self.nominal.alpha)],
        #            [0, (r/self.I)*ctrl.F_T],
        #            ])

        # make alpha to delta alpha
        delta_alpha = current_state[2] - self.nominal.alpha

        s = ca.MX(6, 1)
        s[0, 0] = current_state[0]
        s[1, 0] = current_state[1]
        s[2, 0] = delta_alpha
        s[3, 0] = current_state[3]
        s[4, 0] = current_state[4]
        s[5, 0] = current_state[5]

        X_dot = A @ s + B @ u
        X_new = current_state + X_dot * dt
        return X_new

    ########################################################
    #                   Setup MPC
    ########################################################
    def setup_mpc(self,
                  current_state: State_Vector,
                  target_state: State_Vector,
                  dt: float):

        self.nominal = current_state

        X = state_space_to_mpc_vector(current_state)
        Z = state_space_to_mpc_vector(target_state)

        residual = Z - X

        criteria_dist_to_ground = self.opti.parameter()
        criteria_falling_velocity = self.opti.parameter()

        # FIXME: not correct
        self.opti.set_value(criteria_dist_to_ground, abs(residual[1]))
        self.opti.set_value(criteria_falling_velocity, abs(residual[4]))

        # Weight to penalize the differences in target and current state
        q = (5000/criteria_dist_to_ground+1e-6)*(criteria_falling_velocity)

        Q = ca.MX(6, 6)
        Q[1, 1] = q
        Q[2, 2] = q
        Q[4, 4] = q
        Q[5, 5] = q

        # Penalize the distance to target [6x6]
        # Q = ca.MX([[1, 0, 0, 0, 0, 0],
        #            [0, q, 0, 0, 0, 0],
        #            [0, 0, q, 0, 0, 0],
        #            [0, 0, 0, 1, 0, 0],
        #            [0, 0, 0, 0, q, 0],
        #            [0, 0, 0, 0, 0, 1],
        #            ])

        # ca.substitute(Q, q, 1000)

        # Penalize the controls [2x2]
        R = ca.DM([[5, 0],
                   [0, 5]])

        # N horizon, each [F_T, theta]
        U = self.opti.variable(self.N, 2)
        cost = 0

        for i in range(self.N-1):
            u_i = U[i, :]
            cost_i = (X - Z).T @ Q @ (X - Z) + u_i @ R @ u_i.T

            X = self.evolution(current_state=X, u=u_i.T, dt=dt)
            cost = cost + cost_i

        # Putting more weight on the last step
        cost = cost + (X - Z).T @ Q @ (X - Z)

        self.opti.minimize(cost)

        # Set the constraints on the control inputs
        # T = U[:, 0]

        # 0 <= T <= T_max
        self.opti.subject_to(self.opti.bounded(self.T_max, U[:, 0], 0))
        # -0.3 Radians < theta < 0.3 Radians
        self.opti.subject_to(self.opti.bounded(-0.3, U[:, 1], 0.3))
        return U

    def solve(self, U):
        p_opts = {
            'print_time': False,         # Disable printing of timing information
            'ipopt': {
                'print_level': 0,        # Set print level to 0 (no output)
                'sb': 'yes',             # Suppress IPOPT banner
                'file_print_level': 0    # No output in log files
            },
            "expand": True
        }
        s_opts = {"max_iter": 100, "print_level": 0, "sb": "yes"}
        self.opti.solver('ipopt', p_opts, s_opts)

        solution = self.opti.solve()
        U_opt = solution.value(U)
        return U_opt
