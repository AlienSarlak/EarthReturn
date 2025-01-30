import casadi as ca
from state_vector import State_Vector
from math import radians
from utils import state_space_to_mpc_vector


class MPCController:
    def __init__(self, gravity=9.81, mass=20, rocket_height=10, T_max=-600, N=5):
        self.gravity = gravity
        self.mass = mass
        self.T_max = T_max
        # N => Number of iterations
        self.N = N
        self.opti = ca.Opti()
        self.rocket_height = rocket_height
        self.I = (1 / 12) * mass * (rocket_height**2)

    def dot_s(self, current_state: ca.DM, u: ca.MX, dt: float):
        # System Dynamic

        m = self.mass
        r = self.rocket_height * 0.5

        alpha = current_state[2]
        F_t = u[0]
        theta = u[1]

        ###################################
        ddot_x = -(1 / m) * F_t * ca.sin(alpha + theta)
        ddot_y = +(1 / m) * F_t * ca.cos(alpha - theta) - self.gravity
        ddot_alpha = (r/self.I) * F_t * ca.sin(theta)
        ###################################

        dot_x = current_state[3] + ddot_x * dt
        dot_y =  current_state[4] + ddot_y * dt
        dot_alpha =  current_state[5] + ddot_alpha * dt

        X_dot =  ca.vertcat(
            dot_x,
            dot_y,
            dot_alpha,
            ddot_x,
            ddot_y,
            ddot_alpha,
        )

        return X_dot

    def new_state(self,current_state: ca.DM, u: ca.MX, dt: float):
        # In this version Euler
        new_state = current_state + self.dot_s(current_state, u, dt) * dt
        # RK4
        # k1 = self.dot_s(current_state, u, dt)
        # k2 = self.dot_s(current_state + 0.5 * dt * k1, u, dt)
        # k3 = self.dot_s(current_state + 0.5 * dt * k2, u, dt)
        # k4 = self.dot_s(current_state + dt * k3, u, dt)
        # new_state = current_state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return new_state

    ########################################################
    #                   Setup MPC
    ########################################################
    def setup_mpc(
        self, current_state: State_Vector, target_state: State_Vector, dt: float
    ):

        # X is current state
        X = state_space_to_mpc_vector(current_state)
        # Z is target state
        Z = state_space_to_mpc_vector(target_state)

        residual = X - Z

        alpha_penalty = self.opti.parameter()
        dot_y_penalty = self.opti.parameter()
        dot_x_penalty = self.opti.parameter()
        q = self.opti.parameter()

        # y
        # self.opti.set_value(criteria_dist_to_ground, distance_to_ground)
        self.opti.set_value(q, 1e3)
        self.opti.set_value(alpha_penalty, 1e6)
        self.opti.set_value(
            dot_y_penalty, 4e3 / (abs(residual[1] / target_state.y) + 1)
        )
        self.opti.set_value(dot_x_penalty, 4e3)

        # Weight to penalize the differences in target and current state
        # q = (1e6 / criteria_dist_to_ground + 1)

        Q = ca.MX(6, 6)
        Q[0, 0] = dot_x_penalty
        Q[1, 1] = q
        Q[2, 2] = alpha_penalty
        Q[3, 3] = q
        Q[4, 4] = dot_y_penalty
        Q[5, 5] = alpha_penalty

        # Penalize the distance to target [6x6]
        # Q = ca.MX([[q, 0, 0, 0, 0, 0],
        #            [0, q, 0, 0, 0, 0],
        #            [0, 0, q, 0, 0, 0],
        #            [0, 0, 0, q, 0, 0],
        #            [0, 0, 0, 0, q, 0],
        #            [0, 0, 0, 0, 0, q],
        #            ])

        # Penalize the controls [2x2]
        R = ca.DM(
            [
                [0.85, 0],
                [0, 1e4],
            ]
        )

        #########################################

        self.opti.set_value(q, 1e3)
        self.opti.set_value(alpha_penalty, 4e3)
        self.opti.set_value(
            dot_y_penalty, 4e3 / (abs(residual[1] / target_state.y) + 1)
        )
        self.opti.set_value(dot_x_penalty, 8e3)

        Q_F = ca.MX(6, 6)
        Q[0, 0] = dot_x_penalty
        Q[1, 1] = q
        Q[2, 2] = q
        Q[3, 3] = q
        Q[4, 4] = dot_y_penalty
        Q[5, 5] = alpha_penalty

        R_F = ca.DM(
            [
                [5, 0],
                [0, 1e5],
            ]
        )

        #########################################

        # N horizon, each [F_T, theta]
        U = self.opti.variable(self.N, 2)
        cost = 0

        for i in range(self.N - 1):
            u_i = U[i, :]
            cost_i = (X - Z).T @ Q @ (X - Z) + u_i @ R @ u_i.T

            X = self.new_state(current_state=X, u=u_i.T, dt=dt)
            cost = cost + cost_i

        # Putting more weight on the last step
        cost = cost + (X - Z).T @ Q_F @ (X - Z) + u_i @ R_F @ u_i.T

        self.opti.minimize(cost)

        ###########################################
        # Set the constraints on the control inputs
        ###########################################

        # 0 <= T <= T_max
        self.opti.subject_to(self.opti.bounded(self.T_max, U[:, 0], 0))
        # -0.2 Radians < theta < 0.2 Radians
        t_limit = radians(60)
        self.opti.subject_to(self.opti.bounded(-t_limit, U[:, 1], t_limit))
        return U

    def solve(self, U):
        p_opts = {
            "print_time": False,  # Disable printing of timing information
            "ipopt": {
                "print_level": 0,  # Set print level to 0 (no output)
                "sb": "yes",  # Suppress IPOPT banner
                "file_print_level": 0,  # No output in log files
            },
            "expand": True,
        }
        s_opts = {"max_iter": 50, "print_level": 0, "sb": "yes"}
        self.opti.solver("ipopt", p_opts, s_opts)

        solution = self.opti.solve()
        U_opt = solution.value(U)
        return U_opt
