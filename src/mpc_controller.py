import casadi as ca
from state_vector import State_Vector
from utils import state_space_to_mpc_vector


class MPCController:
    def __init__(self, gravity=9.81, mass=20, rocket_height=10, T_max=-600, N=5):
        self.gravity = gravity
        self.mass = mass
        self.T_max = T_max
        # N => Number of iterations
        self.N = N
        self.opti = ca.Opti()
        self.nominal = State_Vector()
        self.F_T_nominal = 0.0
        self.rocket_height = rocket_height
        self.I = (1 / 12) * mass * (rocket_height**2)

    def evolution(self, current_state: ca.DM, u: ca.MX, dt: float):
        m = self.mass
        r = self.rocket_height * 0.5
        p_alpha = self.nominal.alpha

        A = ca.MX(6, 6)
        A[0, 3] = 1
        A[1, 4] = 1
        A[2, 5] = 1
        A[3, 2] = -(1 / m) * self.F_T_nominal
        A[4, 2] = +(1 / m) * self.F_T_nominal * (p_alpha)
        # A = ca.DM([[0, 0, 0, 1, 0, 0],
        #            [0, 0, 0, 0, 1, 0],
        #            [0, 0, 0, 0, 0, 1],
        #            [0, 0, -(1/m)*self.F_T_nominal * cos(p_alpha), 0, 0, 0],
        #            [0, 0, (1/m)*self.F_T_nominal * sin(p_alpha), 0, 0, 0],
        #            [0, 0, 0, 0, 0, 0],
        #            ])

        B = ca.MX(6, 2)
        B[3, 0] = (1 / m) * (p_alpha)
        B[3, 1] = -(1 / m) * self.F_T_nominal
        B[4, 0] = (1 / m)
        B[4, 1] = +(1 / m) * self.F_T_nominal * (p_alpha)
        B[5, 1] = +(r / self.I) * self.F_T_nominal

        # B = ca.DM([[0, 0],
        #            [0, 0],
        #            [0, 0],
        #            [(1/m)*sin(p_alpha), (1/m) * self.F_T_nominal*cos(p_alpha)],
        #            [(1/m)*cos(p_alpha), -(1/m) * self.F_T_nominal*sin(p_alpha)],
        #            [0, (r/self.I)*self.F_T_nominal],
        #            ])

        # make alpha to delta alpha
        delta_alpha = current_state[2] - (p_alpha)

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
    def setup_mpc(
        self, current_state: State_Vector, target_state: State_Vector, dt: float
    ):
        self.nominal = current_state

        X = state_space_to_mpc_vector(current_state)
        Z = state_space_to_mpc_vector(target_state)

        residual = X - Z


        alpha_penalty = self.opti.parameter()
        dot_y_penalty = self.opti.parameter()
        dot_x_penalty = self.opti.parameter()
        q = self.opti.parameter()

        # y
        # self.opti.set_value(criteria_dist_to_ground, distance_to_ground)
        self.opti.set_value(q, 1e3)
        self.opti.set_value(alpha_penalty, 6e3)
        self.opti.set_value(dot_y_penalty, 4e3 / (abs(residual[1]/target_state.y) + 1))
        self.opti.set_value(dot_x_penalty, 4e3)

        # Weight to penalize the differences in target and current state
        # q = (1e6 / criteria_dist_to_ground + 1)



        Q = ca.MX(6, 6)
        Q[0, 0] = dot_x_penalty
        Q[1, 1] = q
        Q[2, 2] = q
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
        self.opti.set_value(dot_y_penalty, 4e3 / (abs(residual[1]/target_state.y) + 1))
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

            X = self.evolution(current_state=X, u=u_i.T, dt=dt)
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
        self.opti.subject_to(self.opti.bounded(-1, U[:, 1],1))
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
        self.F_T_nominal = float(U_opt[0, 0])
        return U_opt
