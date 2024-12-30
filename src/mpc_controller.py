import casadi as ca
from math import radians, degrees
from numpy import matrix


class MPCController:
    def __init__(self, gravity=9.81,
                 mass=20,
                 T_max=-600,
                 N=10):
        self.gravity = gravity
        self.mass = mass
        # T_min is 0
        self.T_max = T_max
        # N => Number of iterations
        self.N = N
        self.opti = ca.Opti()

    def evolution(self, current_state, u, dt):
        A = ca.DM([[0, 1],
                   [0, 0],])

        B = ca.DM([[0],
                   [1/self.mass],])

        f = ca.DM([[0],
                   [-self.gravity],])

        X_dot = A @ current_state + B @ u + f
        X_new = current_state + X_dot * dt
        return X_new

    def setup_mpc(self, real_state, target_vector, dt):

        X = real_state
        # we have N x u and for this case u is just a thrust value
        U = self.opti.variable(self.N, 1)

        result_vector = target_vector - X
        criteria_dist_to_ground = self.opti.parameter()
        criteria_falling_velocity = self.opti.parameter()

        self.opti.set_value(criteria_dist_to_ground, abs(result_vector[0]))
        self.opti.set_value(criteria_falling_velocity, abs(result_vector[1]))

        # Weight to penalize the differences in target and current state
        q = (4000/criteria_dist_to_ground+1e-6)*(criteria_falling_velocity)

        Q = ca.MX(2, 2)
        Q[0, 0] = 1
        Q[0, 1] = 0
        Q[1, 0] = 0
        Q[1, 1] = q

        # Weight to penalize abrupt controls
        R = ca.DM([[5],])

        # q = (20000/criteria_dist_to_ground+1e-6)*(criteria_falling_velocity*10)

        # # Weight to penalize the last state
        # P = ca.MX(2, 2)
        # P[0, 0] = 1
        # P[0, 1] = 0
        # P[1, 0] = 0
        # P[1, 1] = q

        cost = 0
        for i in range(self.N-1):
            u_i = U[i, 0]

            current_cost = (X - target_vector).T @ Q @ (X - target_vector) + \
                u_i.T @ R @ u_i
            X = self.evolution(current_state=X, u=u_i.T, dt=dt)
            cost = cost + current_cost

        # Putting more weight on the last step
        cost = cost + (X - target_vector).T @ Q @ (X - target_vector)

        self.opti.minimize(cost)

        # Set the constraints on the control inputs
        # T = U[:, 0]

        # 0 <= T <= T_max
        self.opti.subject_to(self.opti.bounded(self.T_max, U, 0))
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
