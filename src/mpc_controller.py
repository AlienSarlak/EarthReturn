import casadi as ca
from math import radians, degrees


class MPCController:
    def __init__(self, gravity=9.81,
                 ltc=0.5,
                 mass=20,
                 moi=2.5,
                 T_max=100,
                 N=10):
        self.gravity = gravity
        self.ltc = ltc
        self.mass = mass
        self.moi = moi
        self.T_max = T_max
        self.N = N

        self.opti = ca.Opti()

    def evolution(self, current_state, u, dt):
        A = ca.DM([[0, 0, 1, 0],
                   [0, 0, 0, 1],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]])

        B = ca.DM([[0, 0],
                   [0, 0],
                   [1/self.mass, 0],
                   [0, self.ltc/self.moi]])
        f = ca.DM([[0],
                   [0],
                   [-self.gravity],
                   [0]])

        X_new = A @ current_state + B @ u + f
        X_dot = current_state + X_new * dt
        return X_dot

    def setup_mpc(self, real_state, target_vector, dt):
        X = real_state
        U = self.opti.variable(self.N, 2)
        l_u = 10
        # height_penalty = 100
        print(f"====================")
        print(f"X => {X}")
        print(f"********************")
        cost = 0
        for i in range(self.N):
            u_i = U[i, :]
            cost += ca.norm_2(X - target_vector) ** 2
            X = self.evolution(current_state=X, u=u_i.T, dt=dt)
            print(f"====================")
            print(f"X => {X}")
            print(f"~~~~~~~~~~~~~~~~~~~~")

        # Putting more weight on the last step
        cost += ca.norm_2(X - target_vector) ** 2

        self.opti.minimize(cost)

        # Set the constraints on the control inputs
        T = U[:, 0]
        theta = U[:, 1]

        # 0 <= T <= T_max
        self.opti.subject_to(self.opti.bounded(0, T, self.T_max))
        # -pi/4 <= theta <= pi/4
        self.opti.subject_to(self.opti.bounded(-ca.pi/4, theta, ca.pi/4))

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
        s_opts = {"max_iter": 20, "print_level": 0, "sb": "yes"}
        self.opti.solver('ipopt', p_opts, s_opts)

        solution = self.opti.solve()
        U_opt = solution.value(U)
        return U_opt
