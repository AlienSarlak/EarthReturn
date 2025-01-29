import casadi as ca
from math import radians, degrees
import matplotlib.pyplot as plt
import numpy as np


class OCPController:
    def __init__(
        self,
        gravity=9.81,
        mass=20,
        rocket_height=10,
    ):
        self.gravity = gravity
        self.mass = mass

        # OPTI Environment
        self.opti = ca.Opti()
        self.r = rocket_height * 0.5
        self.I = (1 / 12) * mass * (rocket_height**2)

    def system_dynamic(self, x, u, t):
        # x -> 6x1 [x,y,alpha,dot_x,dot_y,dot_alpha]
        # u -> 2x1 [F_t, theta]
        m = self.mass
        T = u[0]
        theta = u[1]

        ddot_x = (T / m) * ca.sin(x[2] + theta)
        ddot_y = (T / m) * ca.cos(x[2] + theta) - self.gravity
        ddot_alpha = (self.r / self.I) * T * ca.sin(theta)

        dot_x = x[3] + ddot_x * t
        dot_y = x[4] + ddot_y * t
        dot_alpha = x[5] + ddot_alpha * t

        dxdt = ca.vertcat(
            dot_x,
            dot_y,
            dot_alpha,
            ddot_x,
            ddot_y,
            ddot_alpha,
        )
        return dxdt

    def solve(self, delta_t, duration):
        N = int(duration / delta_t)
        F_t = self.opti.variable(N)
        theta = self.opti.variable(N)

        # State vector [x, y, alpha, dot_x, dot_y, dot_alpha]
        x = self.opti.variable(6, N)

        # Cost/objectvie function
        J = 0

        # Initial state
        x_initial = [200, 500, radians(10), 0, 10, 0]
        self.opti.subject_to(x[:, 0] == x_initial)

        # Target state
        x_target = [100, 0, 0, 0, 0, 0]
        self.opti.subject_to(x[:, -1] == x_target)

        # -----------------------------------------------
        for k in range(N - 1):
            u_k = ca.vertcat(F_t[k], theta[k])
            # x_k -> 6x1
            x_k = x[:, k]

            x_dot_k = self.system_dynamic(x_k, u_k, delta_t)
            # TODO: RK4
            # Euler integration for x_{k+1}
            x_k_next = x_k + delta_t * x_dot_k

            self.opti.subject_to(x[:, k + 1] == x_k_next)

            penalty = 0  # (1e6 * (x_k[4] - x_k_next[4]) ** 2)

            J += (1) + penalty

        # -----------------------------------------------

        self.opti.minimize(J)

        # Control constraints F, theta
        self.opti.subject_to(0 <= F_t)
        # 600 N
        self.opti.subject_to(F_t <= 600)
        # Around ±17.1 degree
        self.opti.subject_to(-0.3 <= theta)
        self.opti.subject_to(theta <= 0.3)

        p_opts = {
            "print_time": False,
            "ipopt": {
                "print_level": 0,
                "sb": "yes",  # Suppress banner
                "file_print_level": 0,  # No output in log files
            },
            "expand": True,
        }
        # max iteration 50 (XXX: play with it later due to Maximum_Iterations_Exceeded)
        s_opts = {"max_iter": 150, "print_level": 0, "sb": "yes"}
        self.opti.solver("ipopt", p_opts, s_opts)

        # Solve the optimization problem
        sol = self.opti.solve()

        # Solution extraxtion
        Thrusts = sol.value(F_t)
        thetas = sol.value(theta)
        states = sol.value(x)

        # FIXME: return the .....
        return states, Thrusts, thetas


########################################################
#                         Main
########################################################


def main():
    print("OCP SIMULATION...")
    ocp = OCPController()
    duration = 25
    time_step = 0.01
    trajectory, thrust, theta = ocp.solve(delta_t=time_step, duration=duration)

    fig, axes = plt.subplots(4, 1, figsize=(8, 10))

    # THrust
    idx=0
    axes[idx].plot(np.arange(0, duration, time_step), thrust, color="b", label="Thrust")
    axes[idx].set_xlabel("Time (s)")
    axes[idx].set_ylabel("Thrust")
    axes[idx].set_title("Thrust Over Time")
    axes[idx].grid(True, linestyle="--", alpha=0.7)
    axes[idx].legend()

    # Theta
    idx=1
    axes[idx].plot(np.arange(0, duration, time_step), theta, color="m", label="Theta")
    axes[idx].set_xlabel("Time (s)")
    axes[idx].set_ylabel("Theta")
    axes[idx].set_title("Nozzle Angle Over Time")
    axes[idx].grid(True, linestyle="--", alpha=0.7)
    axes[idx].legend()

    # alpha
    idx=2
    axes[idx].plot(
        np.arange(0, duration * 100, 1),
        trajectory[2, :],
        color="g",
        label="Alpha Component",
    )
    axes[idx].set_xlabel("Time (scaled)")
    axes[idx].set_ylabel("alpha")
    axes[idx].set_title("alpha Over Time")
    axes[idx].grid(True, linestyle="--", alpha=0.7)
    axes[idx].legend()

    # Trajectory
    idx=3
    axes[idx].plot(
        trajectory[0, :],
        trajectory[1, :],
        marker="o",
        linestyle="-",
        color="r",
        label="2D Trajectory",
    )
    axes[idx].set_xlabel("X Position")
    axes[idx].set_ylabel("Y Position")
    axes[idx].set_title("2D Trajectory Plot")
    axes[idx].grid(True, linestyle="--", alpha=0.7)
    axes[idx].legend()

    # plt.figure(figsize=(16, 12))
    # plt.plot(
    #     np.arange(0, duration, time_step),
    #     thrust,
    #     color="b",
    #     label="thrust",
    # )

    # plt.plot(
    #     np.arange(0, duration *100, 1),
    #     trajectory[5, :],
    #     color="b",
    #     label="trajectory",
    # )

    # plt.plot(
    #     trajectory[0, :],
    #     trajectory[1, :],
    #     marker='o', linestyle='-',
    #     color="b",
    #     label="trajectory",
    # )
    plt.tight_layout()
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.show()


if __name__ == "__main__":
    main()
