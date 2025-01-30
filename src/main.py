from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize
from mpc_controller import MPCController
from math import radians, degrees
import matplotlib.pyplot as plt
import time


def main():
    print("Start ...")

    # print(plt.style.available)

    fps = 60
    mass = 30  # KG

    visualizer = Visualize(width=800, height=1000, fps=fps)
    rocket_y = 100  # in pixels
    # initial SS vector
    initial_state = State_Vector(x=200, y=rocket_y, y_dot=0, alpha=radians(-90))
    print(initial_state)

    rocket = Rocket(
        state_vector=initial_state,
        mass=mass,
        position=(initial_state.x, initial_state.y),
    )
    ps = Physics_Simulator(ground_height=visualizer.display_height, rocket=rocket)

    mpc = MPCController(mass=mass)

    visualizer.add_object(ps)

    print(
        f"ps.groud_level = {ps.groud_level} and ps.groud_tickness = {ps.groud_tickness}"
    )

    # Note that rocket position is in the centre of it that is why we have
    # rocket.size.height * 0.5
    ground = ps.groud_level - ps.groud_tickness
    x_target = visualizer.display_width * 0.5
    y_target = ground - rocket.size.height * 0.5

    target_vector = State_Vector(
        x=x_target, y=y_target, alpha=0, x_dot=0, y_dot=0, alpha_dot=0
    )

    running = True

    thrust = 0
    nozzle_angle = 0

    print(f"initial => {rocket.state_vector}")

    alpha_list = []
    nozzle_list = []
    doty_list = []
    x_list = []
    dotalpha_list = []
    thrust_list = []
    time_stamp = []

    start_time = time.time()

    while running:
        visualizer.handle_events()

        # force should be a tuple
        ps.rocket.apply_force(force=thrust, nozzle_angle=nozzle_angle)

        ps.update_rocket_state(dt=1 / fps)

        # optimization
        U = mpc.setup_mpc(
            current_state=ps.rocket.state_vector, target_state=target_vector, dt=1 / fps
        )

        # u_opt is the optimal control
        u_opt = mpc.solve(U)

        thrust = u_opt[0, 0]
        nozzle_angle = u_opt[0, 1]

        # thrust = -200
        # nozzle_angle = radians(0.0)

        print(
            f"Thrust => {'{:.2f}'.format(thrust)}, \
              Nozzle Angle => {'{:.2f}'.format(degrees(nozzle_angle))}"
        )

        print(f"vector: {ps.rocket.state_vector}")
        print(
            f"Distance to the ground: {'{:.2f}'.format(y_target - ps.rocket.state_vector.y)}, \
          Velocity: {'{:.2f}'.format(ps.rocket.state_vector.y_dot)}"
        )

        print("\n")

        current_time = time.time()
        time_stamp.append(current_time - start_time)
        alpha_list.append(degrees(ps.rocket.state_vector.alpha))
        nozzle_list.append(degrees(nozzle_angle))
        doty_list.append(ps.rocket.state_vector.y_dot)
        x_list.append(ps.rocket.state_vector.x)
        dotalpha_list.append(degrees(ps.rocket.state_vector.alpha_dot))
        thrust_list.append(abs(thrust))

        visualizer.update()
        # running = False
        # time.sleep(0.5)
        running = True if (current_time - start_time) < 15 else False
        # running = False if (y_target - ps.rocket.state_vector.y) < 2 else True

    print("End ...")

    # DRAW PLOTS

    # Titles for each plot
    titles = [
        "Alpha (degree)",
        "DotAlpha (degree/sec)",
        "DotY (m/s)",
        "X (m)",
        "Nozzle (degree)",
        "Thrust (N)",
    ]

    # Data lists to plot
    data_lists = [
        alpha_list,
        dotalpha_list,
        doty_list,
        x_list,
        nozzle_list,
        thrust_list,
    ]

    plt.style.use("seaborn-v0_8-deep")

    # Create subplots
    fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 8), constrained_layout=True)

    # Flatten axes for easy iteration (since it's a grid)
    axes = axes.flatten()

    for _, (ax, data, title) in enumerate(zip(axes, data_lists, titles)):
        # ax.set_facecolor("lightgrey")
        ax.plot(time_stamp, data, label=title, linewidth=2, color="blue")

        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Value", fontsize=10)

        ax.grid(which="both")
        ax.grid(which="minor", alpha=0.2)
        ax.grid(which="major", alpha=0.5)

        ax.legend()

    # Remove the last empty subplot
    if len(data_lists) < len(axes):
        fig.delaxes(axes[-1])

    # Display the plot
    plt.suptitle(
        "Smooth Line Charts of Variables vs Time", fontsize=16, fontweight="bold"
    )
    plt.show()


if __name__ == "__main__":
    main()
