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

    fps = 50
    mass = 30  # KG

    visualizer = Visualize(width=800, height=1000, fps=fps)
    rocket_y = 100  # in pixels
    # initial SS vector
    #
    #   Do not set alpha > ±50
    #    abs(alpha) must be <=50
    #  300 <  x  < 500
    #
    initial_state = State_Vector(x=100, y=rocket_y, alpha=radians(-60))
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
    doty_list = []
    x_list = []
    dotalpha_list = []

    p_alpha_list = []
    p_doty_list = []
    p_x_list = []
    p_dotalpha_list = []

    nozzle_list = []
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
        u_opt,predicted_state  = mpc.solve(U)

        thrust = u_opt[0, 0]
        nozzle_angle = u_opt[0, 1]

        # thrust = -300
        # nozzle_angle = radians(-1)
        current_time = time.time()

        # print(
        #     f"Thrust => {'{:.2f}'.format(-thrust)}, \
        #       Nozzle Angle => {'{:.2f}'.format(degrees(nozzle_angle))}"
        # )

        # print(f"vector: {ps.rocket.state_vector}")
        # print(
        #     f"Distance to the ground: {y_target - ps.rocket.state_vector.y:.2f}, "
        #     f"Velocity: {ps.rocket.state_vector.y_dot:.2f}, "
        #     f"Time: {(current_time - start_time):.2f}"
        # )

        print("\n")

        # if(current_time - start_time > 200):
        time_stamp.append(current_time - start_time)
        alpha_list.append(degrees(ps.rocket.state_vector.alpha))
        nozzle_list.append(degrees(nozzle_angle))
        doty_list.append(ps.rocket.state_vector.y_dot)
        x_list.append(ps.rocket.state_vector.x)
        dotalpha_list.append(degrees(ps.rocket.state_vector.alpha_dot))
        thrust_list.append(abs(thrust))


        p_alpha_list.append(degrees(predicted_state[2]))
        p_doty_list.append(predicted_state[4])
        p_x_list.append(predicted_state[0])
        p_dotalpha_list.append(degrees(predicted_state[5]))
        # if(current_time - start_time>235):
        #     break

        visualizer.update()
        # running = False
        # time.sleep(0.5)
        running = True if (current_time - start_time) < 35 else False

        # running = False if (y_target - ps.rocket.state_vector.y) < 2 else True

    print("End ...")

    # Titles for each plot
    titles = [
        "Alpha (degree)",
        "DotAlpha (degree/sec)",
        "DotY (m/s)",
        "X (m)",
        "Nozzle (degree)",
        "Thrust (N)",
    ]

    # Data lists to plot (each tuple contains original and p_ version)
    data_lists = [
        (alpha_list, p_alpha_list),
        (dotalpha_list, p_dotalpha_list),
        (doty_list, p_doty_list),
        (x_list, p_x_list),
        (nozzle_list, None),  # No corresponding p_ data
        (thrust_list, None),  # No corresponding p_ data
    ]

    plt.style.use("seaborn-v0_8-deep")

    # Create subplots
    # fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 8), constrained_layout=True)

    # Flatten axes for easy iteration (since it's a grid)
    # axes = axes.flatten()

    # Define colors for the two sets of data
    colors = ["blue", "red"]  # Original data in blue, p_ data in red

    for i, ((data, p_data), title) in enumerate(zip(data_lists, titles)):

        fig, ax = plt.subplots(figsize=(10, 8))

        ax.plot(time_stamp, data, label=f"{title} (Original)", linewidth=2, color=colors[0])

        if p_data is not None:
            ax.plot(time_stamp, p_data, label=f"{title} (Predicted)", linewidth=2, color=colors[1], linestyle="dashed")

        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Value", fontsize=10)

        ax.grid(which="both")
        ax.grid(which="minor", alpha=0.2)
        ax.grid(which="major", alpha=0.5)

        ax.legend()

        filename = f"{i}.png"
        plt.savefig(filename, dpi=600, bbox_inches="tight")

        plt.close(fig)

    # Display the plot
    # plt.suptitle(
    #     "Smooth Line Charts of Variables vs Time", fontsize=16, fontweight="bold"
    # )
    # plt.show()


    # # DRAW PLOTS

    # # Titles for each plot
    # titles = [
    #     "Alpha (degree)",
    #     "DotAlpha (degree/sec)",
    #     "DotY (m/s)",
    #     "X (m)",
    #     "Nozzle (degree)",
    #     "Thrust (N)",
    # ]

    # # Data lists to plot
    # data_lists = [
    #     alpha_list,
    #     dotalpha_list,
    #     doty_list,
    #     x_list,
    #     nozzle_list,
    #     thrust_list,
    # ]

    # plt.style.use("seaborn-v0_8-deep")

    # # Create subplots
    # fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 8), constrained_layout=True)

    # # Flatten axes for easy iteration (since it's a grid)
    # axes = axes.flatten()

    # for i, (ax, data, title) in enumerate(zip(axes, data_lists, titles)):
    #     # ax.set_facecolor("lightgrey")
    #     ax.plot(time_stamp, data, label=title, linewidth=2, color="blue")

    #     ax.set_title(title, fontsize=12, fontweight="bold")
    #     ax.set_xlabel("Time (s)", fontsize=10)
    #     ax.set_ylabel("Value", fontsize=10)

    #     ax.grid(which="both")
    #     ax.grid(which="minor", alpha=0.2)
    #     ax.grid(which="major", alpha=0.5)

    #     ax.legend()

    # # Remove the last empty subplot
    # if len(data_lists) < len(axes):
    #     fig.delaxes(axes[-1])

    # # Display the plot
    # plt.suptitle(
    #     "Smooth Line Charts of Variables vs Time", fontsize=16, fontweight="bold"
    # )
    # plt.show()


if __name__ == "__main__":
    main()


    # for i, (data, title) in enumerate(zip(data_lists, titles)):
    #     fig, ax = plt.subplots(figsize=(10, 4))  # Create a new figure for each plot

    #     ax.plot(time_stamp, data, label=title, linewidth=2, color="blue")
    #     ax.set_title(title, fontsize=12, fontweight="bold")
    #     ax.set_xlabel("Time (s)", fontsize=10)
    #     ax.set_ylabel("Value", fontsize=10)

    #     ax.grid(which="both")
    #     ax.grid(which="minor", alpha=0.2)
    #     ax.grid(which="major", alpha=0.5)

    #     ax.legend()

    #     # Save each figure separately
    #     filename = f"{i}.png"
    #     plt.savefig(filename, dpi=300, bbox_inches="tight")

    #     plt.close(fig)


    # # Create subplots
    # fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 8), constrained_layout=True)

    # # Flatten axes for easy iteration (since it's a grid)
    # axes = axes.flatten()

    # for i, (ax, data, title) in enumerate(zip(axes, data_lists, titles)):
    #     # ax.set_facecolor("lightgrey")
    #     ax.plot(time_stamp, data, label=title, linewidth=2, color="blue")

    #     ax.set_title(title, fontsize=12, fontweight="bold")
    #     ax.set_xlabel("Time (s)", fontsize=10)
    #     ax.set_ylabel("Value", fontsize=10)

    #     ax.grid(which="both")
    #     ax.grid(which="minor", alpha=0.2)
    #     ax.grid(which="major", alpha=0.5)

    #     ax.legend()

    # # Remove the last empty subplot
    # if len(data_lists) < len(axes):
    #     fig.delaxes(axes[-1])

    # # Display the plot
    # plt.suptitle(
    #     "Smooth Line Charts of Variables vs Time", fontsize=16, fontweight="bold"
    # )
    # plt.show()
