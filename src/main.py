from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize
from mpc_controller import MPCController
from math import radians, degrees
import matplotlib.pyplot as plt
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import time


def main():
    print("Start ...")

    # print(plt.style.available)

    fps = 60
    mass = 30  # KG

    visualizer = Visualize(width=800, height=1000, fps=fps)
    rocket_y = 50  # in pixels
    # initial SS vector
    initial_state = State_Vector(x=600, y=rocket_y, y_dot=0, alpha=radians(+30))
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
        distance_to_ground = y_target - ps.rocket.state_vector.y
        print(f"vector: {ps.rocket.state_vector}")
        print(
            f"Distance to the ground: {'{:.2f}'.format(distance_to_ground)}, \
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
        # running = True if (current_time - start_time) < 200 else False
        running = False if distance_to_ground <=0 else True
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

    for i, (data, title) in enumerate(zip(data_lists, titles)):
        fig, ax = plt.subplots(figsize=(6, 4))  # Adjust figure size if needed
        ax.plot(time_stamp, data, label=title, linewidth=2, color="blue")

        # Customize each plot
        ax.set_title(title, fontsize=14, fontweight="bold")
        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Value", fontsize=12)
        ax.grid(which="both", linestyle="--", alpha=0.5)
        ax.legend()

        # filename = f"plot_{i+1}.png"
        # plt.savefig(filename, dpi=300, bbox_inches="tight")

        # Close the figure to avoid overlap in memory
        # plt.close()

    # Display the plot
    plt.suptitle(
        "Smooth Line Charts of Variables vs Time", fontsize=16, fontweight="bold"
    )
    plt.show()


if __name__ == "__main__":
    main()
