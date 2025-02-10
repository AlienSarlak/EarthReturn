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

    fps = 50
    mass = 30

    visualizer = Visualize(width=800, height=1000, fps=fps)
    rocket_y = 200  # in pixels
    # initial SS vector
    #***********************************
    #   Do not set alpha > ±50
    #    abs(alpha) must be <=50
    #  300 <  x  < 500
    #***********************************
    initial_state = State_Vector(x=250, y=rocket_y, alpha=radians(-70))
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

        print(
            f"Thrust => {'{:.2f}'.format(-thrust)}, \
              Nozzle Angle => {'{:.2f}'.format(degrees(nozzle_angle))}"
        )

        print(f"vector: {ps.rocket.state_vector}")
        print(
            f"Distance to the ground: {y_target - ps.rocket.state_vector.y:.2f}, "
            f"Velocity: {ps.rocket.state_vector.y_dot:.2f}, "
            f"Time: {(current_time - start_time):.2f}"
        )

        print("\n")

        if(current_time - start_time > 210):
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
        if(current_time - start_time>245):
            break

        visualizer.update()
        # running = False
        # time.sleep(0.5)
        # running = True if (current_time - start_time) < 35 else False

        # running = False if (y_target - ps.rocket.state_vector.y) < 2 else True

    print("End ...")

    # Titles for each plot
    titles = [
        r"$\alpha$ (deg)",
        r"$\dot{\alpha}$ (deg/sec)",
        r"$\dot{Y}$ (m/s)",
        r"$X$ (m)",
        r"Nozzle (deg)",
        r"Thrust (N)",
    ]



    data_lists = [
        (alpha_list, p_alpha_list),
        (dotalpha_list, p_dotalpha_list),
        (doty_list, p_doty_list),
        (x_list, p_x_list),
        (nozzle_list, None),  # No corresponding p_ data
        (thrust_list, None),  # No corresponding p_ data
    ]

    plt.style.use("seaborn-v0_8-deep")

    colors = ["blue", "red"]

    font = {'family' : 'Sans serif',
            'weight' : 'bold',
            'size'   : 14}

    plt.rc('font', **font)

    for i, ((data, p_data), title) in enumerate(zip(data_lists, titles)):

        fig, ax = plt.subplots(figsize=(10, 8))

        ax.plot(time_stamp, data, label=f"{title} (Original)", linewidth=2, color=colors[0])

        if p_data is not None:
            ax.plot(time_stamp, p_data, label=f"{title} (Predicted)", linewidth=2, color=colors[1], linestyle="dashed")

        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")


        ax.grid(which="both")
        ax.grid(which="minor", alpha=0.2)
        ax.grid(which="major", alpha=0.5)

        ax.legend(fontsize=18)

        filename = f"{i}.png"
        plt.savefig(filename, dpi=600, bbox_inches="tight")

        plt.close(fig)


if __name__ == "__main__":
    main()
