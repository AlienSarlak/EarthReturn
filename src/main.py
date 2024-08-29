from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize
from mpc_controller import MPCController
from casadi import DM as ca_dm
from math import radians, degrees
from state_vector import State_Vector
import time


def mpc_vector(state_vector: State_Vector):
    # state_vector = [y, alpha, y_dot, alpha_dot]
    real_state = ca_dm([[state_vector.y],
                        [state_vector.alpha],
                        [state_vector.y_dot],
                        [state_vector.alpha_dot]])
    return real_state


def main():
    print("Start ...")

    fps = 120

    visualizer = Visualize(fps=fps)
    rocket_y = 500
    state = State_Vector(y=rocket_y, y_dot=50)
    print(state)
    rocket = Rocket(state_vector=state, mass=20, position=(300, rocket_y))
    ps = Physics_Simulator(rocket=rocket)
    mpc = MPCController()

    visualizer.add_object(ps)

    target_vector = ca_dm([[ps.groud_level - ps.groud_tickness - rocket.size.height*0.5 - 20],
                           [0],
                           [0],
                           [0]])

    running = True

    thrust = 0
    nozzle_angle = 0

    while running:
        visualizer.handle_events()  # Handle events such as window close

        ps.rocket.apply_force(force=(0, thrust))
        ps.update_rocket_state(dt=1/fps)

        # optimization
        U = mpc.setup_mpc(real_state=mpc_vector(ps.rocket.state_vector),
                          target_vector=target_vector)

        u_opt = mpc.solve(U)
        thrust = u_opt[0, 0]
        nozzle_angle = u_opt[0, 1]

        print(f"Thrust => {"{:.2f}".format(thrust)}, Nozzle Angle => {
              "{:.2f}".format(degrees(nozzle_angle))}")
        print(ps.rocket.state_vector)
        print("\n")

        visualizer.update()
        # time.sleep(0.25)

    print("End ...")


if __name__ == "__main__":
    main()
