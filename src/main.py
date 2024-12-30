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
                        [state_vector.y_dot]])
    return real_state


def main():
    print("Start ...")

    fps = 60
    mass = 30

    visualizer = Visualize(width=800, height=1000, fps=fps)
    # rocket_y will be a random position
    rocket_y = 0  # in pixels
    state = State_Vector(y=rocket_y, y_dot=0)  # initial ss vector
    print(state)
    rocket = Rocket(state_vector=state, mass=mass, position=(300, rocket_y))
    ps = Physics_Simulator(
        ground_height=visualizer.display_height,
        rocket=rocket)

    mpc = MPCController(mass=mass)

    visualizer.add_object(ps)

    # It means y=ground, {y_dot, alpha, alpha_dot}=0
    print(f'ps.groud_level = {
          ps.groud_level} - ps.groud_tickness = {ps.groud_tickness}')

    # Note that rocket position is in the centre of it that is why we have
    # rocket.size.height * 0.5
    ground = ps.groud_level - ps.groud_tickness
    y_target = ground - rocket.size.height * 0.5
    target_vector = ca_dm([[y_target],
                           [0]])

    running = True

    thrust = 0
    nozzle_angle = 0

    print(f'initial => {rocket.state_vector}')

    while running:
        visualizer.handle_events()

        # force should be a tuple
        ps.rocket.apply_force(force=(0, thrust))
        ps.update_rocket_state(dt=1/fps)

        # optimization
        U = mpc.setup_mpc(real_state=mpc_vector(ps.rocket.state_vector),
                          target_vector=target_vector, dt=1/fps)

        # u_opt is the optimal control
        u_opt = mpc.solve(U)

        thrust = u_opt[0]  # first u in the list of u_opt
        # nozzle_angle = u_opt[0, 1]  # first angle in the list of u_opt

        print(f"Thrust => {"{:.2f}".format(thrust)}, \
              Nozzle Angle => {"{:.2f}".format(degrees(nozzle_angle))}")

        print(f"Distance to the ground: {"{:.2f}".format(y_target - ps.rocket.state_vector.y)}, \
              Velocity: {"{:.2f}".format(ps.rocket.state_vector.y_dot)}")

        print("\n")

        visualizer.update()
        # running = False
        # time.sleep(0.2)

    print("End ...")


if __name__ == "__main__":
    main()
