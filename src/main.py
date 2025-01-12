from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize
from mpc_controller import MPCController
from math import radians, degrees, sin, cos
from state_vector import State_Vector
from utils import rotate_point
import time


def main():
    print("Start ...")

    fps = 60
    mass = 30  # KG

    visualizer = Visualize(width=800, height=1000, fps=fps)
    rocket_y = 100  # in pixels
    # initial SS vector
    initial_state = State_Vector(
        x=300, y=rocket_y, y_dot=0, alpha=radians(-30))
    print(initial_state)

    rocket = Rocket(state_vector=initial_state,
                    mass=mass, position=(initial_state.x, initial_state.y))
    ps = Physics_Simulator(
        ground_height=visualizer.display_height,
        rocket=rocket)

    mpc = MPCController(mass=mass)

    visualizer.add_object(ps)

    print(f'ps.groud_level = {
          ps.groud_level} and ps.groud_tickness = {ps.groud_tickness}')

    # Note that rocket position is in the centre of it that is why we have
    # rocket.size.height * 0.5
    ground = ps.groud_level - ps.groud_tickness
    x_target = visualizer.display_width * 0.5
    y_target = ground - rocket.size.height * 0.5

    target_vector = State_Vector(x=x_target,
                                 y=y_target,
                                 alpha=0,
                                 x_dot=0,
                                 y_dot=0,
                                 alpha_dot=0)

    running = True

    thrust = 0
    nozzle_angle = 0

    print(f'initial => {rocket.state_vector}')

    while running:
        visualizer.handle_events()

        # force should be a tuple
        ps.rocket.apply_force(force=thrust,
                              nozzle_angle=nozzle_angle)

        ps.update_rocket_state(dt=1/fps)

        # optimization
        U = mpc.setup_mpc(current_state=ps.rocket.state_vector,
                          target_state=target_vector, dt=1/fps)

        # u_opt is the optimal control
        u_opt = mpc.solve(U)

        thrust = u_opt[0, 0]
        nozzle_angle = u_opt[0, 1]

        # thrust = -200
        # nozzle_angle = radians(0.0)

        print(f"Thrust => {"{:.2f}".format(thrust)}, \
              Nozzle Angle => {"{:.2f}".format(degrees(nozzle_angle))}")

        print(f"vector: {ps.rocket.state_vector}")
        print(f"Distance to the ground: {"{:.2f}".format(y_target - ps.rocket.state_vector.y)}, \
          Velocity: {"{:.2f}".format(ps.rocket.state_vector.y_dot)}")

        print("\n")

        visualizer.update()
        # running = False
        # time.sleep(0.2)

    print("End ...")


if __name__ == "__main__":
    main()
