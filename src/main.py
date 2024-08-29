from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize


def main():
    print("Start ...")

    fps = 60
    visualizer = Visualize(fps=fps)
    rocket_y = 200
    state = State_Vector(y=rocket_y, y_dot=50)
    rocket = Rocket(state_vector=state, mass=20, position=(300, rocket_y))
    ps = Physics_Simulator(rocket=rocket)

    visualizer.add_object(ps)

    running = True

    while running:
        visualizer.handle_events()  # Handle events such as window close
        ps.update_rocket_state(dt=1/fps)
        visualizer.update()

    print("End ...")


if __name__ == "__main__":
    main()
