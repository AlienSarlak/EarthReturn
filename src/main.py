from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector
from visualize import Visualize


def main():
    print("Start ...")
    v = Visualize()

    rocket_y = 200
    state = State_Vector(y=rocket_y, y_dot=50)
    rocket = Rocket(state_vector=state, mass=20, position=(300, rocket_y))
    ps = Physics_Simulator(rocket=rocket)
    v.loop(physics=ps)

    print("End ...")


if __name__ == "__main__":
    main()
