from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector

import pymunk               # Import pymunk..

def main():

    print("Start ...")
    state = State_Vector(y=100, y_dot=5)
    rocket = Rocket(state_vector=state, position=(0, 100))
    ps = Physics_Simulator(rocket=rocket)

    for i in range(5):
        ps.update_rocket_state(dt=0.01)
        print(ps)

    print("end.")

if __name__ == "__main__":
    main()
