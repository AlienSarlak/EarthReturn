import numpy as np
from state_vector import State_Vector


def sv2np(x: State_Vector):
    return np.array([
        [x.y],
        [x.theta],
        [x.y_dot],
        [x.alpha_dot],
    ])


def np2sv(x: np.array):
    return State_Vector(x[0, 0],
                        x[1, 0],
                        x[2, 0],
                        x[3, 0])
