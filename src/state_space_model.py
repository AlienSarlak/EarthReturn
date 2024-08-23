import numpy as np
from state_vector import State_Vector
from utils import np2sv


class StateSpaceModel:
    def __init__(self, gravity, mass, moi, length) -> None:
        self.A = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

        self.B = np.array(
            [
                [0],
                [0],
                [1/mass],
                [length/moi]
            ]
        )

        self.f = np.array(
            [
                [0],
                [0],
                [-gravity],
                [0]
            ]
        )

    def evolution(self, current_state: State_Vector, u: np.array):
        # X = AX + Bu + f
        X_old = np.array([
            [current_state.y],
            [current_state.theta],
            [current_state.y_dot],
            [current_state.theta_dot],
        ])
        X_new = self.A @ X_old + self.B @ u + self.f
        return np2sv(X_new)
