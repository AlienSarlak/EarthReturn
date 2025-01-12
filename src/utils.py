import numpy as np
from state_vector import State_Vector
from casadi import DM as ca_dm
from math import pi
import numpy as np


def rotate_point(x, y, cx, cy, angle):
    # Translate the point to the origin based on pymunk
    # in pymunk zero degree is on north and positive angle is cw
    # while in math zero is on east and positive angles are ccw
    translated_point = np.array([x - cx, y - cy])

    # angle = (pi/2) - angle
    # Define the rotation matrix
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])

    # Apply the rotation
    rotated_point = rotation_matrix @ translated_point

    # Translate back to the original center
    final_point = rotated_point + np.array([cx, cy])

    return tuple(final_point)


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


def state_space_to_mpc_vector(state_vector: State_Vector) -> ca_dm:
    vector = ca_dm([[state_vector.x],
                    [state_vector.y],
                    [state_vector.alpha],
                    [state_vector.x_dot],
                    [state_vector.y_dot],
                    [state_vector.alpha_dot]])
    return vector


def mpc_vector_to_state_space(mpc_vector: ca_dm) -> State_Vector:
    vector = State_Vector(
        x=(mpc_vector[0]),
        y=(mpc_vector[1]),
        alpha=(mpc_vector[2]),
        x_dot=(mpc_vector[3]),
        y_dot=(mpc_vector[4]),
        alpha_dot=(mpc_vector[5])
    )
    return vector
