import math
import pymunk
from state_vector import State_Vector


class Size:
    def __init__(self, width, height):
        self._size = (width, height)

    @property
    def width(self):
        return self._size[0]

    @property
    def height(self):
        return self._size[1]

    def __getitem__(self, index):
        return self._size[index]

    def __repr__(self):
        return f"Size(w={self.width}, h={self.height})"


class Rocket:
    """
    A class representing a rocket in a physics simulation.

    Attributes
    ----------
    mass : float
        The mass of the rocket in kilograms.
    position : tuple
        The (x, y) position of the rocket in meters in the global frame.
    orientation : float
        The orientation of the rocket in radians.
    state_vector : State_Vector
        The state vector of the rocket, containing y, theta, y_dot, and theta_dot.
    body : pymunk.Body
        The pymunk body representing the rocket in the physics simulation.
    shape : pymunk.Poly
        The shape of the rocket, attached to the pymunk body.
    """

    def __init__(self, state_vector: State_Vector, mass: float = 10.0,
                 position=(0, 0), orientation: float = 0.0, size=(50, 50)) -> None:
        """
        Initialize the Rocket with the given parameters.

        Parameters
        ----------
        state_vector : State_Vector
            The initial state vector of the rocket.
        mass : float, optional
            The mass of the rocket in kilograms (default is 0.0).
        position : tuple, optional
            The initial (x, y) position of the rocket in meters (default is (0, 0)).
        orientation : float, optional
            The initial orientation of the rocket in radians (default is 0.0).
        """
        self.mass = mass
        self.position = position
        self.orientation = orientation  # in radians
        self.state_vector = state_vector
        self.size = Size(*size)

        self.body = pymunk.Body()
        self.body.position = position
        self.body.angle = orientation
        self.body.mass = mass
        self.body.body_type = pymunk.Body.DYNAMIC

        self.shape = pymunk.Poly.create_box(self.body, size=size)
        self.shape.mass = mass
        self.shape.elasticity = 0
        self.shape.friction = 1.0

    def update_state_vector(self) -> None:
        """
        Update the state vector based on the current physics simulation state.

        This method updates the `y`, `theta`, `y_dot`, and `theta_dot` attributes
        of the `state_vector` based on the current position, angle, velocity,
        and angular velocity of the rocket's `pymunk.Body`.
        """
        self.state_vector.y = self.body.position.y
        self.state_vector.theta = self.body.angle
        self.state_vector.y_dot = self.body.velocity.y
        self.state_vector.theta_dot = self.body.angular_velocity

    def __repr__(self) -> str:
        """
        Return a string representation of the Rocket.

        Returns
        -------
        str
            A string that represents the Rocket's current state, including mass, position,
            orientation, and state vector.
        """
        return (f"Rocket(mass={self.mass}, position={self.position}, "
                f"orientation={self.orientation}, "
                f"state_vector={self.state_vector})")
