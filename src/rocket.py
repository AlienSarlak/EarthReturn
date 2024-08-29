from elements import Elements
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
    def __init__(self, state_vector: State_Vector, mass: float = 10.0,
                 position=(0, 0), orientation: float = 0.0, size=(50, 50)) -> None:
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
        self.state_vector.y = self.body.position.y
        self.state_vector.theta = self.body.angle
        self.state_vector.y_dot = self.body.velocity.y
        self.state_vector.alpha_dot = self.body.angular_velocity

    def __repr__(self) -> str:
        return (f"Rocket(mass={self.mass}, position={self.position}, "
                f"orientation={self.orientation}, "
                f"state_vector={self.state_vector})")
