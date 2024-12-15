import pygame
import pymunk
import os
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
                 position=(0, 0), orientation: float = 0.0, size=(190/2, 210/2)) -> None:
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

        # set the initial state to pymunk
        self.body.velocity = pymunk.Vec2d(0, state_vector.y_dot)
        # self.body.angular_velocity = state_vector.alpha_dot

        image_path = os.path.join(os.path.dirname(
            __file__), 'img', 'rocket-model.png')
        # Load the rocket image

        self.image = pygame.image.load(image_path)
        self.image = pygame.transform.scale(
            self.image, (self.size.width, self.size.height))

        self.current_thrust = 0

    def update_state_vector(self) -> None:
        self.state_vector.y = self.body.position.y
        # self.state_vector.alpha = self.body.angle
        self.state_vector.y_dot = self.body.velocity.y
        # self.state_vector.alpha_dot = self.body.angular_velocity

    def apply_force(self, force, point=(0, 0)):
        # the unit must be 100 times since pymunk understand pixel/second (pps)
        self.current_thrust = force[1]
        force = tuple([x * 100 for x in force])
        self.body.apply_force_at_local_point(force, point)

    def __repr__(self) -> str:
        return (f"Rocket(mass={self.mass}, position={self.position}, "
                f"orientation={self.orientation}, "
                f"state_vector={self.state_vector})")
