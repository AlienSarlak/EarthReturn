import pymunk
import pygame
from rocket import Rocket
from elements import Elements
from math import degrees

# FIXME: This is a draft physics simulation need more


class Physics_Simulator(Elements):
    def __init__(self, rocket: Rocket,
                 gravity_x: float = 0.0,
                 gravity_y: float = +981) -> None:
        self._gravity = gravity_x, gravity_y

        self._rocket = rocket

        self._space = pymunk.Space()
        self._space.gravity = self._gravity

        ground_body = self._space.static_body

        ground_shape = pymunk.Segment(
            ground_body,
            (0, 0) if gravity_y < 0 else (0, 1000),
            (1000, 0) if gravity_y < 0 else (1000, 1000),
            10
        )

        ground_shape.friction = 0.02
        self._space.add(ground_shape)
        self._space.add(self._rocket.body, self._rocket.shape)

        self._print_options = pymunk.SpaceDebugDrawOptions()

    @property
    def rocket(self):
        return self._rocket

    def update_rocket_state(self, dt):
        self._space.step(dt)
        self.rocket.update_state_vector()

    def __repr__(self) -> str:
        # return f"{self._space.debug_draw(self._print_options)}"
        return f"p: {self._rocket.body.position}, v: {self._rocket.body.velocity}"

    def draw(self, screen, draw_option):
        # self._space.debug_draw(draw_option)
        rotated_image = pygame.transform.rotate(
            self._rocket.image, -degrees(self._rocket.body.angle))
        rect = rotated_image.get_rect(
            center=(self._rocket.body.position.x, self._rocket.body.position.y))
        screen.blit(rotated_image, rect.topleft)
