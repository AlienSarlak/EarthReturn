import pymunk
import pygame
from rocket import Rocket
from elements import Elements
from math import degrees
from exhaust_flame import ExhaustFlame


class Physics_Simulator(Elements):
    def __init__(self, rocket: Rocket,
                 gravity_x: float = 0.0,
                 gravity_y: float = +981,
                 ) -> None:
        self._gravity = gravity_x, gravity_y

        self._rocket = rocket

        self._space = pymunk.Space()
        self._space.gravity = self._gravity

        ground_body = self._space.static_body

        self.groud_level = 1000
        self.groud_tickness = 10
        ground_shape = pymunk.Segment(
            ground_body,
            (0, 0) if gravity_y < 0 else (0, self.groud_level),
            (self.groud_level, 0) if gravity_y < 0 else (
                self.groud_level, self.groud_level),
            self.groud_tickness
        )

        ground_shape.friction = 0.02
        self._space.add(ground_shape)
        self._space.add(self._rocket.body, self._rocket.shape)

        self._print_options = pymunk.SpaceDebugDrawOptions()

        self.exhaust_flame = ExhaustFlame(ground=self.groud_level - self.groud_tickness,
                                          position=(0, 0),
                                          thrust_force=1,
                                          number_of_particles=250)

    @property
    def rocket(self):
        return self._rocket

    def update_rocket_state(self, dt):
        self._space.step(dt)
        self.rocket.update_state_vector()

    def __repr__(self) -> str:
        # return f"{self._space.debug_draw(self._print_options)}"
        return f"p: {self._rocket.body.position}, v: {self._rocket.body.velocity}"

    def apply_forces_to_rocket(self, force):
        self._rocket.apply_force(force=force)

    def draw(self, screen):
        # Draw the rocket
        rotated_image = pygame.transform.rotate(
            self._rocket.image, -degrees(self._rocket.body.angle))
        rect = rotated_image.get_rect(
            center=(self._rocket.body.position.x, self._rocket.body.position.y))
        screen.blit(rotated_image, rect.topleft)

        self.exhaust_flame.position = (
            self._rocket.body.position.x,
            self._rocket.body.position.y+self._rocket.size.height/2)

        self.exhaust_flame.thrust_force = abs(self._rocket.current_thrust)/10

        self.exhaust_flame.number_of_particles = int(
            self.exhaust_flame.thrust_force)*5

        # Emit new particles
        self.exhaust_flame.emit()

        # Update and draw particles
        self.exhaust_flame.update()
        self.exhaust_flame.draw(screen)
