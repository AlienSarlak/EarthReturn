import pymunk
import pygame
from rocket import Rocket
from elements import Elements
from math import degrees
from exhaust_flame import ExhaustFlame
from math import exp
from utils import rotate_point


class Physics_Simulator(Elements):
    def __init__(self, rocket: Rocket,
                 ground_height,
                 gravity_x: float = 0.0,
                 gravity_y: float = +981,
                 ) -> None:
        self._gravity = gravity_x, gravity_y

        self._rocket = rocket

        self._space = pymunk.Space()
        self._space.gravity = self._gravity

        ground_body = self._space.static_body

        self.groud_level = ground_height
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
                                          angle=0.0,
                                          number_of_particles=500)

    @property
    def rocket(self):
        return self._rocket

    def update_rocket_state(self, dt):
        self._space.step(dt)
        self.rocket.update_state_vector()

    def __repr__(self) -> str:
        # return f"{self._space.debug_draw(self._print_options)}"
        return f"p: {self._rocket.body.position}, v: {self._rocket.body.velocity}"

    def draw(self, screen):
        # Draw the rocket

        # Unfiltered counterclockwise rotation.
        # The angle argument represents degrees and can be any floating point value.
        # Negative angle amounts will rotate clockwise.
        rotated_image = pygame.transform.rotate(
            self._rocket.image, degrees(self._rocket.body.angle))

        rect = rotated_image.get_rect(
            center=(self._rocket.body.position.x, self._rocket.body.position.y))
        screen.blit(rotated_image, rect.topleft)


        # \frac{1}{1+e^{-4\left(x\cdot2-1\right)}}
        x = abs(self.rocket.current_thrust) / 600.0
        self.exhaust_flame.number_of_particles = int(1/(1+exp(-4*(2*x-1))) * 700)

        # Exhuast flame
        self.exhaust_flame.position = rotate_point(x=rect.centerx,
                                                   y=rect.centery +
                                                   (self._rocket.size.height * 0.5),
                                                   cx=rect.centerx,
                                                   cy=rect.centery,
                                                   angle=-self._rocket.body.angle)

        self.exhaust_flame.angle = self._rocket.body.angle + self._rocket.nozzle_angle

        # From current_thrust we can identify the direction and magnitude
        # of the force and there is no need for rocket angle
        self.exhaust_flame.thrust_force = self._rocket.current_thrust

        # Emit new particles
        self.exhaust_flame.emit()

        #########################################################
        #########################################################
        #########################################################
        #########################################################
        # pygame.draw.circle(screen, (0, 0, 255), (int(
        #     self._rocket.wpx), int(self._rocket.wpy)), 10)

        # Update and draw particles
        self.exhaust_flame.update()
        self.exhaust_flame.draw(screen)
