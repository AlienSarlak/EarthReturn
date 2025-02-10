import pygame
import random
from math import sin, cos, pi


class Particle:
    def __init__(self, position, velocity, lifetime, ground_level=885):
        self.position = list(position)
        self.initial_position = list(position)
        self.velocity = list(velocity)

        lifetime_range = (max(0, (lifetime - 20)), (lifetime + 20))
        self.lifetime = random.randint(*lifetime_range)
        self.initial_lifetime = self.lifetime

        self.radius = 3
        self.dust = False
        # Define colors for transition
        self.start_color = (255, 0, 0)  # Red
        self.middle_color = (255, 255, 0)  # Yellow
        self.end_color = (255, 255, 255)  # White

        # Ground level for collision
        self.ground_level = ground_level

    def interpolate_color(self):
        fraction = self.lifetime / self.initial_lifetime

        if fraction > 0.5:
            t = (fraction - 0.5) * 2
            color = (
                int(self.middle_color[0] * (1 - t) + self.start_color[0] * t),
                int(self.middle_color[1] * (1 - t) + self.start_color[1] * t),
                int(self.middle_color[2] * (1 - t) + self.start_color[2] * t),
            )
        else:
            t = fraction * 2
            color = (
                int(self.end_color[0] * (1 - t) + self.middle_color[0] * t),
                int(self.end_color[1] * (1 - t) + self.middle_color[1] * t),
                int(self.end_color[2] * (1 - t) + self.middle_color[2] * t),
            )

        return color

    def update(self, src_position: tuple):
        # Update position based on velocity
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        # Detect collision with ground
        if self.position[1] >= self.ground_level - self.radius:
            bounce_factor = random.uniform(0.1, 0.5)

            self.velocity[1] = -self.velocity[1] * bounce_factor
            self.velocity[0] += random.uniform(-2, 2)
            self.position[1] = self.ground_level - self.radius
            # self.lifetime -= 2
            self.dust = True

        self.lifetime -= 1

        # Ensure particles are not on the body of the rocket
        if (src_position[1] - self.position[1]) > 0:
            self.lifetime -= 3

        # Shrink particle as it ages
        self.radius = max(1, self.radius - 0.1)

    def is_alive(self):
        return self.lifetime > 0

    def draw(self, screen):

        current_color = (
            int(135),
            int(35),
            int(0),
        )

        r = self.radius

        if self.dust:
            self.radius+=0.1
            r = self.radius

        if not self.dust:
            current_color = self.interpolate_color()




        pygame.draw.circle(screen, current_color, (int(
            self.position[0]), int(self.position[1])), int(r))


class ExhaustFlame:

    def __init__(self,
                 ground: float,
                 position: tuple,
                 angle: float,
                 thrust_force: float,
                 number_of_particles: int):
        # Origin of the flame
        self.ground = ground
        self.position = position
        self.thrust_force = abs(thrust_force)
        self.number_of_particles = number_of_particles
        self.angle = angle

        # List to hold active particles
        self.particles = []

    def emit(self):
        force_magnitude = abs(self.thrust_force) / 10
        if force_magnitude < 5:
            return
        for _ in range(self.number_of_particles):
            # The angle of rocket body in radians
            angle = self.angle
            # ±30 degrees random deviation
            deviation = random.normalvariate(0, pi / 30)
            # Random speed
            speed = random.uniform(2, force_magnitude)

            # Calculate velocity with deviation
            vx = sin(angle + deviation) * speed
            vy = cos(angle + deviation) * speed

            # Create a particle
            particle = Particle(position=self.position, velocity=(
                vx, vy), lifetime=random.randint(30, 50),
                ground_level=self.ground,)
            self.particles.append(particle)

    def update(self):
        # Update all particles
        for particle in self.particles:
            particle.update(self.position)

        # Remove dead particles
        self.particles = [p for p in self.particles if p.is_alive()]

    def draw(self, screen):
        for particle in self.particles:
            particle.draw(screen)
