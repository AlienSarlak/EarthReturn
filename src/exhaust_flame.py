import pygame
import random
import math


class Particle:
    def __init__(self, position, velocity, lifetime, ground_level=885):
        self.position = list(position)
        self.initial_position = list(position)
        self.velocity = list(velocity)

        lifetime_range = (max(0, (lifetime - 20)), (lifetime + 20))
        self.lifetime = random.randint(*lifetime_range)
        self.initial_lifetime = self.lifetime

        self.radius = 3

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
                int(self.middle_color[2] * (1 - t) + self.start_color[2] * t)
            )
        else:
            t = fraction * 2
            color = (
                int(self.end_color[0] * (1 - t) + self.middle_color[0] * t),
                int(self.end_color[1] * (1 - t) + self.middle_color[1] * t),
                int(self.end_color[2] * (1 - t) + self.middle_color[2] * t)
            )

        # color_variation = random.randint(-30, 30)
        # color = (
        #     max(0, min(255, color[0] + color_variation)),
        #     max(0, min(255, color[1] + color_variation)),
        #     max(0, min(255, color[2] + color_variation)),
        # )

        return color

    def update(self):
        # Update position based on velocity
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        # if self.position[1] < self.initial_position[1]:
        #     self.lifetime = 0
        #     return

        # Detect collision with ground
        if self.position[1] >= self.ground_level - self.radius:
            bounce_factor = random.uniform(0.1, 0.5)

            self.velocity[1] = -self.velocity[1] * bounce_factor
            self.velocity[0] += random.uniform(-2, 2)
            self.position[1] = self.ground_level - self.radius
            self.lifetime -= 15

        else:
            # Decrease lifetime
            self.lifetime -= 2

        # Shrink particle as it ages
        self.radius = max(1, self.radius - 0.1)

    def is_alive(self):
        return self.lifetime > 0

    def draw(self, screen):
        current_color = self.interpolate_color()

        pygame.draw.circle(screen, current_color, (int(
            self.position[0]), int(self.position[1])), int(self.radius))


class ExhaustFlame:

    def __init__(self,
                 ground: float,
                 position: tuple,
                 angle: float,
                 thrust_force: tuple,
                 number_of_particles: int):
        # Origin of the flame
        self.ground = ground
        self.position = position
        self.thrust_force = thrust_force
        self.number_of_particles = number_of_particles
        self.angle = angle

        # List to hold active particles
        self.particles = []

    def emit(self):
        force_magnitude = math.sqrt(self.thrust_force[0]**2 +
                                    self.thrust_force[1]**2) / 10
        if force_magnitude < 5:
            return
        for _ in range(self.number_of_particles):
            # The angle of rocket body in radians
            angle = self.angle
            # ±30 degrees random deviation
            deviation = random.normalvariate(0, math.pi / 30)
            # Random speed
            speed = random.uniform(2, force_magnitude)

            # Calculate velocity with deviation
            vx = math.sin(angle + deviation) * speed
            vy = math.cos(angle + deviation) * speed

            # Create a particle
            particle = Particle(position=self.position, velocity=(
                vx, vy), lifetime=random.randint(30, 50),
                ground_level=self.ground,)
            self.particles.append(particle)

    def update(self):
        # Update all particles
        for particle in self.particles:
            particle.update()

        # Remove dead particles
        self.particles = [p for p in self.particles if p.is_alive()]

    def draw(self, screen):
        for particle in self.particles:
            particle.draw(screen)
