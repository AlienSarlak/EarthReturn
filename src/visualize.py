import pymunk.pygame_util
import pygame
from pygame.locals import QUIT


class Visualize:
    def __init__(self, fps: int = 60) -> None:
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 1000))
        self.clock = pygame.time.Clock()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.fps = fps

    def loop(self, physics):
        # Run the simulation with Pygame
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False

            self.screen.fill((55, 55, 55))
            physics.update_rocket_state(
                dt=1/self.fps, draw_options=self.draw_options)
            # print(physics)
            pygame.display.flip()  # Update the display
            self.clock.tick(self.fps)

        pygame.quit()
