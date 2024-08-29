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
        self.objects = []

    def add_object(self, object):
        self.objects.append(object)

    def remove_object(self, obj):
        if obj in self.objects:
            self.objects.remove(obj)

    def update(self):
        self.screen.fill((55, 55, 55))

        for obj in self.objects:
            obj.draw(self.draw_options)

        pygame.display.flip()
        self.clock.tick(self.fps)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()
