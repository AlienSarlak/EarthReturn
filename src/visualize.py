import pymunk.pygame_util
import pygame
import os
from pygame.locals import QUIT


class Visualize:
    def __init__(self, width, height, fps: int = 60) -> None:
        # Initialize Pygame
        pygame.init()
        self.display_width = width
        self.display_height = height
        self.screen = pygame.display.set_mode(
            (self.display_width, self.display_height))
        pygame.display.set_caption("Landing on Earth [MPC]")
        self.clock = pygame.time.Clock()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.fps = fps

        background_path = os.path.join(os.path.dirname(
            __file__), 'img', 'bg_dark.jpg')
        self.background = pygame.image.load(background_path)
        self.background = pygame.transform.scale(
            self.background, self.screen.get_size())
        self.objects = []

    def add_object(self, object):
        self.objects.append(object)

    def remove_object(self, obj):
        if obj in self.objects:
            self.objects.remove(obj)

    def update(self):
        self.screen.blit(self.background, (0, 0))

        for obj in self.objects:
            obj.draw(self.screen)

        pygame.display.flip()
        self.clock.tick(self.fps)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()
