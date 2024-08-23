from physics_simulator import Physics_Simulator
from rocket import Rocket
from state_vector import State_Vector

import pymunk.pygame_util
import pygame
from pygame.locals import QUIT


def game_init():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((800, 1000))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    return screen, clock, draw_options


def game_loop(screen, clk, draw_opt, physics):
    # Run the simulation with Pygame
    running = True
    fps = 60
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        screen.fill((55, 55, 55))
        physics.update_rocket_state(dt=1/fps, draw_options=draw_opt)
        print(physics)
        pygame.display.flip()  # Update the display
        clk.tick(fps)

    pygame.quit()


def main():
    screen, clk, draw_opt = game_init()

    print("Start ...")
    rocket_y = 200
    state = State_Vector(y=rocket_y, y_dot=50)
    rocket = Rocket(state_vector=state, mass=20, position=(300, rocket_y))
    ps = Physics_Simulator(rocket=rocket)
    game_loop(screen=screen, clk=clk, draw_opt=draw_opt, physics=ps)


if __name__ == "__main__":
    main()
