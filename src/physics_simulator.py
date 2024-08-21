import pymunk
from rocket import Rocket

# FIXME: This is a draft physics simulation need more


class Physics_Simulator:
    def __init__(self, rocket: Rocket,
                 gravity_x: float = 0.0,
                 gravity_y: float = +981) -> None:
        self._gravity = gravity_x, gravity_y

        self._rocket = rocket

        self._space = pymunk.Space()
        self._space.gravity = self._gravity

        # Create the Ground (Static Body)
        ground_body = self._space.static_body

        ground_shape = pymunk.Segment(
            ground_body,
            (0, 0) if gravity_y < 0 else (0, 1000),
            (1000, 0) if gravity_y < 0 else (1000, 1000),
            10
        )
        # ground_shape = pymunk.Segment(ground_body, (0, 0), (1000, 0), 10)
        ground_shape.friction = 0.02
        self._space.add(ground_shape)
        self._space.add(self._rocket.body, self.rocket.shape)

        # Assign collision types
        ground_shape.collision_type = 1
        self.rocket.shape.collision_type = 2

        handler = self._space.add_collision_handler(2, 1)
        handler.begin = self.on_rocket_landing

        self._print_options = pymunk.SpaceDebugDrawOptions()

    @property
    def rocket(self):
        return self._rocket

    def on_rocket_landing(self, arbiter, space, data):
        rocket, ground = arbiter.shapes
        rocket.body.position = (
            rocket.body.position.x,
            (
                (ground.a.y + ground.b.y) * 0.5 +
                (self._rocket.size.height * 0.5 + ground.radius)
                if self._gravity[1] < 0 else
                (ground.a.y + ground.b.y) * 0.5 -
                (self._rocket.size.height * 0.5 + ground.radius)
            ))
        rocket.body.velocity = (0, 0)
        return True

    def update_rocket_state(self, dt, draw_options):
        self._space.step(dt)
        if draw_options is not None:
            self._space.debug_draw(draw_options)

    def __repr__(self) -> str:
        # return f"{self._space.debug_draw(self._print_options)}"
        return f"p: {self._rocket.body.position}, v: {self._rocket.body.velocity}"
