import pymunk
from rocket import Rocket


# FIXME: This is a draft physics simulation need more

class Physics_Simulator:
    def __init__(self,rocket:Rocket,
                 gravity_x: float = 0.0,
                 gravity_y: float = -981) -> None:
        self._gravity = gravity_x, gravity_y

        self._rocket = rocket

        self._space = pymunk.Space()
        self._space.gravity = self._gravity
        self._space.add(self._rocket.body, self.rocket.shape)

        self._print_options = pymunk.SpaceDebugDrawOptions()

        # Create a ground (static body)
        # ground = pymunk.Segment(self._space.static_body,
        #                         (50, 50),
        #                         (550, 50), 5)
        # ground.friction = 1.0
        # self._space.add(ground)


    @property
    def rocket(self):
        return self._rocket

    def update_rocket_state(self, dt):
        self._space.step(dt)

    def __repr__(self) -> str:
        return f"Physics: {self._space.debug_draw(self._print_options)}"
