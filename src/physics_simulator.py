import pymunk
from rocket import Rocket


# FIXME: This is a draft physics simulation need more

class Physics_Simulator:
    def __init__(self,rocket:Rocket,
                 gravity_x: float = 0.0,
                 gravity_y: float = -9.81) -> None:
        self._gravity = gravity_x, gravity_y
        self._space = pymunk.Space()
        self._rocket = rocket
        self._space.add(self._rocket.body)

        # Create a ground (static body)
        ground = pymunk.Segment(self._space.static_body,
                                (50, 50),
                                (550, 50), 5)
        ground.friction = 1.0
        self._space.add(ground)


    @property
    def rocket(self):
        return self._rocket

    def update_rocket_state(self, dt):
        self._space.step(dt)
