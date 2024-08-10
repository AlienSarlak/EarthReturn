import math

class Wind:
    def __init__(self, direction=None) -> None:
        if direction is None:
            direction = [0, 0]
        self._direction = direction
        self._force = None  # Initialize force as None; it will be calculated when accessed.

    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, new_direction):
        self._direction = new_direction
        self._force = None  # Invalidate the cached force value.

    @property
    def force(self):
        if self._force is None:
            self._force = math.sqrt(self._direction[0]**2 + self._direction[1]**2)
        return self._force
