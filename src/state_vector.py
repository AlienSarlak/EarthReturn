
class State_Vector:
    def __init__(self, y: float = 0.0,
                 y_dot: float = 0.0) -> None:
        self.y = y
        self.y_dot = y_dot

    def __repr__(self) -> str:
        return f"State_Vector(y={"{:.2f}".format(self.y)}, " \
            f"y_dot={"{:.2f}".format(self.y_dot)})"
