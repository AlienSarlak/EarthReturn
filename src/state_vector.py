
class State_Vector:
    def __init__(self, y: float = 0.0,
                 alpha: float = 0.0,
                 y_dot: float = 0.0,
                 alpha_dot: float = 0.0) -> None:
        self.y = y
        self.alpha = alpha
        self.y_dot = y_dot
        self.alpha_dot = alpha_dot

    def __repr__(self) -> str:
        return f"State_Vector(y={"{:.2f}".format(self.y)}, " \
            f"alpha={"{:.2f}".format(self.alpha)}, " \
            f"y_dot={"{:.2f}".format(self.y_dot)}, " \
            f"alpha_dot={"{:.2f}".format(self.alpha_dot)})"
