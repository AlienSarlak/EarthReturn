from math import degrees


class State_Vector:
    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 alpha: float = 0.0,
                 x_dot: float = 0.0,
                 y_dot: float = 0.0,
                 alpha_dot: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.alpha = alpha
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.alpha_dot = alpha_dot

    def __repr__(self) -> str:
        return (
            f"x={self.x:.2f}, "
            f"y={self.y:.2f}, "
            f"alpha={degrees(self.alpha):.2f}, "
            f"x_dot={self.x_dot:.2f}, "
            f"y_dot={self.y_dot:.2f}, "
            f"alpha_dot={degrees(self.alpha_dot):.2f}"
        )
