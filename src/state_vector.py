class State_Vector:
    def __init__(self, y: float = 0.0, theta: float = 0.0, y_dot: float = 0.0, theta_dot: float = 0.0) -> None:
        self.y = y
        self.theta = theta
        self.y_dot = y_dot
        self.theta_dot = theta_dot

    def __repr__(self) -> str:
        return f"State_Vector(y={self.y}, testa={self.theta}, y_dot={self.y_dot}, theta_dot={self.theta_dot})"
