class PID:
    """
    Control input: wheel rpm/angular velocity
    State: [pitch]
    """

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt) -> float:
        proportional = error
        self.integral = (error + self.previous_error) / 2 * dt
        derivative = (error - self.previous_error) / dt
        return self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
