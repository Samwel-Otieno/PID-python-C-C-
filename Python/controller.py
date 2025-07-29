class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        # Proportional
        P = self.Kp * error

        # Integral
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Save current error for next derivative calculation
        self.prev_error = error

        # PID output
        return P + I + D