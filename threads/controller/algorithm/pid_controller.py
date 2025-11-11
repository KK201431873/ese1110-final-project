import time

class PIDController:
    def __init__(self, kP: float, kI: float, kD: float):
        """Create a new PID controller with the given tuning constants."""
        # Constants
        self.kP = kP
        self.kI = kI
        self.kD = kD

        # States
        self.last_error = 0
        self.total_error = 0
        self.last_update_time = time.perf_counter()
    
    def update(self, setpoint: float, state: float) -> float:
        """Update using the given setpoint and state. Returns the combined control output."""

        # Get delta time
        now = time.perf_counter()
        delta_time = now - self.last_update_time
        self.last_update_time = now

        # Calculate error
        error = state - setpoint

        # P term
        p = self.kP * error

        # I term
        self.total_error += error * delta_time
        i = self.kI * self.total_error

        # D term
        derivative = (error - self.last_error) / delta_time
        d = self.kD * derivative

        # Control law
        u = p + i + d
        return u