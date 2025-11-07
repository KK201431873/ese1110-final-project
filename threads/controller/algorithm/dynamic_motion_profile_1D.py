import math
from time_span import TimeSpan

class DynamicMotionProfile1D:
    """
    Python translation of DynamicMotionProfile1D.
    Models 1D motion with asymmetric acceleration/deceleration and optional time scaling.
    """

    def __init__(self, 
                 target_displacement: float, 
                 start_time: float, 
                 v0: float, 
                 v_max: float, 
                 a_max_1: float, 
                 a_max_2: float
                 ):
        if v_max <= 0:
            raise ValueError(f"v_max must be positive (given {v_max})")
        if a_max_1 <= 0:
            raise ValueError(f"a_max_1 must be positive (given {a_max_1})")
        if a_max_2 <= 0:
            raise ValueError(f"a_max_2 must be positive (given {a_max_2})")

        self.sign: float = math.copysign(1, target_displacement) if target_displacement != 0 else 1
        self.distance: float = abs(target_displacement)
        self.v_max: float = abs(v_max)
        self.a_max_1: float = abs(a_max_1)
        self.a_max_2: float = abs(a_max_2)
        self.v0: float = self._bound(v0 * (self.sign if self.sign != 0 else 1), -self.v_max, self.v_max)

        # placeholders
        self.v1: float = 0
        self.v2: float = 0
        self.t1: float = 0
        self.t2: float = 0
        self.t3: float = 0
        self.t4: float = 0
        self.T: float = 0
        self.D1: float = 0
        self.D2: float = 0
        self.D3: float = 0
        self.D4: float = 0

        self._init()

        self.time_span: TimeSpan = TimeSpan(start_time, start_time + self.T)
        self.min_duration: float = self.time_span.get_duration()

    # --- Utility --------------------------------------------------------------

    @staticmethod
    def _bound(x, lower: float, upper: float) -> float:
        return max(lower, min(upper, x))

    # --- Initialization -------------------------------------------------------

    def _init(self) -> None:
        """Equivalent to the Java init() method."""
        D1plusD3_max = self.v_max**2 * (self.a_max_1 + self.a_max_2) / (2 * self.a_max_1 * self.a_max_2) - self.v0**2 / (2 * self.a_max_1)
        D3_min = self.v0**2 / (2 * self.a_max_2)

        # Case 1: Right triangle
        if self.distance == D3_min and self.v0 >= 0:
            self.v1 = self.v0
            self.v2 = 0
            self.t1 = self.t2 = 0
            self.t3 = self.v0 / self.a_max_2
            self.t4 = self.t3
            self.T = self.t4
            self.D1 = self.D2 = 0
            self.D3 = self.v0**2 / (2 * self.a_max_2)
            self.D4 = self.D3

        # Case 2: Truncated isosceles triangle
        elif (self.distance > D3_min or self.v0 < 0) and self.distance <= D1plusD3_max:
            self.v1 = math.sqrt((self.distance + self.v0**2 / (2 * self.a_max_1)) * 2 * self.a_max_1 * self.a_max_2 / (self.a_max_1 + self.a_max_2))
            self.v2 = 0
            self.t1 = (self.v1 - self.v0) / self.a_max_1
            self.t2 = self.t1
            self.t3 = self.t1 + (self.v1 / self.a_max_2)
            self.t4 = self.t3
            self.T = self.t4
            self.D1 = (self.v1**2 - self.v0**2) / (2 * self.a_max_1)
            self.D2 = self.D1
            self.D3 = self.D2 + self.v1**2 / (2 * self.a_max_2)
            self.D4 = self.D3

        # Case 3: Truncated trapezoid
        elif self.distance > D1plusD3_max or self.v0 < 0:
            self.D1 = (self.v_max**2 - self.v0**2) / (2 * self.a_max_1)
            self.D2 = self.D1 + self.distance - D1plusD3_max
            self.D3 = self.D2 + (self.v_max**2) / (2 * self.a_max_2)
            self.D4 = self.D3
            self.v1 = self.v_max
            self.v2 = 0
            self.t1 = (self.v_max - self.v0) / self.a_max_1
            self.t2 = self.t1 + (self.D2 - self.D1) / self.v_max
            self.t3 = self.t2 + (self.v_max / self.a_max_2)
            self.t4 = self.t3
            self.T = self.t4

        # Case 4: Overshoot right triangle
        else:
            self.D1 = 0
            self.D2 = self.D1
            self.D3 = self.D2 + (self.v0**2) / (2 * self.a_max_2)
            self.D4 = self.D3 - (self.D3 - self.distance) / 2
            self.v1 = self.v0
            self.v2 = -math.sqrt((self.D3 - self.distance) * self.a_max_2)
            self.t1 = 0
            self.t2 = self.t1
            self.t3 = self.t2 + self.v0 / self.a_max_2
            self.t4 = self.t3 + math.sqrt((self.D3 - self.distance) / self.a_max_2)
            self.T = 2 * self.t4 - self.t3
    
    def get_duration(self) -> float:
        return self.T

    # --- Core functions -------------------------------------------------------

    def get_displacement(self, elapsed_time: float) -> float:
        if self.T == 0:
            return 0
        t = self._bound(elapsed_time - self.time_span.get_start_time(), 0, self.T)

        if t <= self.t1:
            disp = self.v0 * t + 0.5 * self.a_max_1 * t**2
        elif t <= self.t2:
            disp = self.D1 + self.v1 * (t - self.t1)
        elif t <= self.t3:
            dt = t - self.t2
            disp = self.D2 + self.v1 * dt - 0.5 * self.a_max_2 * dt**2
        elif t <= self.t4:
            dt = t - self.t3
            disp = self.D3 - 0.5 * self.a_max_2 * dt**2
        else:
            dt = t - self.t4
            disp = self.D4 + self.v2 * dt + 0.5 * self.a_max_2 * dt**2

        return disp * self.sign

    def get_velocity(self, elapsed_time: float) -> float:
        if self.T == 0:
            return 0
        if elapsed_time - self.time_span.get_start_time() <= 0 or elapsed_time - self.time_span.get_start_time() >= self.T:
            return 0
        t = self._bound(elapsed_time - self.time_span.get_start_time(), 0, self.T)

        if t <= self.t1:
            vel = self.v0 + self.a_max_1 * t
        elif t <= self.t2:
            vel = self.v1
        elif t <= self.t3:
            vel = self.v1 - self.a_max_2 * (t - self.t2)
        elif t <= self.t4:
            vel = -self.a_max_2 * (t - self.t3)
        else:
            vel = self.v2 + self.a_max_2 * (t - self.t4)

        return vel * self.sign

    def get_acceleration(self, elapsed_time: float) -> float:
        if self.T == 0:
            return 0
        if elapsed_time - self.time_span.get_start_time() <= 0 or elapsed_time - self.time_span.get_start_time() >= self.T:
            return 0
        t = self._bound(elapsed_time - self.time_span.get_start_time(), 0, self.T)

        if t <= self.t1:
            acc = self.a_max_1
        elif t <= self.t2:
            acc = 0
        elif t <= self.t3:
            acc = -self.a_max_2
        elif t <= self.t4:
            acc = -self.a_max_2
        else:
            acc = self.a_max_2

        return acc * self.sign
