from dynamic_motion_profile_2D import DynamicMotionProfile2D
from utils.vector2 import Vector2
import math
import time

def clamp(x, lower, upper):
    return max(lower, min(x, upper))

def sinc(x):
    """Numerically stable sinc(x) = sin(x)/x with x in radians."""
    if abs(x) < 1e-8:
        return 1.0 - x**2/6.0
    return math.sin(x)/x

class RAMSETEController:
    motion_profile: DynamicMotionProfile2D | None
    """The current motion profile being used by the controller."""

    motion_profile_creation_time: float | None
    """The time at which the current motion profile was created."""

    last_target_pos: Vector2 | None
    """The last target position used by the controller."""

    def __init__(self, 
                 b: float, 
                 zeta: float,
                 v_max: float,
                 a_max: float,
                 replanning_max_delay: float,
                 replanning_drift_threshold: float,
                 lookahead_time: float,
                 wheel_diameter: float,
                 wheel_base: float):
        self.b = b
        """Responsiveness tuning parameter for the RAMSETE controller."""

        self.zeta = zeta
        """Damping coefficient for the RAMSETE controller."""

        self.v_max = v_max
        """Maximum linear velocity (m/s)."""

        self.a_max = a_max
        """Maximum linear acceleration (m/s^2)."""

        self.replanning_max_delay = replanning_max_delay
        """Maximum time before forcing a new motion profile (seconds)."""

        self.replanning_drift_threshold = replanning_drift_threshold
        """Maximum movement of the ball before replanning (meters)."""
        
        self.lookahead_time = lookahead_time
        """Lookahead time for the motion profile (seconds)."""

        self.wheel_circumference = math.pi * wheel_diameter
        """Circumference of the wheels (meters)."""

        self.wheel_base = wheel_base
        """Distance between the left and right wheels (meters)."""

        # Initialize motion profile to None
        self.motion_profile = None
        self.motion_profile_creation_time = None
    
    def update_relative(self,
                        ball_pos: Vector2,
                        current_vel: Vector2) -> tuple[float, float]:
        """
        Calculate motor velocities in meters per second given ball position relative to 
        the robot and current velocity of the robot. Use METERS as distance unit.
        """

        now = time.perf_counter()

        # Conditions for recalculating motion profile
        if (self.motion_profile is None or \
            self.motion_profile_creation_time is None or \
            self.last_target_pos is None or \
            now - self.motion_profile_creation_time > self.replanning_max_delay or \
            (ball_pos - self.last_target_pos).norm() > self.replanning_drift_threshold
            ):
            self.motion_profile = DynamicMotionProfile2D(
                Vector2(0, 0),
                ball_pos,
                current_vel,
                self.v_max,
                self.a_max
            )
            self.motion_profile_creation_time = now
            self.last_target_pos = ball_pos

        # Calculate targets and errors
        elapsed = now - self.motion_profile_creation_time
        target_pos = self.motion_profile.get_position(elapsed + self.lookahead_time)
        target_vel = self.motion_profile.get_velocity(elapsed + self.lookahead_time)

        v_d = target_vel.norm()
        theta_d = math.atan2(target_vel.y, target_vel.x)
        omega_d = 0.0 # trajectory is a straight line

        e_x = target_pos.x
        e_y = target_pos.y
        e_theta = math.atan2(math.sin(theta_d), math.cos(theta_d))  # normalized theta_d

        # Control law
        k = 2 * self.zeta * math.sqrt(omega_d**2 + self.b * v_d**2)
        v = v_d * math.cos(e_theta) + k * e_x # units: m/s
        omega = omega_d + k * e_theta + self.b * v_d * sinc(e_theta) * e_y # units: rad/s

        # Convert to left and right wheel velocities
        left = v - omega * (self.wheel_base / 2)
        right = v + omega * (self.wheel_base / 2)

        # Clamp
        left = clamp(left, -self.v_max, self.v_max)
        right = clamp(right, -self.v_max, self.v_max)

        return (left, right)