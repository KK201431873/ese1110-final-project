from .dynamic_motion_profile_1D import DynamicMotionProfile1D
from utils.vector2 import Vector2

class DynamicMotionProfile2D:
    motion_profile_1d: DynamicMotionProfile1D

    """Calculates the linear trajectory between two points, given start position and velocity vectors."""
    def __init__(self, 
                 p0: Vector2, 
                 p1: Vector2, 
                 v0: Vector2, 
                 v_max: float, 
                 a_max: float
                 ):
        self.p0: Vector2 = p0
        self.p1: Vector2 = p1
        
        # Calculate projection of initial velocity onto displacement vector
        self.displacement: Vector2 = self.p1 - self.p0
        self.direction_uvec = self.displacement.normalize()
        v0_1d: float = v0.dot(self.direction_uvec)

        # Init 1D motion profile along displacement vector
        self.distance: float = self.displacement.norm()
        self.motion_profile_1d = DynamicMotionProfile1D(
            target_displacement = self.distance,
            start_time = 0,
            v0 = v0_1d,
            v_max = v_max,
            a_max_1 = a_max,
            a_max_2 = a_max
        )
    
    def get_position(self, elapsed_time: float) -> Vector2:
        """Get the position at `elapsed_time` according to this motion profile."""
        if self.distance == 0 or self.motion_profile_1d is None:
            return self.p0
        displacement_1d = self.motion_profile_1d.get_displacement(elapsed_time)
        t = displacement_1d / self.distance
        return self.p0 + self.displacement * t

    def get_velocity(self, elapsed_time: float) -> Vector2:
        """Get the velocity at `elapsed_time` according to this motion profile."""
        if self.motion_profile_1d is None:
            return Vector2(0, 0)
        velocity_1d = self.motion_profile_1d.get_velocity(elapsed_time)
        return self.direction_uvec * velocity_1d
    
    def get_acceleration(self, elapsed_time: float) -> Vector2:
        """Get the acceleration at `elapsed_time` according to this motion profile."""
        if self.motion_profile_1d is None:
            return Vector2(0, 0)
        acceleration_1d = self.motion_profile_1d.get_acceleration(elapsed_time)
        return self.direction_uvec * acceleration_1d