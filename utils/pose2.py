from vector2 import Vector2

class Pose2:
    """2D Vector and heading angle to describe planar robot pose."""

    def __init__(self, COORDS: Vector2, heading: float):
        self.COORDS = COORDS
        self.h = heading

