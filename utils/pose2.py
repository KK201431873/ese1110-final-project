from .vector2 import Vector2

class Pose2:
    """2D Vector and heading angle to describe planar robot pose."""

    def __init__(self, COORDS: Vector2, heading: float, s: str = ""):
        """
        Initialize a new Pose2. Passing a string of format "`x`;`y`;`h`" will use those values
        as the constructor instead.
        """
        # Default init
        self.COORDS = COORDS
        self.h = heading

        # Overwrite if s is valid
        if len(s) > 0:
            values = s.split(";")
            if len(values) != 3:
                return
            try:
                x = float(values[0])
                y = float(values[1])
                h = float(values[2])
                self.COORDS = Vector2(x, y)
                self.h = h
            except ValueError:
                return