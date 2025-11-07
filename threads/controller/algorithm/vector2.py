class Vector2:
    """2D vector class with basic operations."""

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector2":
        return Vector2(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: float) -> "Vector2":
        return Vector2(self.x / scalar, self.y / scalar)

    def dot(self, other: "Vector2") -> float:
        return self.x * other.x + self.y * other.y

    def norm(self) -> float:
        return (self.x**2 + self.y**2) ** 0.5

    def normalize(self) -> "Vector2":
        n = self.norm()
        if n == 0:
            return Vector2(0, 0)
        return self / n