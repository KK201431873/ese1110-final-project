class TimeSpan:
    """An Object that stores the duration, start, and end timestamps for a Movement."""
    def __init__(self, start_time: float, end_time: float):
        self.start_time = start_time
        self.end_time = end_time

    def get_start_time(self) -> float:
        return self.start_time

    def get_end_time(self) -> float:
        return self.end_time

    def get_duration(self) -> float:
        return self.end_time - self.start_time