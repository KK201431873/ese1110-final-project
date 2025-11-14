"""
Defines the expected sensor variable keys and their data types for Arduino -> Raspberry Pi communication.

Expected serial message format:
    <key>:<value>\n
Example:
    imu.roll:12.45

Each key is defined in `serial_entries` and maps to a type/cast function.
"""

from dataclasses import dataclass
from typing import Callable, Any
from utils.vector2 import Vector2
from utils.pose2 import Pose2

@dataclass(frozen=True)
class SerialEntry:
    cast: Callable[[str], Any]
    description: str = ""
    units: str = ""

sensor_variables: dict[str, SerialEntry] = {
    # Drive encoders
    "encoder.left": SerialEntry(int, "Left drive encoder count", "ticks"),
    "encoder.right": SerialEntry(int, "Right drive encoder count", "ticks"),

    # IMU
    "imu.roll": SerialEntry(float, "IMU roll angle", "deg"),
    "imu.pitch": SerialEntry(float, "IMU pitch angle", "deg"),
    "imu.yaw": SerialEntry(float, "IMU yaw angle", "deg"),

    # Localization
    "localization.pose": SerialEntry(
        lambda s: Pose2(Vector2(0,0),0,s=s), 
        "Robot absolute pose", 
        "(x: meters, y: meters, h:rad)"
    ),
    # "localization.x": SerialEntry(float, "Robot absolute X coordinate", "meters"),
    # "localization.y": SerialEntry(float, "Robot absolute Y coordinate", "meters"),
    # "localization.h": SerialEntry(float, "Robot absolute heading", "rad"),

    # Debug
    "debug": SerialEntry(str, "Debug message string", "")
}

__all__ = ["sensor_variables", "SerialEntry"]