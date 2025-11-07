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

@dataclass(frozen=True)
class SerialEntry:
    cast: Callable[[str], Any]
    description: str = ""
    units: str = ""

sensor_variables: dict[str, SerialEntry] = {
    # Drive encoders
    "sensor.encoder.left": SerialEntry(float, "Left drive encoder count", "ticks"),
    "sensor.encoder.right": SerialEntry(float, "Right drive encoder count", "ticks"),

    # IMU
    "sensor.imu.roll": SerialEntry(float, "IMU roll angle", "deg"),
    "sensor.imu.pitch": SerialEntry(float, "IMU pitch angle", "deg"),
    "sensor.imu.yaw": SerialEntry(float, "IMU yaw angle", "deg"),

    # Debug
    "sensor.debug": SerialEntry(str, "Debug message string", "")
}

__all__ = ["sensor_variables", "SerialEntry"]