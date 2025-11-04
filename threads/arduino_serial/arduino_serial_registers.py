"""
Defines the expected serial keys and their data types for Arduino -> Raspberry Pi communication.

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

serial_entries: dict[str, SerialEntry] = {
    # IMU
    "imu.roll": SerialEntry(float, "IMU roll angle", "deg"),
    "imu.pitch": SerialEntry(float, "IMU pitch angle", "deg"),
    "imu.yaw": SerialEntry(float, "IMU yaw angle", "deg"),

    # Debug
    "debug": SerialEntry(str, "Debug message string", "")
}

__all__ = ["serial_entries", "SerialEntry"]