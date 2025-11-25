from utils.mcu_serial_interface import MCUSerialInterface
from utils.debug import print_from
from .sensor_thread_registers import sensor_variables
from typing import Any
import math

# --- Sensor functions ---
def set_config(
        wheel_diameter: float,
        wheel_base: float,
        ticks_per_rev: float,
        gear_ratio: float,
        alpha: float) -> None:
    """Set the localization constants to the given values."""
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.config.wheel_diameter:{wheel_diameter}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.config.wheel_base:{wheel_base}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.config.ticks_per_rev:{ticks_per_rev}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.config.gear_ratio:{gear_ratio}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.config.alpha:{alpha}")

def reset_IMU(radians: float = 0) -> None:
    """Reset the IMU yaw to zero (or the given angle in radians)."""
    radians = normalize_angle(radians)
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.imu.yaw:{radians}")

def reset_encoders(left: int = 0, right: int = 0) -> None:
    """Reset the left and right encoder counts to zero (or the given values)."""
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.encoder.left:{left}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.encoder.right:{right}")

def reset_localization(x: float = 0, y: float = 0, h: float = 0) -> None:
    """Reset the localization pose to zero (or the given values, where `x,y` are in meters and `h` is in radians)."""
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.localization.x:{x}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.localization.y:{y}")
    MCUSerialInterface.write_line_peripheral("SensorThread", f"set.localization.h:{h}")

def get_variables(max_lines: int) -> list[tuple[str, str, Any]]:
    """Reads variables from serial and returns a list of (key, value_string, value)."""
    data_lines = MCUSerialInterface.read_lines(which_thread="SensorThread", max_lines=max_lines)
    # self.print(f"Read {len(data_lines)} lines from serial")
    if not data_lines:
        return []
    
    # Parse data lines
    variables: list[tuple[str, str, Any]] = []
    for line in data_lines:
        # Variables are formatted with comma delineators, e.g:
        # sensor.imu.roll:10,sensor.imu.pitch:30,sensor.imu.yaw:90,...
        split_variables = line.split(",")
        for variable in split_variables:
            split_var = variable.split(":", 1)
            if len(split_var) != 2:
                continue
            
            # Process split variable
            key, value = split_var
            entry = sensor_variables.get(key)
            if entry:
                try:
                    cast_value = entry.cast(value)
                    variables.append((key, value, cast_value))
                except ValueError:
                    print_from("SensorThread", f"Bad value for key {key}: {value}")
    return variables

__all__ = [
    "set_config",
    "reset_IMU",
    "reset_encoders",
    "reset_localization",
    "get_variables"
]

# --- Helper functions ---
def normalize_angle(radians: float) -> float:
    """Normalize an angle to the range [-pi, pi)."""
    while radians >= math.pi:
        radians -= 2 * math.pi
    while radians < -math.pi:
        radians += 2 * math.pi
    return radians