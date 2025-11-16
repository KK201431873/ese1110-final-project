from utils.mcu_serial_interface import MCUSerialInterface

# --- Controller functions ---
def drive_speed_angle(speed: float, angle: float) -> None:
    """Positive speed drives forward, positive angle turns CCW. Speed and angle must both be between `-1.0` and `1.0`."""
    speed = clamp(speed, -1.0, 1.0)
    angle = clamp(angle, -1.0, 1.0)
    left = clamp(speed - angle, -1.0, 1.0)
    right = clamp(speed + angle, -1.0, 1.0)
    set_left_drive_power(left)
    set_right_drive_power(right)

def set_left_drive_power(power: float) -> None:
    """Set the power of the left drive motor. Power must be between `-1.0` and `1.0`."""
    power = clamp(power, -1.0, 1.0)
    MCUSerialInterface.write_line("ControllerThread", f"command.drive.left:{power}")

def set_right_drive_power(power: float) -> None:
    """Set the power of the right drive motor. Power must be between `-1.0` and `1.0`."""
    power = clamp(power, -1.0, 1.0)
    MCUSerialInterface.write_line("ControllerThread", f"command.drive.right:{power}")

def stop_drive() -> None:
    """Set the power of both drive motors to zero."""
    set_left_drive_power(0.0)
    set_right_drive_power(0.0)

def set_intake_position(degrees: float) -> None:
    """Set the position of the intake servo. Position must be between `0.0` and `1.0`."""
    degrees = clamp(degrees, 0.0, 1.0)
    MCUSerialInterface.write_line("ControllerThread", f"command.intake.servo:{degrees}")

def set_intake_power(power: float) -> None:
    """Set the power of the intake motor. Power must be between `-1.0` and `1.0`."""
    power = clamp(power, -1.0, 1.0)
    MCUSerialInterface.write_line("ControllerThread", f"command.intake.motor:{power}")

__all__ = [
    "set_left_drive_power",
    "set_right_drive_power",
    "stop_drive",
    "set_intake_position",
    "set_intake_power"
]

# --- Helper functions ---
def clamp(x, lower, upper):
    return max(lower, min(x, upper))