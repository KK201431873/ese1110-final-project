from utils.arduino_serial_interface import ArduinoSerialInterface

# --- Controller functions ---
def set_left_drive_power(power: float) -> None:
    """Set the power of the left drive motor. Power must be between -1.0 and 1.0."""
    power = clamp(power, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.drive.left:{power}")

def set_right_drive_power(power: float) -> None:
    """Set the power of the right drive motor. Power must be between -1.0 and 1.0."""
    power = clamp(power, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.drive.right:{power}")

def stop_drive() -> None:
    """Set the power of both drive motors to zero."""
    set_left_drive_power(0.0)
    set_right_drive_power(0.0)

def set_intake_position(degrees: int) -> None:
    """Set the position (in degrees) of the intake servo. Position must be between 0 and 180."""
    degrees = clamp(degrees, 0, 180)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.intake.servo:{degrees}")

def set_intake_power(power: float) -> None:
    """Set the power of the intake motor. Power must be between -1.0 and 1.0."""
    power = clamp(power, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.intake.motor:{power}")

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