from utils.arduino_serial_interface import ArduinoSerialInterface

# --- Controller functions ---
def set_left_drive_speed(speed: float) -> None:
    """Set the speed of the left drive motor. Speed must be between -1.0 and 1.0."""
    speed = clamp(speed, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.drive.left:{speed}")

def set_right_drive_speed(speed: float) -> None:
    """Set the speed of the right drive motor. Speed must be between -1.0 and 1.0."""
    speed = clamp(speed, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.drive.right:{speed}")

def stop_drive() -> None:
    """Set the speed of both drive motors to zero."""
    set_left_drive_speed(0.0)
    set_right_drive_speed(0.0)

def set_intake_position(position: float) -> None:
    """Set the position of the intake servo. Position must be between 0.0 and 1.0."""
    position = clamp(position, 0.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.intake.servo:{position}")

def set_intake_speed(speed: float) -> None:
    """Set the speed of the intake motor. Speed must be between -1.0 and 1.0."""
    speed = clamp(speed, -1.0, 1.0)
    ArduinoSerialInterface.write_line("ControllerThread", f"command.intake.motor:{speed}")

__all__ = [
    "set_left_drive_speed",
    "set_right_drive_speed",
    "stop_drive",
    "set_intake_position",
    "set_intake_speed"
]

# --- Helper functions ---
def clamp(x, lower, upper):
    return max(lower, min(x, upper))