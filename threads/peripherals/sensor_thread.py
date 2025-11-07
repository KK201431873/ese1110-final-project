from utils.arduino_serial_interface import ArduinoSerialInterface
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from .sensor_thread_registers import sensor_variables
import math

def normalize_angle(radians: float) -> float:
    """Normalize an angle to the range [-pi, pi)."""
    while radians >= math.pi:
        radians -= 2 * math.pi
    while radians < -math.pi:
        radians += 2 * math.pi
    return radians

class SensorThread(PiThread):
    ROBOT_X: float
    """Global X coordinate of the robot in meters."""

    ROBOT_Y: float
    """Global Y coordinate of the robot in meters."""

    ROBOT_H: float
    """Global heading of the robot in radians."""

    _max_lines_read_per_loop: int
    """Maximum number of lines to read from the serial interface per loop."""

    _last_encoder_left: float | None
    """Last recorded value of the left encoder in ticks."""

    _last_encoder_right: float | None
    """Last recorded value of the right encoder in ticks."""

    _ticks_per_rev: int
    """Encoder ticks per revolution of the wheel motor."""

    _gear_ratio: float
    """Gear ratio from the motor's gear to the wheel's gear."""

    _wheel_diameter: float
    """Diameter of the wheels (meters)."""

    _wheel_base: float
    """Distance between the left and right wheels (meters)."""

    _alpha: float
    """Filter coefficient between IMU yaw and motor encoder localization."""

    def _on_created_impl(self) -> None:
        self.ROBOT_X = 0.0
        self.ROBOT_Y = 0.0
        self.ROBOT_H = 0.0

        self._last_encoder_left = None
        self._last_encoder_right = None

        # Load sensor settings
        settings = load_settings()
        sensor_settings = settings["sensor_thread"]
        self._max_lines_read_per_loop = sensor_settings["max_lines_read_per_loop"]

        # Load drive config
        drive_settings = settings["drive_config"]
        self._ticks_per_rev = drive_settings["ticks_per_rev"]
        self._gear_ratio = drive_settings["gear_ratio"]
        self._wheel_diameter = drive_settings["wheel_diameter"]
        self._wheel_base = drive_settings["wheel_base"]
        self._alpha = drive_settings["alpha"]

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        data_lines = ArduinoSerialInterface.read_lines(which_thread=self, max_lines=self._max_lines_read_per_loop)
        if not data_lines:
            return
        
        # Parse data lines
        for line in data_lines:
            split_line = line.split(":", 1)
            if len(split_line) != 2:
                continue

            key, value = split_line
            entry = sensor_variables.get(key)
            if entry:
                try:
                    cast_value = entry.cast(value)
                    self[key] = cast_value # Write to global data
                    # self.print(f"Received {key}: {self[key]} {type(self[key])}")
                except ValueError:
                    self.print(f"Bad value for key {key}: {value}")
        
        # Localization
        self.do_tank_localization()
    
    def do_tank_localization(self):
        # Try reading values
        encoder_left = self["sensor.encoder.left"]
        encoder_right = self["sensor.encoder.right"]
        if encoder_left is None or encoder_right is None:
            return
        
        # Check if first loop
        if self._last_encoder_left is None or self._last_encoder_right is None:
            self._last_encoder_left = encoder_left
            self._last_encoder_right = encoder_right
            return
        
        # Calculate differences
        delta_left = encoder_left - self._last_encoder_left
        delta_right = encoder_right - self._last_encoder_right

        # Update last encoder values
        self._last_encoder_left = encoder_left
        self._last_encoder_right = encoder_right

        # Calculate distance traveled by each wheel
        distance_left = (delta_left / self._ticks_per_rev) * (math.pi * self._wheel_diameter) / self._gear_ratio
        distance_right = (delta_right / self._ticks_per_rev) * (math.pi * self._wheel_diameter) / self._gear_ratio

        # Calculate change in orientation and position
        delta_distance = (distance_left + distance_right) / 2
        delta_heading = (distance_right - distance_left) / self._wheel_base

        # Update robot's pose
        self.ROBOT_X += delta_distance * math.cos(self.ROBOT_H + delta_heading / 2)
        self.ROBOT_Y += delta_distance * math.sin(self.ROBOT_H + delta_heading / 2)
        self.ROBOT_H = normalize_angle(self.ROBOT_H + delta_heading)

        # Fuse with IMU yaw
        imu_yaw = self["sensor.imu.yaw"]
        if imu_yaw is not None:
            # Calculate weighted average of angles
            x_h, y_h = math.cos(self.ROBOT_H), math.sin(self.ROBOT_H)
            x_imu, y_imu = math.cos(imu_yaw), math.sin(imu_yaw)
            x_bar = self._alpha * x_imu + (1 - self._alpha) * x_h
            y_bar = self._alpha * y_imu + (1 - self._alpha) * y_h
            self.ROBOT_H = math.atan2(y_bar, x_bar)

        self["localization.x"] = self.ROBOT_X
        self["localization.y"] = self.ROBOT_Y
        self["localization.h"] = self.ROBOT_H

    def _on_shutdown_impl(self) -> None:
        ArduinoSerialInterface.close()