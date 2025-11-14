from utils.mcu_serial_interface import MCUSerialInterface
from utils.websocket_interface import WebSocketInterface
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from utils.vector2 import Vector2
from utils.pose2 import Pose2
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
    ROBOT_POSE: Pose2
    """Global robot X and Y coordinates in meters and heading in radians."""

    _max_lines_read_per_loop: int
    """Maximum number of lines to read from the serial interface per loop."""

    _alpha: float
    """Filter coefficient between IMU yaw and motor encoder localization."""

    _last_encoder_left: float | None
    """Last recorded value of the left encoder in ticks."""

    _last_encoder_right: float | None
    """Last recorded value of the right encoder in ticks."""

    _ticks_per_rev: int
    """Encoder ticks per revolution of the wheel motor."""

    _gear_ratio: float
    """Gear ratio of motor:wheel."""

    _wheel_diameter: float
    """Diameter of the wheels (meters)."""

    _wheel_base: float
    """Distance between the left and right wheels (meters)."""

    def _on_created_impl(self) -> None:
        # Instantiate at the origin, facing east
        self.ROBOT_POSE = Pose2(
            Vector2(0.0, 0.0),
            0.0
        )
        self["localization.pose"] = self.ROBOT_POSE

        self._last_encoder_left = None
        self._last_encoder_right = None

        # Load sensor settings
        settings = load_settings()
        sensor_settings = settings["sensor_thread"]
        self._max_lines_read_per_loop = sensor_settings["max_lines_read_per_loop"]
        self._alpha = sensor_settings["alpha"]

        # Load drive config
        drive_settings = settings["drive_config"]
        self._ticks_per_rev = drive_settings["ticks_per_rev"]
        self._gear_ratio = drive_settings["gear_ratio"]
        self._wheel_diameter = drive_settings["wheel_diameter"]
        self._wheel_base = drive_settings["wheel_base"]

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        data_lines = MCUSerialInterface.read_lines(which_thread=self, max_lines=self._max_lines_read_per_loop)
        # self.print(f"Read {len(data_lines)} lines from serial")
        if not data_lines:
            return
        
        # Parse data lines
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
                        self[key] = cast_value # Write to global data
                        WebSocketInterface.send_variable(self, key, value)
                        # self.print(f"Received {key}: {self[key]} {type(self[key])}")
                    except ValueError:
                        self.print(f"Bad value for key {key}: {value}")
    
        # Share robot pose
        self.broadcast_robot_pose()

    def broadcast_robot_pose(self) -> None:
        self.ROBOT_POSE = self["localization.pose"] or self.ROBOT_POSE
        WebSocketInterface.send_variable(self, "localization.pose.x", str(round(self.ROBOT_POSE.COORDS.x, 3)))
        WebSocketInterface.send_variable(self, "localization.pose.y", str(round(self.ROBOT_POSE.COORDS.y, 3)))
        WebSocketInterface.send_variable(self, "localization.pose.h", str(round(math.degrees(self.ROBOT_POSE.h), 2)))

    def _on_shutdown_impl(self) -> None:
        MCUSerialInterface.close_all()