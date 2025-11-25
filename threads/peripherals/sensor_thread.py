from utils.mcu_serial_interface import MCUSerialInterface
from utils.websocket_interface import WebSocketInterface
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from utils.vector2 import Vector2
from utils.pose2 import Pose2
from . import sensor_serial_interface as sensors
import math

class SensorThread(PiThread):
    ROBOT_POSE: Pose2
    """Global robot X and Y coordinates in meters and heading in radians."""

    _max_lines_read_per_loop: int
    """Maximum number of lines to read from the serial interface per loop."""

    _alpha: float
    """Filter coefficient between IMU yaw and motor encoder localization."""

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
        # Set config and reset sensors
        sensors.set_config(
            wheel_diameter=self._wheel_diameter,
            wheel_base=self._wheel_base,
            ticks_per_rev=self._ticks_per_rev,
            gear_ratio=self._gear_ratio,
            alpha=self._alpha
        )
        sensors.reset_IMU()
        sensors.reset_encoders()
        sensors.reset_localization()

        self.print("Alive!")

    def _loop_impl(self) -> None:
        variables = sensors.get_variables(max_lines=self._max_lines_read_per_loop)
        for key, value_str, value in variables:
            self[key] = value # Write to global data
            WebSocketInterface.send_variable(self, key, value_str)
            # self.print(f"Received {key}: {self[key]} {type(self[key])}")
    
        # Share robot pose
        self.broadcast_robot_pose()

    def broadcast_robot_pose(self) -> None:
        self.ROBOT_POSE = self["localization.pose"] or self.ROBOT_POSE
        WebSocketInterface.send_variable(self, "localization.pose.x", str(round(self.ROBOT_POSE.COORDS.x, 3)))
        WebSocketInterface.send_variable(self, "localization.pose.y", str(round(self.ROBOT_POSE.COORDS.y, 3)))
        WebSocketInterface.send_variable(self, "localization.pose.h", str(round(math.degrees(self.ROBOT_POSE.h), 2)))

    def _on_shutdown_impl(self) -> None:
        MCUSerialInterface.close_all()