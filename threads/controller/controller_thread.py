from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from threads.vision.camera_thread import CameraThread
from threads.peripherals.sensor_thread import SensorThread
from .algorithm.ramsete_controller import RAMSETEController
from utils.vector2 import Vector2
from utils.pose2 import Pose2
from . import controller_serial_interface as controller
from enum import Enum
import time

class State(Enum):
    SEARCH = 0
    PICKUP = 1
    TRANSFER = 2

class ControllerThread(PiThread):
    ROBOT_POSE: Pose2 | None

    STATE: State
    """Current state of the controller."""

    _target_absolute_position: Vector2 | None
    """Relative position of the target ball."""

    _max_pickup_timeout: int
    """Maximum time without detecting a ball in PICKUP state before switching to SEARCH state (ms)."""

    _last_detection_time: float
    """Timestamp of the last detected ball."""

    _left_power: float
    """Left wheel speed."""

    _right_power: float
    """Right wheel speed."""

    _POWER_TO_SPEED: float
    """Conversion factor from power to speed."""

    _ramsete_controller: RAMSETEController
    """Instance of the RAMSETE controller."""

    def _on_created_impl(self) -> None:
        self.STATE = State.SEARCH

        # Load config
        settings = load_settings()
        self._target_absolute_position = None
        self._max_pickup_timeout = settings["controller_thread"]["max_pickup_timeout"]
        self._last_detection_time = 0.0
        self._left_power = 0.0
        self._right_power = 0.0
        self._POWER_TO_SPEED = settings["drive_config"]["v_max"]

        # Initialize RAMSETE controller
        self._ramsete_controller = RAMSETEController(
            b = settings["controller_thread"]["b"], 
            zeta = settings["controller_thread"]["zeta"], 
            v_max = settings["drive_config"]["v_max"], 
            a_max = settings["drive_config"]["a_max"],
            replanning_max_delay = settings["controller_thread"]["replanning_max_delay"], 
            replanning_drift_threshold = settings["controller_thread"]["replanning_drift_threshold"],
            lookahead_time = settings["controller_thread"]["lookahead_time"], 
            wheel_diameter = settings["drive_config"]["wheel_diameter"], 
            wheel_base = settings["drive_config"]["wheel_base"]
        )

    def _on_start_impl(self) -> None:
        controller.stop_drive()
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Get robot pose
        self.ROBOT_POSE = SensorThread["localization.pose"]
        
        # self.print(f"State: {self.STATE}, Target: {self.target_relative_position}")

        # Test send variables
        # if self._target_absolute_position is not None:
        #     WebSocketInterface.send_variable(self, "controller.ball.x", str(self._target_absolute_position.x))
        #     WebSocketInterface.send_variable(self, "controller.ball.y", str(self._target_absolute_position.y))

        # --- State machine logic ---
        match self.STATE:

            # The robot is searching for a ping-pong ball
            case State.SEARCH:
                # Turn counterclockwise until found ball
                # controller.set_left_drive_power(-0.25)
                # controller.set_right_drive_power(0.25)

                # Raise and stop intake
                # controller.set_intake_position(90)
                # controller.set_intake_power(0.0)
                
                # Go PICKUP if a ball is detected
                closest_ball = self.get_closest_ball()
                self._target_absolute_position = closest_ball
                if closest_ball is not None:
                    self.STATE = State.PICKUP
                    self._last_detection_time = time.perf_counter()


            # The robot found a ping-pong ball, trying to pick it up now
            case State.PICKUP:
                # Lower and run intake
                # controller.set_intake_position(45)
                # controller.set_intake_power(1.0)

                # Go SEARCH if no ball detected for a while
                closest_ball = self.get_closest_ball()
                time_since_last_detection = (time.perf_counter() - self._last_detection_time) * 1000
                if closest_ball is None:
                    if time_since_last_detection > self._max_pickup_timeout:
                        self._target_absolute_position = None
                        self.STATE = State.SEARCH
                    return
                self._target_absolute_position = closest_ball
                self._last_detection_time = time.perf_counter()

                # Run RAMSETE 
                if self.ROBOT_POSE is not None:

                    v0 = self._POWER_TO_SPEED * (self._left_power + self._right_power) / 2

                    left, right = self._ramsete_controller.update_controller(
                        ball_pos = self._target_absolute_position,
                        robot_pose = self.ROBOT_POSE,
                        current_vel = Vector2(0, 0) # TODO: Estimate velocity and plug in here
                    )

                    left_power = left / self._POWER_TO_SPEED
                    right_power = right / self._POWER_TO_SPEED

                    # controller.set_left_drive_power(left_power)
                    # controller.set_right_drive_power(right_power)
                else:
                    # controller.stop_drive()

                    pass


            # The robot is transferring the ball to its bin
            case State.TRANSFER:
                # TODO: Implement transfer logic
                pass

            # This should never happen
            case _:
                pass

    def _on_shutdown_impl(self) -> None:
        controller.stop_drive()
    
    def get_closest_ball(self) -> Vector2 | None:
        found_objects: list[Vector2] = CameraThread["detection.points"] or []
        if len(found_objects) == 0 or self.ROBOT_POSE is None:
            return None
        def key(p: Vector2) -> float:
            if self.ROBOT_POSE is None:
                return -1
            return (p - self.ROBOT_POSE.COORDS).norm()
        closest_ball = min(found_objects, key=key)
        return closest_ball
