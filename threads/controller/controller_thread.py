from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from threads.vision.camera_thread import CameraThread
from threads.peripherals.sensor_thread import SensorThread
from .algorithm.pid_controller import PIDController
from utils.vector2 import Vector2
from utils.pose2 import Pose2
from . import controller_serial_interface as controller
from enum import Enum
import time
import math

class State(Enum):
    SEARCH = 0
    PICKUP = 1
    TRANSFER = 2

class ControllerThread(PiThread):
    STATE: State
    """Current state of the controller."""

    NO_DETECTION_TIMEOUT: float
    """Maximum time without detecting a ball in PICKUP state before switching to SEARCH state (seconds)."""

    BALL_MAX_LIFESPAN: float
    """Maximum time before forcing a new target position (seconds)."""

    BALL_MAX_DRIFT: float
    """Maximum movement of the ball before forcing a new target position (meters)."""

    HEADING_PRIORITY: float
    """Priority for heading correction over driving forward during PICKUP. Must be positive."""
    
    MAX_PICKUP_SPEED: float
    """Proportion of maximum linear velocity to cap speed at during PICKUP."""

    kP: float
    """Proportional coefficient of PID controller."""
    kI: float
    """Integral coefficient of PID controller."""
    kD: float
    """Derivative coefficient of PID controller."""
    _pid_controller: PIDController | None
    """Instance of a PID controller."""

    _ball_position: Vector2 | None
    """Relative position of the target ball."""

    _last_detection_time: float
    """Timestamp of the last detected ball."""

    _last_retarget_time: float
    """Timestamp of the last change in the target position."""

    _left_power: float
    """Left wheel speed."""

    _right_power: float
    """Right wheel speed."""

    _POWER_TO_SPEED: float
    """Conversion factor from power to speed."""

    def _on_created_impl(self) -> None:
        self.STATE = State.SEARCH

        # Load config
        settings = load_settings()
        controller_settings = settings["controller_thread"]
        self.NO_DETECTION_TIMEOUT = controller_settings["NO_DETECTION_TIMEOUT"]
        self.BALL_MAX_LIFESPAN = controller_settings["BALL_MAX_LIFESPAN"]
        self.BALL_MAX_DRIFT = controller_settings["BALL_MAX_DRIFT"]
        self.HEADING_PRIORITY = controller_settings["HEADING_PRIORITY"]
        self.MAX_PICKUP_SPEED = controller_settings["MAX_PICKUP_SPEED"]
        self.kP = controller_settings["kP"]
        self.kI = controller_settings["kI"]
        self.kD = controller_settings["kD"]

        self._ball_position = None
        self._last_detection_time = 0.0
        self._last_retarget_time = 0.0

        self._POWER_TO_SPEED = settings["drive_config"]["v_max"]

        self._pid_controller = None

    def _on_start_impl(self) -> None:
        controller.stop_drive()
        self.print("Alive!")

    def _loop_impl(self) -> None:
        now = time.perf_counter()
        
        # self.print(f"State: {self.STATE}, Target: {self.target_relative_position}")

        # Test send variables
        # if self._target_absolute_position is not None:
        #     WebSocketInterface.send_variable(self, "controller.ball.x", str(self._target_absolute_position.x))
        #     WebSocketInterface.send_variable(self, "controller.ball.y", str(self._target_absolute_position.y))

        # === State machine logic ===
        match self.STATE:

            # --- The robot is searching for a ping-pong ball ---
            case State.SEARCH:
                # Turn counterclockwise until found ball
                # controller.set_left_drive_power(-0.25)
                # controller.set_right_drive_power(0.25)

                # Raise and stop intake
                # controller.set_intake_position(90)
                # controller.set_intake_power(0.0)
                
                # Go PICKUP if a ball is detected
                closest_ball = self.get_closest_ball()
                if closest_ball is not None:
                    self._ball_position = closest_ball
                    self._last_detection_time = now
                    self.STATE = State.PICKUP

                    # Prepare PID controller
                    self._pid_controller = PIDController(
                        self.kP, self.kI, self.kD
                    )


            # --- The robot found a ping-pong ball, trying to pick it up now ---
            case State.PICKUP:
                # Handle ball tracking and fallback to SEARCH
                if self.track_ball(now) or self._ball_position is None or self._pid_controller is None:
                    self._ball_position = None
                    self.STATE = State.SEARCH
                    return

                # Lower and run intake
                controller.set_intake_position(45)
                controller.set_intake_power(1.0)

                # Control heading and approach ball
                bx, by = self._ball_position.x, self._ball_position.y
                e_h = math.atan2(by, bx) # Heading error (rad)
                k = math.exp(-e_h*e_h*self.HEADING_PRIORITY)

                # Control outputs
                speed = k*self.MAX_PICKUP_SPEED
                angle = self._pid_controller.update(e_h)
                controller.drive_speed_angle(speed, angle)
            

            # --- The robot is transferring the ball to its bin ---
            case State.TRANSFER:
                # TODO: Implement transfer logic
                pass

            
            case _: # This should never happen
                pass

    def _on_shutdown_impl(self) -> None:
        controller.stop_drive()
    
    def get_closest_ball(self) -> Vector2 | None:
        relative_points: list[Vector2] = CameraThread["detection.relative_points"] or []
        if len(relative_points) == 0:
            return None
        closest_ball = min(relative_points, key=lambda p: p.norm())
        return closest_ball

    def track_ball(self, now: float) -> bool:
        """
        Runs ball-tracking logic.
        Args:
            now (float): the current loop time in seconds.
        Returns:
            change_state (bool): True if need to switch back to SEARCH state.
        """
        # This should never happen ----
        if self._ball_position is None or self._pid_controller is None:
            return True
        # -----------------------------

        # Update ball position
        closest_ball = self.get_closest_ball()

        if closest_ball is None:
            # Go SEARCH if no ball detected for a while
            if now - self._last_detection_time > self.NO_DETECTION_TIMEOUT:
                return True
        else:
            # Check if ball moved
            update_position = (self._ball_position - closest_ball).norm() > self.BALL_MAX_DRIFT
            
            # Check if current detection timed out
            update_position |= now - self._last_retarget_time > self.BALL_MAX_LIFESPAN

            # Update state
            if update_position:
                self._ball_position = closest_ball
                self._last_retarget_time = now
            # Always refresh detection time if ball visible
            self._last_detection_time = now
            
        return False # Detections are fine, no need for state switch
