from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from utils.vector2 import Vector2
from utils.pose2 import Pose2
from threads.vision.camera_thread import CameraThread
from threads.peripherals.sensor_thread import SensorThread
from .algorithm.pid_controller import PIDController
from . import controller_serial_interface as controller
from enum import Enum
import time
import math

def normalize_angle(radians: float) -> float:
    """Normalize an angle to the range [-pi, pi)."""
    while radians >= math.pi:
        radians -= 2 * math.pi
    while radians < -math.pi:
        radians += 2 * math.pi
    return radians

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

    _ball_points: tuple[Vector2, Vector2] | None
    """(Relative, Absolute) points of the actively targeted ball."""

    _last_detection_time: float
    """Timestamp of the last detected ball."""

    _last_retarget_time: float
    """Timestamp of the last change in the target position."""

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

        self._ball_points = None
        self._last_detection_time = 0.0
        self._last_retarget_time = 0.0

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
                # controller.set_intake_position(0)
                # controller.set_intake_power(0.0)
                
                # Go PICKUP if a ball is detected
                self._ball_points = self.get_closest_ball_points()
                if self._ball_points is not None:
                    self._last_detection_time = now
                    self.STATE = State.PICKUP

                    # Prepare PID controller
                    self._pid_controller = PIDController(
                        self.kP, self.kI, self.kD
                    )


            # --- The robot found a ping-pong ball, trying to pick it up now ---
            case State.PICKUP:
                robot_pose: Pose2 | None = SensorThread["localization.pose"]

                # Handle ball tracking and fallback to SEARCH
                change_state, ball_detected = self.track_ball(now)
                if change_state or robot_pose is None or self._ball_points is None or self._pid_controller is None:
                    self._ball_points = None
                    self.STATE = State.SEARCH
                    return
                
                # Lower and run intake
                controller.set_intake_position(0.15)
                controller.set_intake_power(1.0)
                
                if ball_detected:
                    # Control heading and approach ball (convert absolute to relative coordinates using robot pose)
                    dx = self._ball_points[1].x - robot_pose.COORDS.x
                    dy = self._ball_points[1].y - robot_pose.COORDS.y
                    cos = math.cos(robot_pose.h)
                    sin = math.sin(robot_pose.h)

                    # Rotate displacement vector to get error
                    e_x =  dx*cos + dy*sin
                    e_y = -dx*sin + dy*cos

                    if abs(e_x) < 1e-3:
                        e_x = 1e-3 # very, very rare edge case
                    e_h = normalize_angle(math.atan2(e_y, e_x)) # Heading error (rad)
                    k = math.exp(-e_h*e_h*self.HEADING_PRIORITY)

                    # Control outputs
                    speed = k*self.MAX_PICKUP_SPEED
                    angle = self._pid_controller.update(e_h)
                else:
                    # Don't drive if no ball detected
                    speed = 0
                    angle = 0
                
                controller.drive_speed_angle(speed, angle)
                WebSocketInterface.send_variable(self, "control.output.speed", str(speed))
                WebSocketInterface.send_variable(self, "control.output.angle", str(angle))

            # --- The robot is transferring the ball to its bin ---
            case State.TRANSFER:
                
                # TODO: Implement transfer logic
                pass

            
            case _: # This should never happen
                pass
  
        # Share target data for minimap
        self["target_ball_points"] = self._ball_points # (Relative, Absolute) points of actively targeted ball


    def _on_shutdown_impl(self) -> None:
        controller.stop_drive()
    
    def get_closest_ball_points(self) -> tuple[Vector2, Vector2] | None:
        # Get and check points from camera (dictionary with equal-sized lists of relative and absolute ball detection coordinates)
        detection_points: dict[str, list[Vector2]] = CameraThread["detection.points"] or {
            "relative_points": [],
            "absolute_points": []
        }
        relative_points = detection_points["relative_points"]
        absolute_points = detection_points["absolute_points"]
        if len(relative_points)==0 or len(absolute_points)==0 or len(relative_points)!=len(absolute_points):
            return None
        
        # Extract closest point
        zipped_points = list(zip(relative_points, absolute_points)) # [(Relative, Absolute), ...] for each ball
        closest_ball_points = min(zipped_points, key=lambda p: p[0].norm()) # Use relative

        return closest_ball_points # (Relative, Absolute) coordinates of closest ball

    def track_ball(self, now: float) -> tuple[bool, bool]:
        """
        Runs ball-tracking logic.
        Args:
            now (float): the current loop time in seconds.
        Returns:
            (bool, bool): First argument True if need to switch back to SEARCH state, second
            argument True if ball has been detected this frame.
        """
        # This should never happen ----
        if self._ball_points is None or self._pid_controller is None:
            return True, False
        # -----------------------------

        # Update ball position
        closest_ball_points = self.get_closest_ball_points()

        if closest_ball_points is None:
            # Go SEARCH if no ball detected for a while
            if now - self._last_detection_time > self.NO_DETECTION_TIMEOUT:
                return True, False
            return False, False
        else:
            # Check if current detection timed out
            update_position = now - self._last_retarget_time > self.BALL_MAX_LIFESPAN
            
            # Check if ball moved (use absolute coordinates)
            update_position |= (self._ball_points[1] - closest_ball_points[1]).norm() > self.BALL_MAX_DRIFT

            # Update state
            if update_position:
                self._ball_points = closest_ball_points
                self._last_retarget_time = now
            # Always refresh detection time if ball visible
            self._last_detection_time = now
            
            return False, True # Detections are fine, no need for state switch
