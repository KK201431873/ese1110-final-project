from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from threads.vision.camera_thread import CameraThread
from enum import Enum
from . import controller_serial_interface as controller
import time

class State(Enum):
    SEARCH = 0
    PICKUP = 1
    TRANSFER = 2

class ControllerThread(PiThread):
    STATE: State
    """Current state of the controller."""

    target_relative_position: tuple[float, float]
    """Relative position of the target ball."""

    max_pickup_timeout: int
    """Maximum time without detecting a ball in PICKUP state before switching to SEARCH state (ms)."""

    _last_detection_time: float

    def _on_created_impl(self) -> None:
        self.STATE = State.SEARCH

        # Load config
        settings = load_settings()["controller_thread"]
        self.max_pickup_timeout = settings["max_pickup_timeout"]

        self.target_relative_position = (-1, -1)
        self._last_detection_time = 0.0

    def _on_start_impl(self) -> None:
        controller.stop_drive()

    def _loop_impl(self) -> None:
        # self.print(f"State: {self.STATE}, Target: {self.target_relative_position}")

        # Test send variables
        WebSocketInterface.send_variable(self, "controller.ball.forward", str(self.target_relative_position[0]))
        WebSocketInterface.send_variable(self, "controller.ball.left", str(self.target_relative_position[1]))
        match self.STATE:

            # The robot is searching for a ping-pong ball
            case State.SEARCH:
                # Turn counterclockwise until found ball
                controller.set_left_drive_speed(-0.5)
                controller.set_right_drive_speed(0.5)
                
                # State change logic
                closest_ball = self.get_closest_ball()
                if closest_ball is not None:
                    self.STATE = State.PICKUP
                    self.target_relative_position = closest_ball
                    self._last_detection_time = time.perf_counter()


            # The robot found a ping-pong ball, trying to pick it up now
            case State.PICKUP:
                closest_ball = self.get_closest_ball()
                time_since_last_detection = (time.perf_counter() - self._last_detection_time) * 1000
                if closest_ball is None:
                    if time_since_last_detection > self.max_pickup_timeout:
                        self.STATE = State.SEARCH
                    return
                self.target_relative_position = closest_ball
                self._last_detection_time = time.perf_counter()


            # The robot is transferring the ball to its bin
            case State.TRANSFER:
                # TODO: Implement transfer logic
                pass

            # This should never happen
            case _:
                pass

    def _on_shutdown_impl(self) -> None:
        controller.stop_drive()
    
    def get_closest_ball(self) -> tuple[float, float] | None:
        found_objects: list[tuple[float, float]] = CameraThread["detection.points"] or []
        if len(found_objects) == 0:
            return None
        closest_ball = min(found_objects, key=lambda p: p[0]**2 + p[1]**2)
        return closest_ball
