from utils.pi_thread import PiThread
from threads.vision.camera_thread import CameraThread
from enum import Enum

# TODO: FIX THIS IMPORT
# import controller_serial_interface as controller

class State(Enum):
    SEARCH = 0
    PICKUP = 1
    TRANSFER = 2

class ControllerThread(PiThread):
    STATE: State

    def _on_created_impl(self) -> None:
        self.STATE = State.SEARCH

    def _on_start_impl(self) -> None:
        # controller.stop_drive()
        pass

    def _loop_impl(self) -> None:
        match self.STATE:

            # The robot is searching for a ping-pong ball
            case State.SEARCH:
                # Turn counterclockwise until found ball
                # controller.set_left_drive_speed(-0.5)
                # controller.set_right_drive_speed(0.5)
                # TODO: actually implement search logic
                found_objects = CameraThread["detection.points"] or []
                if len(found_objects) > 0:
                    self.STATE = State.PICKUP

            # The robot found a ping-pong ball, trying to pick it up now
            case State.PICKUP:
                # TODO: Implement pickup logic
                pass

            # The robot is transferring the ball to its bin
            case State.TRANSFER:
                # TODO: Implement transfer logic
                pass

    def _on_shutdown_impl(self) -> None:
        # controller.stop_drive()
        pass
