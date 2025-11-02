# read camera images, do apriltag detection, run object detector
from utils import PiThread

class CameraThread(PiThread):
    def _loop_impl(self) -> None:
        self.print("hi")