# read camera images, do apriltag detection, run object detector
from utils import PiThread
from threads.arduino_serial import ArduinoSerialThread

class CameraThread(PiThread):
    def _on_created_impl(self) -> None:
        pass

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        print_toggle = ArduinoSerialThread["print"]
        if print_toggle:
            self.print("hi")
        self.print(print_toggle)
    
    def _on_shutdown_impl(self) -> None:
        pass