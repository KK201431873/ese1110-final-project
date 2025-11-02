# read camera images, do apriltag detection, run object detector
from utils import PiThread
from threads.arduino_serial import ArduinoSerialThread

class CameraThread(PiThread):
    def _loop_impl(self) -> None:
        serial_thread_data = self.fetch_global_data(ArduinoSerialThread)
        print_toggle = serial_thread_data.get("print", False)
        if print_toggle:
            self.print("hi")