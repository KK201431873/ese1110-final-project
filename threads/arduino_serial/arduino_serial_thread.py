# serial comms with Arduino
from utils import PiThread
import time

class ArduinoSerialThread(PiThread):
    print_toggle: bool = True
    last_time: float = time.perf_counter()

    def _loop_impl(self) -> None:
        now = time.perf_counter()
        if now - self.last_time >= 1.0:
            self.write_global_data("print", self.print_toggle)
            self.print_toggle = not self.print_toggle
            self.last_time = now