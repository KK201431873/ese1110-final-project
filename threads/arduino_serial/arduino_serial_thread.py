# serial comms with Arduino
from utils import PiThread
import time

class ArduinoSerialThread(PiThread):
    print_toggle: bool = True
    last_time: float = -1

    def _on_created_impl(self) -> None:
        self["print"] = self.print_toggle

    def _on_start_impl(self) -> None:
        self.last_time = time.perf_counter()
        self.print("Alive!")


    def _loop_impl(self) -> None:
        now = time.perf_counter()
        if now - self.last_time >= 1.0:
            self.print_toggle = not self.print_toggle
            self["print"] = self.print_toggle
            self.last_time = now
    

    def _on_shutdown_impl(self) -> None:
        pass