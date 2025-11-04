from utils.arduino_serial_interface import ArduinoSerialInterface
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from .arduino_serial_registers import serial_entries

class ArduinoSerialThread(PiThread):

    def _on_created_impl(self) -> None:
        pass

    def _on_start_impl(self) -> None:
        settings = load_settings()["arduino_serial"]
        self.max_lines_read_per_loop = settings["max_lines_read_per_loop"]
        self.print("Alive!")


    def _loop_impl(self) -> None:
        data_lines = ArduinoSerialInterface.read_lines(which_thread=self, max_lines=self.max_lines_read_per_loop)
        if not data_lines:
            return
        
        # Parse data lines
        for line in data_lines:
            split_line = line.split(":", 1)
            if len(split_line) != 2:
                continue

            key, value = split_line
            entry = serial_entries.get(key)
            if entry:
                try:
                    cast_value = entry.cast(value)
                    self[key] = cast_value # Write to global data
                    self.print(f"Received {key}: {self[key]} {type(self[key])}")
                except ValueError:
                    self.print(f"Bad value for key {key}: {value}")

    def _on_shutdown_impl(self) -> None:
        pass