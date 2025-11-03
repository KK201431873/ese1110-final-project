from utils import ArduinoSerialInterface
from utils import PiThread
from arduino_serial_registers import serial_entries

class ArduinoSerialThread(PiThread):

    def _on_created_impl(self) -> None:
        pass

    def _on_start_impl(self) -> None:
        pass


    def _loop_impl(self) -> None:
        line = ArduinoSerialInterface.read_line(which_thread=self)
        if line:
            # Try to parse line
            split_line = line.split(":")
            if len(split_line) == 2:
                key, value = split_line
                entry = serial_entries.get(key)
                if entry:
                    try:
                        cast_value = entry.cast(value)
                        self[key] = cast_value # Write to global data
                    except ValueError:
                        self.print(f"Bad value for key {key}: {value}")
    

    def _on_shutdown_impl(self) -> None:
        pass