from utils import ArduinoSerialInterface
from utils import PiThread
from .arduino_serial_registers import serial_entries

class ArduinoSerialThread(PiThread):

    def _on_created_impl(self) -> None:
        pass

    def _on_start_impl(self) -> None:
        self.print("Alive!")


    def _loop_impl(self) -> None:
        data = ArduinoSerialInterface.read_line(which_thread=self)
        if data:
            # Try to parse line
            split_line = data.split(":", 1)
            if len(split_line) == 2:
                key, value = split_line
                entry = serial_entries.get(key)
                if entry:
                    try:
                        cast_value = entry.cast(value)
                        self[key] = cast_value # Write to global data
                        self.print(f"Received {key}: {self[key]} {type(self[key])}")
                    except ValueError:
                        self.print(f"Bad value for key {key}: {value}")
            else:
                self.print(f"Bad line: {data}") 
    

    def _on_shutdown_impl(self) -> None:
        pass