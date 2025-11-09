from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.debug import print_from, raise_error_from
import threading
import serial
import time

settings = load_settings()["arduino_serial_interface"]
serial_port: str = settings["serial_port"]
baudrate: int = settings["baudrate"]

class ArduinoSerialInterface():
    _serial_port: str = serial_port
    """USB on Raspberry Pi that Arduino is connected to."""

    _baudrate: int = baudrate
    """Sync with Arduino."""

    _ser: serial.Serial | None = None
    """Serial communication channel."""

    _lock: threading.Lock = threading.Lock()
    """Lock for accessing serial channel."""

    _buffer: bytes = b""
    """Accumulate bytes until a newline is read"""

    @classmethod
    def _ensure_serial(cls, which_thread: type[PiThread] | PiThread | str) -> None:
        """Ensure serial port is open."""
        with cls._lock:
            if cls._ser is None or not cls._ser.is_open:
                try:
                    cls._ser = serial.Serial(port=cls._serial_port,
                                            baudrate=cls._baudrate,
                                            timeout=0)  # non-blocking
                    cls._buffer = b""
                    time.sleep(0.1)
                except serial.SerialException as e:
                    raise_error_from(
                        which_thread, 
                        RuntimeError, f"Failed to open serial port {cls._serial_port}: {e}"
                    )

    @classmethod
    def read_lines(cls, 
                   which_thread: type[PiThread] | PiThread | str, 
                   max_lines: int = 10) -> list[str]:
        """Read up to `max_lines` complete lines (ending with '\n')."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            raise_error_from(which_thread, RuntimeError, f"Failed _ensure_serial() in read_lines()")
            return []
        
        # Try reading data
        lines: list[str] = []
        with cls._lock:
            try:
                # Read all available bytes (non-blocking)
                available = cls._ser.in_waiting
                if available > 0:
                    cls._buffer += cls._ser.read(available)

                # Split into lines based on newline terminator
                cls._buffer = cls._buffer.replace(b"\r", b"")
                if b"\n" in cls._buffer:
                    parts = cls._buffer.split(b"\n")
                    complete_lines = parts[:-1]  # all full lines
                    cls._buffer = parts[-1]      # leftover partial line

                    for raw_line in complete_lines:
                        decoded = raw_line.decode(errors="ignore").strip()
                        if decoded:
                            lines.append(decoded)
                            if len(lines) >= max_lines:
                                break

            except Exception as e:
                print_from(which_thread, f"Error reading from serial: {e}")
                cls._ser = None
                cls._buffer = b""
                return []

        return lines
        
    @classmethod
    def write_line(cls, which_thread: type[PiThread] | PiThread | str, data: str) -> None:
        """Thread-safe write a line to Arduino serial channel."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            raise_error_from(which_thread, RuntimeError, f"Failed _ensure_serial() in write_line()")
            return
        
        # Try writing data
        with cls._lock:
            try:
                cls._ser.write((data + "\n").encode())
            except Exception as e:
                print_from(which_thread, f"Error writing to serial: {e}")
                cls._ser = None  # force reinit on next read/write
    
    @classmethod
    def close(cls):
        """Close the serial port safely."""
        with cls._lock:
            if cls._ser and cls._ser.is_open:
                cls._ser.close()
            cls._ser = None