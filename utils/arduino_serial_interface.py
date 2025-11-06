from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.debug import print_from, raise_error_from
import threading
import serial

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

    @classmethod
    def _ensure_serial(cls, which_thread: type[PiThread] | PiThread | str) -> None:
        """Ensure serial port is open."""
        with cls._lock:
            if cls._ser is None or not cls._ser.is_open:
                try:
                    cls._ser = serial.Serial(port=cls._serial_port,
                                            baudrate=cls._baudrate,
                                            timeout=1)
                except serial.SerialException as e:
                    raise_error_from(
                        which_thread, 
                        RuntimeError, f"Failed to open serial port {cls._serial_port}: {e}"
                    )

    @classmethod
    def read_lines(cls, 
                   which_thread: type[PiThread] | PiThread | str, 
                   max_lines: int = 10) -> list[str]:
        """Thread-safe read up to `max_lines` data lines from Arduino serial channel."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_serial() in read_line()"
            )
            return []
        
        # Try reading data
        lines: list[str] = []
        with cls._lock:
            try:
                # Non-blocking read: check how many bytes are available
                while cls._ser.in_waiting > 0 and len(lines) < max_lines:
                    raw = cls._ser.read(cls._ser.in_waiting or 1)
                    if not raw:
                        break
                    
                    # Split on newlines (handle partial reads)
                    decoded = raw.decode(errors="ignore")
                    split_lines = decoded.splitlines()
                    lines.extend(split_lines)
                    
                    # Stop early if we already have enough lines
                    if len(lines) >= max_lines:
                        break

            except Exception as e:
                print_from(which_thread, f"Error reading from serial: {e}")
                cls._ser = None
                return []
            
        # Trim to max_lines
        return lines[:max_lines]
        
    @classmethod
    def write_line(cls, which_thread: type[PiThread] | PiThread | str, data: str) -> None:
        """Thread-safe write a line to Arduino serial channel."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_serial() in write_line()"
            )
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